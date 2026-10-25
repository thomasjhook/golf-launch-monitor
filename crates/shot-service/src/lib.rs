//! gRPC shot-processing service.
//!
//! Services (package `launchmonitor.v1`): `ShotService`,
//! `CalibrationService`, plus the standard `grpc.health.v1.Health` and server
//! reflection (v1 and v1alpha) so `grpcurl` works without proto files.
//!
//! State is deliberately small: the active camera setup (persisted to
//! `DATA_DIR/setup.pb` when a data dir is configured), a [`ShotStore`] holding
//! shot history (Postgres in deployment), and a broadcast channel that fans
//! processed shots out to `StreamShots` subscribers.

pub mod store;

use std::future::Future;
use std::path::PathBuf;
use std::pin::Pin;
use std::sync::{Arc, RwLock};

use calibration::Calibration;
use launch_core::{FlashOrder, Pipeline};
use launch_proto::convert::{flash_order_from_i32, ConvertError};
use launch_proto::pb::calibration_service_server::{CalibrationService, CalibrationServiceServer};
use launch_proto::pb::shot_service_server::{ShotService, ShotServiceServer};
use launch_proto::pb::{
    self, CameraSetup, GetSetupRequest, ListShotsRequest, ListShotsResponse, ProcessShotRequest,
    ProcessShotResponse, StreamShotsRequest,
};
use prost::Message;
use tokio::sync::broadcast;
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;
use tokio_stream::wrappers::{BroadcastStream, TcpListenerStream};
use tokio_stream::{Stream, StreamExt};
use tonic::transport::Server;
use tonic::{Request, Response, Status};
use vision::GrayImage;

use store::{MemoryStore, ShotStore};

/// Largest request a client may send. A 5 MP 8-bit frame is ~5 MB; tonic's
/// default of 4 MB is too tight for real cameras.
pub const MAX_REQUEST_BYTES: usize = 32 * 1024 * 1024;

/// Page size when the client sends 0, and the most a client may ask for.
const DEFAULT_PAGE_SIZE: u32 = 50;
const MAX_PAGE_SIZE: u32 = 500;
const SETUP_FILE: &str = "setup.pb";
const RESULT_CHANNEL_CAPACITY: usize = 64;

#[derive(Clone)]
struct Setup {
    calibration: Calibration,
    flash_order: FlashOrder,
}

impl Setup {
    fn from_wire(msg: CameraSetup) -> Result<Self, ConvertError> {
        let calibration = msg
            .calibration
            .ok_or(ConvertError::Missing("calibration"))?
            .try_into()?;
        Ok(Self {
            calibration,
            flash_order: flash_order_from_i32(msg.flash_order)?,
        })
    }

    fn to_wire(&self) -> CameraSetup {
        CameraSetup {
            calibration: Some((&self.calibration).into()),
            flash_order: pb::FlashOrder::from(self.flash_order) as i32,
        }
    }
}

/// Shared service state.
pub struct AppState {
    setup: RwLock<Option<Setup>>,
    setup_path: Option<PathBuf>,
    results: broadcast::Sender<pb::ShotResult>,
    store: Arc<dyn ShotStore>,
}

impl AppState {
    /// Create state, loading a previously persisted setup from `data_dir` if
    /// one exists. A corrupt file is logged and ignored rather than fatal.
    ///
    /// Shot history is kept in memory only; see [`AppState::with_store`].
    pub fn new(data_dir: Option<PathBuf>) -> Arc<Self> {
        Self::with_store(data_dir, Arc::new(MemoryStore::default()))
    }

    /// Like [`AppState::new`] with an explicit shot-history store.
    pub fn with_store(data_dir: Option<PathBuf>, store: Arc<dyn ShotStore>) -> Arc<Self> {
        let setup_path = data_dir.map(|d| d.join(SETUP_FILE));
        let setup = setup_path.as_deref().and_then(load_setup);
        let (results, _) = broadcast::channel(RESULT_CHANNEL_CAPACITY);
        Arc::new(Self {
            setup: RwLock::new(setup),
            setup_path,
            results,
            store,
        })
    }

    fn setup(&self) -> Option<Setup> {
        self.setup.read().expect("setup lock poisoned").clone()
    }
}

fn load_setup(path: &std::path::Path) -> Option<Setup> {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return None,
        Err(e) => {
            tracing::warn!(path = %path.display(), error = %e, "cannot read stored setup");
            return None;
        }
    };
    let setup = CameraSetup::decode(bytes.as_slice())
        .map_err(|e| e.to_string())
        .and_then(|m| Setup::from_wire(m).map_err(|e| e.to_string()));
    match setup {
        Ok(s) => {
            tracing::info!(path = %path.display(), "loaded stored camera setup");
            Some(s)
        }
        Err(e) => {
            tracing::warn!(path = %path.display(), error = %e, "ignoring invalid stored setup");
            None
        }
    }
}

async fn persist_setup(path: &std::path::Path, setup: &Setup) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        tokio::fs::create_dir_all(dir).await?;
    }
    // Write-then-rename so a crash never leaves a half-written file.
    let tmp = path.with_extension("pb.tmp");
    tokio::fs::write(&tmp, setup.to_wire().encode_to_vec()).await?;
    tokio::fs::rename(&tmp, path).await
}

pub struct ShotSvc {
    state: Arc<AppState>,
}

type ShotStream = Pin<Box<dyn Stream<Item = Result<pb::ShotResult, Status>> + Send>>;

#[tonic::async_trait]
impl ShotService for ShotSvc {
    async fn process_shot(
        &self,
        request: Request<ProcessShotRequest>,
    ) -> Result<Response<ProcessShotResponse>, Status> {
        let req = request.into_inner();
        let frame: GrayImage = req
            .frame
            .ok_or(ConvertError::Missing("frame"))?
            .try_into()?;
        let timing = req.timing.ok_or(ConvertError::Missing("timing"))?.into();
        let setup = self
            .state
            .setup()
            .ok_or_else(|| Status::failed_precondition("no camera setup stored; call SetSetup"))?;

        // Detection is CPU-bound; keep it off the async workers.
        let result = tokio::task::spawn_blocking(move || {
            Pipeline::new(setup.calibration, setup.flash_order).process(&frame, &timing)
        })
        .await
        .map_err(|e| Status::internal(format!("pipeline task failed: {e}")))?
        .map_err(|e| Status::failed_precondition(e.to_string()))?;

        let wire = pb::ShotResult::from(&result);
        tracing::info!(
            shot_id = wire.shot_id,
            speed_mps = result.launch.ball_speed_mps,
            carry_m = result.flight.carry_m,
            "shot processed"
        );
        // History is best effort: losing the database must not lose the shot
        // the golfer just hit.
        if let Err(e) = self.state.store.save(&wire).await {
            tracing::error!(shot_id = wire.shot_id, error = %e, "failed to store shot");
        }
        // An error here only means nobody is subscribed.
        let _ = self.state.results.send(wire.clone());
        Ok(Response::new(ProcessShotResponse { result: Some(wire) }))
    }

    type StreamShotsStream = ShotStream;

    async fn stream_shots(
        &self,
        _request: Request<StreamShotsRequest>,
    ) -> Result<Response<Self::StreamShotsStream>, Status> {
        let stream = BroadcastStream::new(self.state.results.subscribe()).filter_map(|r| match r {
            Ok(shot) => Some(Ok(shot)),
            Err(BroadcastStreamRecvError::Lagged(n)) => {
                tracing::warn!(missed = n, "stream subscriber lagged; skipping");
                None
            }
        });
        Ok(Response::new(Box::pin(stream)))
    }

    async fn list_shots(
        &self,
        request: Request<ListShotsRequest>,
    ) -> Result<Response<ListShotsResponse>, Status> {
        let req = request.into_inner();
        if req.before_id < 0 {
            return Err(Status::invalid_argument("before_id must be >= 0"));
        }
        let limit = match req.page_size {
            0 => DEFAULT_PAGE_SIZE,
            n => n.min(MAX_PAGE_SIZE),
        };
        let before = (req.before_id > 0).then_some(req.before_id);
        let shots = self.state.store.list(limit, before).await.map_err(|e| {
            tracing::error!(error = %e, "failed to list shots");
            Status::unavailable("shot history is unavailable")
        })?;
        Ok(Response::new(ListShotsResponse { shots }))
    }
}

pub struct CalSvc {
    state: Arc<AppState>,
}

#[tonic::async_trait]
impl CalibrationService for CalSvc {
    async fn get_setup(
        &self,
        _request: Request<GetSetupRequest>,
    ) -> Result<Response<CameraSetup>, Status> {
        self.state
            .setup()
            .map(|s| Response::new(s.to_wire()))
            .ok_or_else(|| Status::not_found("no camera setup stored"))
    }

    async fn set_setup(
        &self,
        request: Request<CameraSetup>,
    ) -> Result<Response<CameraSetup>, Status> {
        let setup = Setup::from_wire(request.into_inner())?;
        // Persist before applying so memory and disk can't disagree.
        if let Some(path) = &self.state.setup_path {
            persist_setup(path, &setup).await.map_err(|e| {
                tracing::error!(path = %path.display(), error = %e, "cannot persist setup");
                Status::internal("failed to persist camera setup")
            })?;
        }
        let wire = setup.to_wire();
        *self.state.setup.write().expect("setup lock poisoned") = Some(setup);
        tracing::info!("camera setup updated");
        Ok(Response::new(wire))
    }
}

/// Serve all services on `listener` until `shutdown` resolves.
pub async fn serve(
    listener: tokio::net::TcpListener,
    state: Arc<AppState>,
    shutdown: impl Future<Output = ()> + Send + 'static,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let (health, health_service) = tonic_health::server::health_reporter();
    health.set_serving::<ShotServiceServer<ShotSvc>>().await;
    health
        .set_serving::<CalibrationServiceServer<CalSvc>>()
        .await;

    let reflection = || {
        tonic_reflection::server::Builder::configure()
            .register_encoded_file_descriptor_set(launch_proto::FILE_DESCRIPTOR_SET)
            .register_encoded_file_descriptor_set(tonic_health::pb::FILE_DESCRIPTOR_SET)
    };

    Server::builder()
        .add_service(health_service)
        .add_service(reflection().build_v1()?)
        .add_service(reflection().build_v1alpha()?)
        .add_service(
            ShotServiceServer::new(ShotSvc {
                state: state.clone(),
            })
            .max_decoding_message_size(MAX_REQUEST_BYTES),
        )
        .add_service(CalibrationServiceServer::new(CalSvc { state }))
        .serve_with_incoming_shutdown(TcpListenerStream::new(listener), shutdown)
        .await?;
    Ok(())
}
