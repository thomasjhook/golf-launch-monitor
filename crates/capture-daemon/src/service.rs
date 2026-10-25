//! gRPC client side: upload shots to `shot-service`, install the bench setup.

use std::future::Future;
use std::time::Duration;

use launch_proto::pb::calibration_service_client::CalibrationServiceClient;
use launch_proto::pb::shot_service_client::ShotServiceClient;
use launch_proto::pb::{self, CameraSetup, ProcessShotRequest};
use launch_types::StrobeTiming;
use tonic::transport::{Channel, Endpoint};
use tonic::{Code, Status};
use vision::GrayImage;

use crate::error::{Error, Result};

/// Bounded retry with exponential backoff, for transient errors only.
#[derive(Debug, Clone, Copy)]
pub struct RetryPolicy {
    /// Total attempts including the first.
    pub max_attempts: u32,
    pub initial_backoff: Duration,
    pub max_backoff: Duration,
}

impl Default for RetryPolicy {
    fn default() -> Self {
        Self {
            max_attempts: 4,
            initial_backoff: Duration::from_millis(100),
            max_backoff: Duration::from_secs(2),
        }
    }
}

/// Only these are worth retrying. INVALID_ARGUMENT / FAILED_PRECONDITION (bad
/// frame, flash-count mismatch, no setup stored) fail identically every time.
pub fn is_transient(code: Code) -> bool {
    matches!(code, Code::Unavailable | Code::DeadlineExceeded)
}

pub async fn retry_transient<T, Fut>(
    policy: RetryPolicy,
    mut op: impl FnMut() -> Fut,
) -> Result<T, Status>
where
    Fut: Future<Output = Result<T, Status>>,
{
    let mut backoff = policy.initial_backoff;
    let mut attempt = 1;
    loop {
        match op().await {
            Ok(v) => return Ok(v),
            Err(s) if is_transient(s.code()) && attempt < policy.max_attempts => {
                tracing::warn!(attempt, error = %s, "transient gRPC error; retrying");
                tokio::time::sleep(backoff).await;
                backoff = (backoff * 2).min(policy.max_backoff);
                attempt += 1;
            }
            Err(s) => return Err(s),
        }
    }
}

/// Create a channel without connecting yet, so a service outage at startup
/// does not stop capture (shots are archived first and retried on upload).
pub fn connect_lazy(addr: &str) -> Result<Channel> {
    let endpoint = Endpoint::from_shared(addr.to_string())
        .map_err(|e| Error::Setup(format!("bad service address {addr:?}: {e}")))?
        .connect_timeout(Duration::from_secs(2))
        .timeout(Duration::from_secs(30));
    Ok(endpoint.connect_lazy())
}

#[derive(Clone)]
pub struct ShotUploader {
    client: ShotServiceClient<Channel>,
    policy: RetryPolicy,
}

impl ShotUploader {
    pub fn new(channel: Channel, policy: RetryPolicy) -> Self {
        Self {
            client: ShotServiceClient::new(channel),
            policy,
        }
    }

    pub async fn upload(
        &self,
        frame: &GrayImage,
        timing: &StrobeTiming,
    ) -> Result<pb::ShotResult, Status> {
        let request = ProcessShotRequest {
            frame: Some(frame.into()),
            timing: Some(timing.into()),
        };
        let resp = retry_transient(self.policy, || {
            let mut client = self.client.clone();
            let request = request.clone();
            async move { client.process_shot(request).await }
        })
        .await?;
        resp.into_inner()
            .result
            .ok_or_else(|| Status::internal("ProcessShot response had no result"))
    }
}

/// The setup matching `launch_sim`'s bench camera (flash order x-descending).
pub fn bench_setup() -> CameraSetup {
    CameraSetup {
        calibration: Some((&launch_sim::bench_calibration([0.0; 5])).into()),
        flash_order: pb::FlashOrder::XDescending as i32,
    }
}

/// `CalibrationService::SetSetup` with the bench setup (sim mode only).
pub async fn install_bench_setup(channel: Channel, policy: RetryPolicy) -> Result<()> {
    let client = CalibrationServiceClient::new(channel);
    retry_transient(policy, || {
        let mut client = client.clone();
        async move { client.set_setup(bench_setup()).await }
    })
    .await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    fn fast(max_attempts: u32) -> RetryPolicy {
        RetryPolicy {
            max_attempts,
            initial_backoff: Duration::from_millis(1),
            max_backoff: Duration::from_millis(2),
        }
    }

    async fn run(code: Code, succeed_on: u32, policy: RetryPolicy) -> (Result<u32, Status>, u32) {
        let calls = AtomicU32::new(0);
        let r = retry_transient(policy, || {
            let n = calls.fetch_add(1, Ordering::SeqCst) + 1;
            async move {
                if n >= succeed_on {
                    Ok(n)
                } else {
                    Err(Status::new(code, "boom"))
                }
            }
        })
        .await;
        (r, calls.load(Ordering::SeqCst))
    }

    #[tokio::test]
    async fn retries_transient_until_success() {
        for code in [Code::Unavailable, Code::DeadlineExceeded] {
            let (r, calls) = run(code, 3, fast(4)).await;
            assert_eq!(r.unwrap(), 3);
            assert_eq!(calls, 3);
        }
    }

    #[tokio::test]
    async fn gives_up_after_max_attempts() {
        let (r, calls) = run(Code::Unavailable, 99, fast(3)).await;
        assert_eq!(r.unwrap_err().code(), Code::Unavailable);
        assert_eq!(calls, 3);
    }

    #[tokio::test]
    async fn never_retries_permanent_errors() {
        for code in [
            Code::InvalidArgument,
            Code::FailedPrecondition,
            Code::Internal,
        ] {
            let (r, calls) = run(code, 99, fast(5)).await;
            assert_eq!(r.unwrap_err().code(), code);
            assert_eq!(calls, 1, "{code:?} must not be retried");
        }
    }
}
