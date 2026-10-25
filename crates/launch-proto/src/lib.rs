//! gRPC contract for the launch monitor, plus conversions between the wire
//! messages and the internal domain types.
//!
//! The domain crates (`launch-types`, `calibration`, `vision`, ...) never see
//! protobuf types; conversion happens at the service edge via [`convert`].

pub mod convert;

/// Generated messages and gRPC service stubs (package `launchmonitor.v1`).
pub mod pb {
    tonic::include_proto!("launchmonitor.v1");
}

/// Encoded `FileDescriptorSet` for gRPC server reflection.
pub const FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/launchmonitor_descriptor.bin"));
