//! Compile the .proto files with `protox` (pure Rust, no `protoc` needed),
//! generate tonic/prost code, and emit a descriptor set for gRPC reflection.

use std::path::PathBuf;

use prost::Message;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let protos = [
        "proto/launchmonitor/v1/types.proto",
        "proto/launchmonitor/v1/calibration.proto",
        "proto/launchmonitor/v1/shot.proto",
    ];
    let fds = protox::compile(protos, ["proto"])?;

    let out = PathBuf::from(std::env::var("OUT_DIR")?);
    std::fs::write(
        out.join("launchmonitor_descriptor.bin"),
        fds.encode_to_vec(),
    )?;

    tonic_prost_build::configure().compile_fds(fds)?;
    println!("cargo:rerun-if-changed=proto");
    Ok(())
}
