//! Generates Rust types for the host protocol from `proto/axis.proto`.

use std::{env, path::PathBuf};

use micropb_gen::{Config, Generator};

const PROTO_DIR: &str = "../proto";
const PROTO: &str = "axis.proto";

fn main() {
    println!("cargo:rerun-if-changed={PROTO_DIR}/{PROTO}");

    let mut generator = Generator::new();
    generator.add_protoc_arg(format!("-I{PROTO_DIR}"));
    // No allocator on the target: variable-length fields get fixed capacities.
    generator.use_container_heapless();
    generator.configure(".axis.Info.firmware_version", Config::new().max_bytes(16));
    generator.configure(".axis.Info.mac_address", Config::new().max_bytes(6));
    generator.configure(".axis.Info.serial_number", Config::new().max_bytes(16));
    generator.configure(".axis.Log.text", Config::new().max_bytes(240));
    generator.configure(".axis.AddPath.segments", Config::new().max_len(16));

    let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("axis.rs");
    generator
        .compile_protos(&[PROTO], out)
        .expect("failed to compile proto/axis.proto (is protoc installed?)");
}
