use std::env;

fn main() {
    println!(
        "cargo:rustc-link-search={}",
        env::var("CARGO_MANIFEST_DIR").unwrap()
    );
    println!("cargo:rerun-if-changed=memory.x");
}
