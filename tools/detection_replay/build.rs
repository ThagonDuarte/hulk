use std::env;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    if env::var_os("CARGO_CFG_TARGET_OS").as_deref() != Some(std::ffi::OsStr::new("linux")) {
        return;
    }
    // ort-sys copies its shared libraries next to Cargo's output artifacts.
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
}
