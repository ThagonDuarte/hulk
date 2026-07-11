use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=HOME");
    println!("cargo:rerun-if-env-changed=XDG_CACHE_HOME");
    if env::var_os("CARGO_CFG_TARGET_OS").as_deref() != Some(std::ffi::OsStr::new("linux")) {
        return;
    }

    let cache_root = env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| env::var_os("HOME").map(|home| PathBuf::from(home).join(".cache")));
    let Some(cache_root) = cache_root else {
        return;
    };
    let Some(target) = env::var_os("TARGET") else {
        return;
    };
    let distribution_root = cache_root.join("ort.pyke.io/dfbin").join(target);
    let Ok(distributions) = fs::read_dir(distribution_root) else {
        return;
    };
    let mut distributions = distributions
        .flatten()
        .map(|entry| entry.path())
        .collect::<Vec<_>>();
    distributions.sort();

    for distribution in distributions {
        let library_directory = distribution.join("onnxruntime/lib");
        if library_directory.join("libwebgpu_dawn.so").is_file() {
            println!(
                "cargo:rustc-link-arg=-Wl,-rpath,{}",
                library_directory.display()
            );
            return;
        }
    }
}
