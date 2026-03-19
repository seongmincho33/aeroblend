use std::env;
use std::path::PathBuf;

fn main() {
    let crate_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    // Output header to the cpp/include/aeroblend directory
    let mut header_path = PathBuf::from(&crate_dir);
    header_path.pop(); // aeroblend-ffi
    header_path.pop(); // rust
    header_path.push("cpp");
    header_path.push("include");
    header_path.push("aeroblend");
    header_path.push("ffi.h");

    let config = cbindgen::Config::from_file(format!("{}/cbindgen.toml", crate_dir))
        .expect("Unable to read cbindgen.toml");

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .with_parse_deps(true)
        .with_parse_include(&["aeroblend-core"])
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(&header_path);

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
}
