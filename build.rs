use std::env;
use std::path::PathBuf;
use std::process::Command;
use std::fs;

fn main() {
    let target = env::var("TARGET").unwrap();
    
    if target.contains("unknown-none") {
        println!("cargo:warning=Bare metal target detected - HDL disabled, using pure Rust CPU");
        println!("cargo:warning=Verilator runtime requires pthread and C++ stdlib which aren't available in bare metal");
        return;
    }
    
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    
    println!("cargo:rerun-if-changed=hdl/cpu57.v");
    println!("cargo:rerun-if-changed=hdl/verilator_wrapper.cpp");
    
    let verilator_root = Command::new("verilator")
        .arg("--getenv")
        .arg("VERILATOR_ROOT")
        .output()
        .expect("Failed to get VERILATOR_ROOT. Is Verilator installed?")
        .stdout;
    
    let verilator_root = String::from_utf8(verilator_root)
        .expect("VERILATOR_ROOT not UTF-8")
        .trim()
        .to_string();
    
    println!("cargo:warning=Building HDL for hosted target");
    
    let obj_dir = out_dir.join("obj_dir");
    
    let status = Command::new("verilator")
        .args(&[
            "--cc",
            "--build",
            "--public",
            "-Wall",
            "-Wno-fatal",
            "-Wno-DECLFILENAME",
            "-Wno-UNUSEDSIGNAL",
            "-Wno-VARHIDDEN",
            "-Wno-EOFNEWLINE",
            "-CFLAGS", "-std=c++17 -O3 -fPIC",
            "--Mdir", obj_dir.to_str().unwrap(),
            "--top-module", "cpu57_system",
            "hdl/cpu57.v",
        ])
        .status()
        .expect("Failed to run verilator");
    
    if !status.success() {
        panic!("Verilator failed");
    }
    
    cc::Build::new()
        .cpp(true)
        .file("hdl/verilator_wrapper.cpp")
        .include(&verilator_root)
        .include(format!("{}/include", verilator_root))
        .include(&obj_dir)
        .flag("-std=c++17")
        .flag("-O3")
        .flag("-fPIC")
        .warnings(false)
        .compile("cpu57_wrapper");
    
    println!("cargo:rustc-link-search=native={}", obj_dir.display());
    println!("cargo:rustc-link-lib=static=Vcpu57_system__ALL");
    println!("cargo:rustc-link-lib=dylib=stdc++");
}