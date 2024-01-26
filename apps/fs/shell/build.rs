fn main() {
    println!("cargo:rustc-link-search=native=ulib/axlibc/");
    println!("cargo:rustc-link-lib=static=c");
}
