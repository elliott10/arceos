#![no_std]
#![no_main]

use core::time::Duration;

use axstd::println;
use axstd::os::arceos::api;
use axstd::os::arceos::modules;

use api::time::ax_monotonic_time;

#[unsafe(no_mangle)]
fn main() {
    println!("Hello, world!");

    println!("Current monotonic time: {:?}", ax_monotonic_time());

    loop {
        modules::axtask::sleep(Duration::from_millis(500));
        println!("Current monotonic time: {:?}", ax_monotonic_time());
    }
}
