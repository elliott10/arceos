#![no_std]
#![no_main]

mod hvc;

use core::time::Duration;

use axstd::format;
use axstd::os::arceos::api;
use axstd::os::arceos::modules;
use axstd::println;

use api::time::ax_monotonic_time;
use modules::axhal::mem::{PhysAddr, VirtAddr, phys_to_virt, virt_to_phys};

#[repr(C)]
pub struct IVCChannelHeader {
    pub publisher_id: u64,
    pub key: u64,
    pub content_size: u64,
}

#[unsafe(no_mangle)]
fn main() {
    println!("Hello, world!");

    println!("Current monotonic time: {:?}", ax_monotonic_time());

    // Example channel key
    let publisher_channel_key: u64 = 0xdeadbeef;
    // Shared memory base physical address.
    let publisher_shm_base: u64 = 0;
    // Shared memory size in bytes.
    let publisher_shm_size: u64 = 4096 * 2;

    let ret = hvc::hvc_publish_channel(
        publisher_channel_key,
        virt_to_phys(VirtAddr::from_ptr_of(&publisher_shm_base)).as_usize() as _,
        virt_to_phys(VirtAddr::from_ptr_of(&publisher_shm_size)).as_usize() as _,
    );

    if ret < 0 {
        println!("Failed to publish channel: {}", ret);
        return;
    }

    println!(
        "hvc_publish_channel established key: {:#x}, base {:#x}, size {:#x}",
        publisher_channel_key, publisher_shm_base, publisher_shm_size
    );

    let shm_base = phys_to_virt(PhysAddr::from_usize(publisher_shm_base as usize));

    println!(
        "Shared memory base {:?}, size: {:#x}",
        shm_base, publisher_shm_size
    );

    let shm_header = unsafe {
        shm_base
            .as_mut_ptr_of::<IVCChannelHeader>()
            .as_mut()
            .unwrap()
    };

    println!(
        "IVCChannelHeader: publisher_id: {:#x}, key: {:#x}, content_size: {:#x}",
        shm_header.publisher_id, shm_header.key, shm_header.content_size
    );

    let shm_buf = unsafe {
        core::slice::from_raw_parts_mut(
            shm_base
                .as_mut_ptr()
                .add(core::mem::size_of::<IVCChannelHeader>()),
            publisher_shm_size as usize - core::mem::size_of::<IVCChannelHeader>(),
        )
    };

    println!(
        "Shared memory buffer @ {:#p}, size: {:#x}",
        shm_buf.as_ptr(),
        shm_buf.len()
    );

    loop {
        modules::axtask::sleep(Duration::from_millis(8000));
        let msg = format!("Hello from IVC Tester! Time: {:?}", ax_monotonic_time());

        println!("{}", msg);

        let msg_bytes = msg.as_bytes();

        // Ensure we only copy the portion of `msg_bytes` that fits into `shm_buf`.
        let copy_len = core::cmp::min(shm_buf.len(), msg_bytes.len());
        shm_buf[..copy_len].copy_from_slice(&msg_bytes[..copy_len]);

        shm_header.content_size = copy_len as u64;

        println!("Publish {} bytes into shared memory buffer.", copy_len);
    }
}
