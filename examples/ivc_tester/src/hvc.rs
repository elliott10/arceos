use core::arch::asm;

use axhvc::HyperCallCode;

#[inline(always)]
fn trigger_hypercall(
    code: HyperCallCode,
    arg1: u64,
    arg2: u64,
    arg3: u64,
    arg4: u64,
    arg5: u64,
    arg6: u64,
) -> isize {
    let result: isize;
    unsafe {
        asm!(
            "hvc #0",
            in("x0") code as u64,
            in("x1") arg1,
            in("x2") arg2,
            in("x3") arg3,
            in("x4") arg4,
            in("x5") arg5,
            in("x6") arg6,
            lateout("x0") result,
            options(nostack, preserves_flags)
        );
    }
    result
}

pub fn hvc_publish_channel(key: u64, shm_base_ptr: u64, shm_size_ptr: u64) -> isize {
    trigger_hypercall(
        HyperCallCode::HIVCPublishChannel,
        key,
        shm_base_ptr,
        shm_size_ptr,
        0,
        0,
        0,
    )
}
