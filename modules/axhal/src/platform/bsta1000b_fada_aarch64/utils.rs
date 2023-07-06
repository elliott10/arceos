use core::ptr::{read_volatile, write_volatile};
use crate::time::{busy_wait, Duration};
use crate::mem::phys_to_virt;

/// 微秒(us) 延时器
pub fn usdelay(us: u64) {
    let d = Duration::from_micros(us);
    busy_wait(d);
}

/// 毫秒(ms) 延时器
pub fn msdelay(ms: u64) {
    usdelay(ms * 1000);
}

/// Do QSPI reset
pub fn reset_qspi() {
    // qspi exit 4-byte mode
    // exit_4byte_qspi();

    let ptr = phys_to_virt((axconfig::A1000BASE_SAFETYCRM + 0x8).into()).as_mut_ptr() as *mut u32;
    unsafe {
        let value = read_volatile(ptr);
        trace!("SAFETY CRM RESET CTRL = {:#x}", value);
        write_volatile(ptr, value & !(0b11 << 15));
        msdelay(100);

        write_volatile(ptr, value | (0b11 << 15));
        msdelay(100);
    }
}

/// Do CPU reset
pub fn reset_cpu() {
    reset_qspi();

    //Data Width = 32
    let ptr = phys_to_virt((axconfig::A1000BASE_SAFETYCRM + 0x8).into()).as_mut_ptr() as *mut u32;
    unsafe { write_volatile(ptr, read_volatile(ptr) & !0b1); }

    loop {}
}

/// reboot system
#[allow(dead_code)]
pub fn do_reset() {
    axlog::ax_println!("resetting ...\n");

    // wait 50 ms
    msdelay(50);

    // disable_interrupts();

    reset_cpu();

    // NOT REACHED
    warn!("NOT REACHED Resetting");
}

/// bootmode define bit [27:26], from strap pin
#[allow(dead_code)]
pub fn get_bootmode() -> u32 {
    unsafe {
        let ptr = phys_to_virt((axconfig::A1000BASE_TOPCRM).into()).as_mut_ptr() as *mut u32;
        (ptr.read_volatile() >> 26) & 0x7
    }
}
