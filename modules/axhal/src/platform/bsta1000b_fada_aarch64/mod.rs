pub mod mem;

#[cfg(feature = "smp")]
pub mod mp;

#[cfg(feature = "irq")]
pub mod irq {
    pub use crate::platform::aarch64_common::gic::*;
}

pub mod console {
    //pub use crate::platform::aarch64_common::pl011::*;
    const UART0_ADDR: usize = axconfig::UART_PADDR + axconfig::PHYS_VIRT_OFFSET;

    /// Example of putchar:
    /// console::putchar(b'X');
    ///
    /// let ptr = 0x20008000 as *mut u32;
    /// ptr.add(0).write_volatile(b'E' as u32);
    pub fn putchar(c: u8) {
        let ptr = UART0_ADDR as *mut u32;
        unsafe {
            //LSR bit:LSR_TEMT
            while ptr.add(5).read_volatile() & (1 << 6) == 0 {}

            ptr.add(0).write_volatile(c as u32);
        }
    }
    pub fn getchar() -> Option<u8> {
        let ptr = UART0_ADDR as *mut u32;
        unsafe {
            //Check LSR bit:DR
            if ptr.add(5).read_volatile() & 0b1 == 0 {
                None
            } else {
                Some((ptr.add(0).read_volatile() & 0xff) as u8)
            }
        }
    }
}

pub mod time {
    pub use crate::platform::aarch64_common::generic_timer::*;
}

pub mod misc {
    pub use crate::platform::aarch64_common::psci::system_off as terminate;
}

extern "C" {
    fn exception_vector_base();
    fn rust_main(cpu_id: usize, dtb: usize);
    #[cfg(feature = "smp")]
    fn rust_main_secondary(cpu_id: usize);
}

pub(crate) unsafe extern "C" fn rust_entry(cpu_id: usize, dtb: usize) {
    crate::mem::clear_bss();
    crate::arch::set_exception_vector_base(exception_vector_base as usize);
    crate::cpu::init_primary(cpu_id);
    //super::aarch64_common::pl011::init_early();
    super::aarch64_common::generic_timer::init_early();
    rust_main(cpu_id, dtb);
}

#[cfg(feature = "smp")]
pub(crate) unsafe extern "C" fn rust_entry_secondary(cpu_id: usize) {
    crate::arch::set_exception_vector_base(exception_vector_base as usize);
    crate::cpu::init_secondary(cpu_id);
    rust_main_secondary(cpu_id);
}

/// Initializes the platform devices for the primary CPU.
///
/// For example, the interrupt controller and the timer.
pub fn platform_init() {
    #[cfg(feature = "irq")]
    super::aarch64_common::gic::init_primary();
    super::aarch64_common::generic_timer::init_percpu();
    //super::aarch64_common::pl011::init();
}

/// Initializes the platform devices for secondary CPUs.
#[cfg(feature = "smp")]
pub fn platform_init_secondary() {
    #[cfg(feature = "irq")]
    super::aarch64_common::gic::init_secondary();
    super::aarch64_common::generic_timer::init_percpu();
}
