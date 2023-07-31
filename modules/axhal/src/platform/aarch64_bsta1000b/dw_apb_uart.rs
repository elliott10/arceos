//! snps,dw-apb-uart serial driver

use crate::mem::phys_to_virt;
use dw_apb_uart::dw_apb_uart::DW8250;
use memory_addr::PhysAddr;
use spinlock::SpinNoIrq;

const UART_BASE: PhysAddr = PhysAddr::from(axconfig::UART_PADDR);

static UART: SpinNoIrq<DW8250> = SpinNoIrq::new(DW8250::new(phys_to_virt(UART_BASE).as_usize()));

/// Writes a byte to the console.
pub fn putchar(c: u8) {
    let mut uart = UART.lock();
    match c {
        b'\r' | b'\n' => {
            uart.putchar(b'\r');
            uart.putchar(b'\n');
        }
        c => uart.putchar(c),
    }
}

/// Reads a byte from the console, or returns [`None`] if no input is available.
pub fn getchar() -> Option<u8> {
    UART.lock().getchar()
}

/// Memory Barrier
fn dmb() {
    unsafe {
        core::arch::asm!("dmb sy");
    }
}

/// UART simply initialize
pub fn init_early() {
    UART.lock().init();
    dmb();
}

/// Set UART IRQ Enable
pub fn init_irq() {
    UART.lock().set_ier(true);
    dmb();

    #[cfg(feature = "irq")]
    {
        use crate::platform::aarch64_common::gic::{gic_irq_tran, IntIdType};
        // IRQ Type: SPI
        crate::irq::register_handler(gic_irq_tran(axconfig::UART_IRQ_NUM, IntIdType::SPI), handle);
    }
}

/// UART IRQ Handler
pub fn handle() {
    trace!("Uart IRQ Handler");
}
