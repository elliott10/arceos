//! snps,dw-apb-uart serial driver

use memory_addr::{PhysAddr, VirtAddr};
use spinlock::SpinNoIrq;
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::register_structs;
use tock_registers::registers::{ReadOnly, ReadWrite};

use crate::mem::phys_to_virt;

const UART_BASE: PhysAddr = PhysAddr::from(axconfig::UART_PADDR);

static UART: SpinNoIrq<DW8250> = SpinNoIrq::new(DW8250::new(phys_to_virt(UART_BASE)));

register_structs! {
    DW8250Regs {
        /// Get or Put Register.
        (0x00 => rbr: ReadWrite<u32>),
        (0x04 => ier: ReadWrite<u32>),
        (0x08 => fcr: ReadWrite<u32>),
        (0x0c => lcr: ReadWrite<u32>),
        (0x10 => mcr: ReadWrite<u32>),
        (0x14 => lsr: ReadOnly<u32>),
        (0x18 => msr: ReadOnly<u32>),
        (0x1c => scr: ReadWrite<u32>),
        (0x20 => lpdll: ReadWrite<u32>),
        (0x24 => _reserved0),
        /// Uart Status Register.
        (0x7c => usr: ReadOnly<u32>),
        (0x80 => _reserved1),
        (0xc0 => dlf: ReadWrite<u32>),
        (0xc4 => @END),
    }
}

struct DW8250 {
    base_vaddr: VirtAddr,
}

impl DW8250 {
    const fn new(base_vaddr: VirtAddr) -> Self {
        Self { base_vaddr }
    }

    const fn regs(&self) -> &DW8250Regs {
        unsafe { &*(self.base_vaddr.as_ptr() as *const _) }
    }

    fn init(&mut self) {
        const UART_SRC_CLK: u32 = 25000000;
        const BST_UART_DLF_LEN: u32 = 6;
        const BAUDRATE: u32 = 115200;
        //const BAUDRATE: u32 = 38400;

        let get_baud_divider = |baudrate| (UART_SRC_CLK << (BST_UART_DLF_LEN - 4)) / baudrate;
        let divider = get_baud_divider(BAUDRATE);

        // Waiting to be no USR_BUSY.
        while self.regs().usr.get() & 0b1 != 0 {}

        // bst_serial_hw_init_clk_rst

        /* Disable interrupts and Enable FIFOs */
        self.regs().ier.set(0);
        self.regs().fcr.set(1);

        /* Disable flow ctrl */
        self.regs().mcr.set(0);

        /* Clear MCR_RTS */
        self.regs().mcr.set(self.regs().mcr.get() | (1 << 1));

        /* Enable access DLL & DLH. Set LCR_DLAB */
        self.regs().lcr.set(self.regs().lcr.get() | (1 << 7));

        dmb();

        /* Set baud rate. Set DLL, DLH, DLF */
        self.regs().rbr.set((divider >> BST_UART_DLF_LEN) & 0xff);
        self.regs()
            .ier
            .set((divider >> (BST_UART_DLF_LEN + 8)) & 0xff);
        self.regs().dlf.set(divider & ((1 << BST_UART_DLF_LEN) - 1));

        /* Clear DLAB bit */
        self.regs().lcr.set(self.regs().lcr.get() & !(1 << 7));

        dmb();

        /* Set data length to 8 bit, 1 stop bit, no parity. Set LCR_WLS1 | LCR_WLS0 */
        self.regs().lcr.set(self.regs().lcr.get() | 0b11);

        dmb();
    }

    fn putchar(&mut self, c: u8) {
        // Check LSR_TEMT
        // Wait for last character to go.
        while self.regs().lsr.get() & (1 << 6) == 0 {}
        self.regs().rbr.set(c as u32);
    }

    fn getchar(&mut self) -> Option<u8> {
        // Check LSR_DR
        // Wait for a character to arrive.
        if self.regs().lsr.get() & 0b1 != 0 {
            Some((self.regs().rbr.get() & 0xff) as u8)
        } else {
            None
        }
    }

    fn set_ier(&mut self, enable: bool) {
        if enable {
            // Enable interrupts
            self.regs().ier.set(1);
        } else {
            // Disable interrupts
            self.regs().ier.set(0);
        }
        dmb();
    }
}

/// Memory Barrier
fn dmb() {
    unsafe {
        core::arch::asm!("dmb sy");
    }
}

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

/// UART simply initialize
pub fn init_early() {
    UART.lock().init();
}

/// Set UART IRQ Enable
pub fn init_irq() {
    UART.lock().set_ier(true);

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
