//! Definitions for snps,dw-apb-uart serial driver.
//! Uart snps,dw-apb-uart driver in Rust for BST A1000b FADA board.
#![no_std]

pub mod gpio;

use tock_registers::{
    interfaces::{Readable, Writeable},
    register_structs,
    registers::{ReadOnly, ReadWrite},
};

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
        (0x88 => srr: ReadWrite<u32>),
        (0x8c => _reserved2),
        (0xc0 => dlf: ReadWrite<u32>),
        (0xc4 => @END),
    }
}

/// dw-apb-uart serial driver: DW8250
pub struct DW8250 {
    base_vaddr: usize,
}

impl DW8250 {
    /// New a DW8250
    pub const fn new(base_vaddr: usize) -> Self {
        Self { base_vaddr }
    }

    const fn regs(&self) -> &DW8250Regs {
        unsafe { &*(self.base_vaddr as *const _) }
    }

    pub fn init(&mut self) {}

    /// DW8250 initialize
    pub fn minit(&mut self) {
        const UART_SRC_CLK: u32 = 24_000_000;
        const BAUDRATE: u32 = 1500000;

        let get_baud_divider = |baudrate| { (UART_SRC_CLK + (baudrate * 16)/2) / (baudrate * 16)};
        let divider = get_baud_divider(BAUDRATE);

        // Waiting to be UART_LSR_TEMT
        while self.regs().lsr.get() & 0x40 == 0 {}

        // Disable interrupts
        self.regs().ier.set(0);

        self.regs().fcr.set(0x6); // Disable fifo
        self.regs().mcr.set(0x3); // Set "data terminal ready" and "request to send"
        self.regs().lcr.set(0x3); // 8bits data length

        /* Enable access DLL & DLH. Set LCR_DLAB */
        self.regs().lcr.set(0x80 | self.regs().lcr.get());

        /* Set baud rate. Set DLL, DLH, DLF */
        self.regs().rbr.set(divider & 0xff);
        self.regs().ier.set((divider >> 8) & 0xff);

        /* Clear DLAB bit */
        self.regs().lcr.set(self.regs().lcr.get() & !0x80);
    }

    /// DW8250 serial output
    pub fn putchar(&mut self, c: u8) {
        // Check LSR_TEMT
        // Wait for last character to go.
        while self.regs().lsr.get() & (1 << 6) == 0 {}
        self.regs().rbr.set(c as u32);
    }

    /// DW8250 serial input
    pub fn getchar(&mut self) -> Option<u8> {
        // Check LSR_DR
        // Wait for a character to arrive.
        if self.regs().lsr.get() & 0b1 != 0 {
            Some((self.regs().rbr.get() & 0xff) as u8)
        } else {
            None
        }
    }

    /// DW8250 serial interrupt enable or disable
    pub fn set_ier(&mut self, enable: bool) {
        if enable {
            // Enable interrupts
            self.regs().ier.set(1);
        } else {
            // Disable interrupts
            self.regs().ier.set(0);
        }
    }

    // Backward-compatible wrappers.
    pub fn iomux_uart7_m2(addr: usize) {
        gpio::iomux_uart7_m2(addr)
    }

    pub fn gpio_output(gpio_bank: usize, number: usize, is_high: bool) {
        gpio::gpio_output(gpio_bank, number, is_high)
    }

    pub fn gpio_output_clear(gpio_bank: usize) {
        gpio::gpio_output_clear(gpio_bank)
    }
}
