/// UART7_M2 switch iomux on RK3588
pub fn iomux_uart7_m2(addr: usize) {
    //const BUS_IOC_BASE: u32 =  0xfd5f8000; // 0x0 ~ 0x9C

    let gpio1b_iomux_sel_h: usize = 0x002c;

    let genmask = |h: u64, l: u64| (!(0_u64) << l) & (!(0_u64) >> (64 - 1 - h));

    let gpio1b4_mask: u64 = genmask(3, 0);
    let gpio1b5_mask: u64 = genmask(7, 4);
    const GPIO1B4_UART7_RX_M2: u64 = 10;
    const GPIO1B4_SHIFT: u64 = 0;
    const GPIO1B5_SHIFT: u64 = 4;
    const GPIO1B5_UART7_TX_M2: u64 = 10;

    rk_clrsetreg(
        (addr + gpio1b_iomux_sel_h) as u64,
        gpio1b4_mask | gpio1b5_mask,
        (GPIO1B4_UART7_RX_M2 << GPIO1B4_SHIFT) | (GPIO1B5_UART7_TX_M2 << GPIO1B5_SHIFT),
    );
}

/// UART7_M2 Switch iomux on RK3588
fn rk_clrsetreg(addr: u64, clr: u64, set: u64) {
    let value = (((clr | set) << 16) | set) as u32;
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, value);
    }
}

/// eg.: Enable GPIO3_C6_u, RK_PC6=22;
/// Pin = bank(3)*32 + number( C(2)*8 + 6 ) = 96 + 22 = 118
/// Data direction(GPIO_SWPORT_DDR_H): Output; Output data: High, Low;
pub fn gpio_output(gpio_bank: usize, number: usize, is_high: bool) {
    // number = [0, 31]
    let (base_offset, num_shift) = if number < 16 {
        // Output data for low 16 bits
        (0, number)
    } else {
        // Output data for high 16 bits
        (0x4, number % 16)
    };

    let data_base = gpio_bank + base_offset;
    let direction_base = gpio_bank + 0x8 + base_offset;
    let mask = (1 << num_shift) << 16;
    let level = if is_high { 1 } else { 0 };

    unsafe {
        // Set data direction: output.
        core::ptr::write_volatile(direction_base as *mut u32, (1 << num_shift) | mask);
        // Set data: high/low; high = 1, low = 0;
        core::ptr::write_volatile(data_base as *mut u32, (level << num_shift) | mask);
    }
}

pub fn gpio_output_clear(gpio_bank: usize) {
    unsafe {
        core::ptr::write_volatile(gpio_bank as *mut u32, 0 | (0xffff << 16));
        core::ptr::write_volatile((gpio_bank + 0x4) as *mut u32, 0 | (0xffff << 16));
    }
}

#[inline]
fn mmio_read32(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline]
fn mmio_write32(addr: usize, value: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) }
}

#[inline]
fn rk_write_masked(addr: usize, mask: u32, value: u32) {
    // RK write-mask format: upper 16 bits are write-enable mask, lower 16 bits are value.
    mmio_write32(addr, (mask << 16) | (value & mask));
}

/// Configure GPIO3_C6 iomux to GPIO function.
/// BUS_IOC_GPIO3C_IOMUX_SEL_H, gpio3c6_sel bits[11:8] = 0.
pub fn iomux_gpio3_c6_gpio(bus_ioc_base: usize) {
    const BUS_IOC_GPIO3C_IOMUX_SEL_H: usize = 0x0074;
    const GPIO3C6_SEL_SHIFT: u32 = 8;
    const GPIO3C6_SEL_MASK: u32 = 0xF << GPIO3C6_SEL_SHIFT;
    const GPIO3C6_SEL_GPIO: u32 = 0 << GPIO3C6_SEL_SHIFT; // GPIO function, default value after reset is 0, so this write can be skipped.
    //const GPIO3C6_SEL_GPIO: u32 = 0x8 << GPIO3C6_SEL_SHIFT; // SPI3_MISO_M3

    rk_write_masked(
        bus_ioc_base + BUS_IOC_GPIO3C_IOMUX_SEL_H,
        GPIO3C6_SEL_MASK,
        GPIO3C6_SEL_GPIO,
    );
}

/// Enable GPIO3 clocks by opening dbclk_gpio3_en and pclk_gpio3_en gate.
/// CRU_GATE_CON17 bits[3:2] = 2'b00 means gate open.
pub fn gpio3_clock_gate_enable(cru_base: usize) {
    const CRU_GATE_CON17: usize = 0x0844;
    const GPIO3_CLK_GATE_MASK: u32 = 0b11 << 2;
    const GPIO3_CLK_GATE_ENABLE: u32 = 0b00 << 2;

    rk_write_masked(
        cru_base + CRU_GATE_CON17,
        GPIO3_CLK_GATE_MASK,
        GPIO3_CLK_GATE_ENABLE,
    );
}

pub fn gpio_ver_id_get(gpio_base: usize) -> u32 {
    mmio_read32(gpio_base + 0x0078)
}

pub fn gpio_ext_port_signals_get(gpio_base: usize) -> u32 {
    mmio_read32(gpio_base + 0x0070)
}