const BUS_IOC_BASE: usize = 0xffff_0000_fd5f8000;
const CRU_BASE: usize = 0xffff_0000_fd7c0000;
const UART7_BASE: usize = 0xffff_0000_feba0000;
pub const GPIO3_BASE: usize = 0xffff_0000_fec40000;

#[inline]
fn mmio_read32(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline]
fn mmio_write32(addr: usize, value: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) }
}

pub fn init_uart7() {
    println!("Bencher init UART7 ...\n");

    // BUS_IOC_GPIO4D_IOMUX_SEL_L 0x0098 = 0x00005500
    let iomux_gpio4d = mmio_read32(BUS_IOC_BASE + 0x0098);
    // BUS_IOC_GPIO1B_IOMUX_SEL_H
    let iomux_gpio1b = mmio_read32(BUS_IOC_BASE + 0x002c);
    println!(
        "Read BUS_IOC, gpio4d default ={:#x}, UART7 iomux = {:#x}\n",
        iomux_gpio4d, iomux_gpio1b
    );

    // CRU clock
    // CRU_CLKSEL_CON53 clk_uart7_src_sel = 0x6
    let clk_sel = mmio_read32(CRU_BASE + 0x03d4);
    // CRU_CLKSEL_CON54 clk_uart7_frac = 0x1403de
    let clk_frac = mmio_read32(CRU_BASE + 0x03d8);
    // CRU_CLKSEL_CON55 sclk_uart7_sel = 0x6
    let sclk_sel = mmio_read32(CRU_BASE + 0x03dc);

    // CRU_GATE_CON12 pclk_uart7_en bit8 = 0
    let pclk_en = mmio_read32(CRU_BASE + 0x0830);
    // CRU_GATE_CON13 sclk_uart7_en bit15 = 0
    let sclk_en = mmio_read32(CRU_BASE + 0x0834);

    println!(
        "Read CRU, clk_uart7_src_sel={:#x}, clk_uart7_frac={:#x}, sclk_uart7_sel={:#x} \n pclk_uart7_en={:#x}, sclk_uart7_en={:#x},\n",
        clk_sel, clk_frac, sclk_sel, pclk_en, sclk_en
    );

    // Keep pclk_uart7_en/sclk_uart7_en CRU gate enabled on rk3588 when Linux may leave them disabled.
    // CRU_GATE_CON12, PCLK_UART7, pclk_uart7_en bit8
    mmio_write32(
        CRU_BASE + 0x0830,
        (pclk_en & !(1 << 8)) | ((1 << 8) << 16),
    );
    mmio_write32(
        CRU_BASE + 0x0834,
        (sclk_en & !(1 << 15)) | ((1 << 15) << 16),
    );

    // CRU_GATE_CON12 pclk_uart7_en bit8 = 0
    let pclk_en = mmio_read32(CRU_BASE + 0x0830);
    // CRU_GATE_CON13 sclk_uart7_en bit15 = 0
    let sclk_en = mmio_read32(CRU_BASE + 0x0834);
    println!(
        "PCLK_UART7, pclk_uart7_en={:#x}, SCLK_UART7, sclk_uart7_en={:#x}",
        pclk_en, sclk_en
    );

    dw_apb_uart::gpio::iomux_uart7_m2(BUS_IOC_BASE);

    /********** rk3588 UART7 **********/
    // UART Component Version = 0x3430322A
    let ucv = mmio_read32(UART7_BASE + 0x00f8);
    // Interrupt Identity Register = 0x1
    let iir = mmio_read32(UART7_BASE + 0x0008);
    // UART Status Register = 0x6
    let usr = mmio_read32(UART7_BASE + 0x007c);
    println!("Read UART7, UCV={:#x}, USR={:#x}, IIR={:#x}\n", ucv, usr, iir);

    let mut uart = dw_apb_uart::DW8250::new(UART7_BASE);
    uart.minit();

    println!("Bencher output to UART7\n");
    uart7_put_bytes(b"\n\rBencher\n\r");
}

pub fn uart7_put_bytes(data: &[u8]) {
    let mut uart = dw_apb_uart::DW8250::new(UART7_BASE);
    for b in data {
        uart.putchar(*b);
    }
}

pub fn uart7_put_hi() {
    uart7_put_bytes(b"Hi\n\r");
}

/// Configure GPIO3_C6 iomux to GPIO function.
/// BUS_IOC_GPIO3C_IOMUX_SEL_H, gpio3c6_sel bits[11:8] = 0.
pub fn iomux_gpio3_c6_gpio() {
    dw_apb_uart::gpio::iomux_gpio3_c6_gpio(BUS_IOC_BASE);
}

/// Enable GPIO3 clocks by opening dbclk_gpio3_en / pclk_gpio3_en gate.
/// CRU_GATE_CON17 bits[3:2] = 2'b00 means gate open (enabled).
pub fn gpio3_clock_gate_enable() {
    dw_apb_uart::gpio::gpio3_clock_gate_enable(CRU_BASE);
}

////////// LED
/// Blue: GPIO1_D5, 61
/// Red:* GPIO3_B2, 106, num=10
/// Green:* GPIO3_C0, 112, num=16
/// GPIO_ACTIVE_HIGH 表示高电平有效;
///
/// GPIO单独物理引脚: (GPIO3_C6_u, RK_PC6=22, Output High)
pub fn gpio3_clear_all() {
    // Turn off all LEDs
    dw_apb_uart::gpio::gpio_output_clear(GPIO3_BASE);
}

pub fn gpio3_led_red_on() {
    // Red LED
    dw_apb_uart::gpio::gpio_output(GPIO3_BASE, 10, true);
}

pub fn gpio3_led_green_on() {
    // Green LED
    dw_apb_uart::gpio::gpio_output(GPIO3_BASE, 16, true);
}

/// GPIO3_C6 (num=22) output high, used as thread-0 pulse level.
pub fn gpio3_output_high() {
    dw_apb_uart::gpio::gpio_output(GPIO3_BASE, 22, true);
}

/// GPIO3_C6 (num=22) output low, used as thread-1 pulse level.
pub fn gpio3_output_low() {
    dw_apb_uart::gpio::gpio_output(GPIO3_BASE, 22, false);
}

pub fn gpio_ver_id_get(gpio_base: usize) {
    let ver_id = dw_apb_uart::gpio::gpio_ver_id_get(gpio_base);
    println!("Read GPIO_VER_ID={:#x}", ver_id);
}

/// 当前 GPIO 端口各引脚在管脚上的实时电平状态。
/// 32位值，每一位对应这个 GPIO 控制器里的一个引脚状态：
/// 0：该引脚当前是低电平
/// 1：该引脚当前是高电平
pub fn gpio_ext_port_signals_get(gpio_base: usize) {
    let ext_port = dw_apb_uart::gpio::gpio_ext_port_signals_get(gpio_base);
    println!("Read GPIO_EXT_PORT={:#x}", ext_port);
}
