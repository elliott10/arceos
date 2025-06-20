use crate::mem::{phys_to_virt, PhysAddr};
use crate::platform::aarch64_common::gic;
use core::ptr::NonNull;
use spinlock::SpinNoIrq;
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::register_structs;
use tock_registers::registers::{ReadOnly, ReadWrite, WriteOnly};

/// GPIO controller: GL061
register_structs! {
    pub PL061Regs {
        (0x000 => pub data: ReadWrite<u32>),
        (0x004 => __reserved_0),
        (0x400 => pub dir: ReadWrite<u32>),
        (0x404 => pub is: ReadWrite<u32>),
        (0x408 => pub ibe: ReadWrite<u32>),
        (0x40c => pub iev: ReadWrite<u32>),
        (0x410 => pub ie: ReadWrite<u32>),
        (0x414 => pub ris: ReadOnly<u32>),
        (0x418 => pub mis: ReadOnly<u32>),
        (0x41c => pub ic: WriteOnly<u32>),
        (0x420 => pub afsel: ReadWrite<u32>),
        (0x424 => @END),
    }
}

pub struct GpioPl061 {
    base: NonNull<PL061Regs>,
    max_irqs: usize,
}

unsafe impl Send for GpioPl061 {}
unsafe impl Sync for GpioPl061 {}

static GPIO: SpinNoIrq<GpioPl061> = SpinNoIrq::new(GpioPl061::new(
    phys_to_virt(PhysAddr::from(axconfig::GPIO_PADDR)).as_mut_ptr(),
));

impl GpioPl061 {
    /// Construct a new GPIO controller instance from a base address.
    pub const fn new(base: *mut u8) -> Self {
        Self {
            base: NonNull::new(base).unwrap().cast(),
            max_irqs: 8,
        }
    }

    const fn regs(&self) -> &PL061Regs {
        unsafe { self.base.as_ref() }
    }

    pub fn max_irqs(&self) -> usize {
        self.max_irqs
    }

    pub fn int_enable(&self, key: usize, is_enable: bool) {
        // Available bits [7:0]
        let gpioie = self.regs().ie.get() & 0xff;
        if is_enable {
            // For example: Power Down Key = 3
            self.regs().ie.set(gpioie | (1 << key));
        } else {
            self.regs().ie.set(gpioie & !(1 << key));
        }
    }

    pub fn int_mask_get(&self) -> u32 {
        self.regs().mis.get() & 0xff
    }

    pub fn int_status(&self) -> u32 {
        self.regs().ris.get() & 0xff
    }

    pub fn int_clear(&self, int_mask: u32) {
        self.regs().ic.set(int_mask & 0xff);
    }
}

/// 初始化GPIO中断
pub fn gpio_init() {
    // GPIO中断号39
    const GPIO_IRQ: usize = 0x07 + 32;

    // 通过GIC中断控制器，使能GPIO的Power Down Key中断
    // 使用Qemu终端的system_powerdown命令触发关机按钮
    crate::irq::set_enable(GPIO_IRQ, true);

    //注册中断处理函数
    gic::register_handler(GPIO_IRQ, handle_gpio_irq);
    info!("GPIO Pin3 interrupt {} is enabled.", GPIO_IRQ);

    // 启用pl061 gpio中的3号线中断
    // poweroff::gpios = <0x8004 0x03 0x00>;
    GPIO.lock().int_enable(3, true);
}

/// GPIO中断处理函数
pub fn handle_gpio_irq() {
    // info!("GPIO raw int status: {:#x}", GPIO.lock().int_status());
    let gpiomis = GPIO.lock().int_mask_get();
    info!("Power Down by GPIO int mask status: {:#x}", gpiomis);

    // 清除中断信号 此时GPIOMIS的值应该是0x8
    GPIO.lock().int_clear(gpiomis);

    if (gpiomis & (1 << 3)) != 0 {
        // Power Off key 3
        use crate::platform::aarch64_common::psci;
        psci::system_off();
        /* unsafe {
            // Just for Qemu -semihosting
            use core::arch::asm;
            asm!("mov w0, #0x18");
            asm!("hlt #0xF000");
        } */
    }
}
