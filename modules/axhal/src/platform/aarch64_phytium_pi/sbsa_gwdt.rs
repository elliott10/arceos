use core::ptr::NonNull;
use tock_registers::interfaces::{Readable, Writeable};
use tock_registers::register_structs;
use tock_registers::registers::{ReadOnly, ReadWrite, WriteOnly};

register_structs! {
    /// Watchdog registers
    SbsaGWdtRegs {
        /// Watchdog 更新寄存器。写操作会重新开始看门狗计数，读返回0
        (0x0000 => wrr: ReadWrite<u32>),
        (0x0004 => _reserved0004),
        /// 看门狗接口身份识别寄存器
        (0x0fcc => iidr: ReadOnly<u32>),
        (0x0fd0 => _reserved0fd0),
        /// 看门狗控制和状态寄存器
        (0x1000 => wcs: ReadWrite<u32>),
        (0x1004 => _reserved1004),
        /// 看门狗超时寄存器
        (0x1008 => wor: ReadWrite<u32>),
        (0x100c => _reserved100c),
        /// WCVL、WCVH 寄存器存储的看门狗计数的比较值。
        /// 注意：写操作有严格的顺序，必须先写高32位，再写低32位。
        ///
        /// Watchdog 比较值的低32位
        (0x1010 => wcvl: ReadWrite<u32>),
        /// Watchdog 比较值的高32位
        (0x1014 => wcvh: ReadWrite<u32>),

        // The end of the struct is marked as follows.
        (0x1018 => @END),
    }
}

pub struct SbsaGWdt {
    base: NonNull<SbsaGWdtRegs>,
}

unsafe impl Send for SbsaGWdt {}
unsafe impl Sync for SbsaGWdt {}

impl SbsaGWdt {
    pub const fn new(base: *mut u8) -> Self {
        Self {
            base: NonNull::new(base).unwrap().cast(),
        }
    }

    const fn regs(&self) -> &SbsaGWdtRegs {
        unsafe { self.base.as_ref() }
    }

    pub fn init(&self) {
        // 可以设置看门狗计数超时值，
        // 如果使能之前未写WOR，那么计数超时值默认为0x3000000，即1s
        //self.regs().wor.set(0x3000000);

        info!("Enable Watchdog");
        // Enable Watchdog
        // 同时写此WCS寄存器也会有喂狗的效果
        self.regs().wcs.set(1);
    }

    pub fn stop(&self) {
        info!("Stop Watchdog");
        self.regs().wcs.set(0);
    }

    pub fn get_ctrl_state(&self) -> u32 {
        self.regs().wcs.get()
    }

    /// 设置超时时间, 单位毫秒
    /// 写WOR寄存器会直接将 当前sys_cnt+WOR寄存器储存的值更新到WCV寄存器中
    pub fn set_timeout(&self, ms: usize) -> usize {
        let counter = ms as u64 * (timer_freq() / 1000);

        //let counter: u32 = ms as u32 * (0x3000000 / 1000);
        // 如果使能之前未写WOR，那么计数超时值默认为0x3000000，即1s
        self.regs().wor.set(counter as u32);

        // Get current sys_cnt
        self.get_sys_cnt()
    }

    pub fn get_timeout(&self) -> u32 {
        self.regs().wor.get()
    }

    pub fn get_sys_cnt(&self) -> usize {
        (self.get_wcv() - self.regs().wor.get() as u64) as usize
    }

    /// 喂狗操作
    /// * 超时：sys_cnt 的计数值大于当前WCV寄存器存储的比较值。
    /// 比较值wcv = 当前sys_cnt + 计数超时值wor
    /// ws0一次超时, 控制器报中断, 后需要进行喂狗，若在计数时间内(WOR)无喂狗操作则会二次超时;
    /// ws1二次超时, 控制器发起复位。
    pub fn feed_wdt(&self) {
        self.regs().wrr.set(0);
    }

    /// 获取看门狗计数的比较值
    pub fn get_wcv(&self) -> u64 {
        let cvl: u32 = self.regs().wcvh.get();
        let cvh: u32 = self.regs().wcvl.get();
        ((cvh as u64) << 32) | (cvl as u64)
    }

    /// 获取看门狗ID
    /// eg: 飞腾派 0x819
    pub fn get_id(&self) -> u32 {
        self.regs().iidr.get()
    }

    pub fn handle_ws0_interrupt(&self) {
        info!("Handle ws0 first timeout");
        self.feed_wdt()
    }
}

pub fn timer_freq() -> u64 {
    let mut freq = 0;
    unsafe { core::arch::asm!("mrs {0}, cntfrq_el0", out(reg) freq) };
    freq
}

use crate::mem::PhysAddr;
use crate::mem::phys_to_virt;
use crate::platform::aarch64_common::gic;
use kspin::SpinNoIrq;

static mut FEEDS: usize = 8;
static WDT0: SpinNoIrq<SbsaGWdt> = SpinNoIrq::new(SbsaGWdt::new(
    phys_to_virt(pa!(axconfig::devices::WDT0_PADDR)).as_mut_ptr(),
));

static WDT1: SpinNoIrq<SbsaGWdt> = SpinNoIrq::new(SbsaGWdt::new(
    phys_to_virt(pa!(axconfig::devices::WDT1_PADDR)).as_mut_ptr(),
));

/// 初始化Watchdog
pub fn watchdog_init() {
    // 中断号
    const WDT0_IRQ: usize = 0xa4 + 32;
    const WDT1_IRQ: usize = 0xa5 + 32;
    crate::irq::set_enable(WDT0_IRQ, true);
    crate::irq::set_enable(WDT1_IRQ, true);

    //注册中断处理函数
    gic::register_handler(WDT0_IRQ, handle_watchdog_ws0_int);
    gic::register_handler(WDT1_IRQ, handle_watchdog_ws0_int);
    info!("Watchdog interrupt {} is enabled.", WDT0_IRQ);

    // 打印看门狗ID
    info!("Watchdog ID0={:#x}, ID1={:#x}", WDT0.lock().get_id(), WDT1.lock().get_id());

    // 设置超时值WOR = TimerFrequecy * seconds[1, 89] 
    info!("Get Timer freq={}", timer_freq());

    // 设置超时时间, 未设置则默认1s超时
    let sys_cnt0 = WDT0.lock().set_timeout(2000);
    //let sys_cnt1 = WDT1.lock().set_timeout(500);
    info!("Now WDT0 sys_cnt={}", sys_cnt0);

    // Start Watchdog
    WDT0.lock().init();
    //WDT1.lock().init();
}

/// GPIO中断处理函数
pub fn handle_watchdog_ws0_int() {
    info!("handle_watchdog_ws0_int");

    let wdt0 = WDT0.lock();
    //let wdt1 = WDT1.lock();

    let wcs0 = wdt0.get_ctrl_state();
    //let wcs1 = wdt1.get_ctrl_state();

    /*
    let ws0 = wcs & 0x2; // bit1
    let ws1 = wcs & 0x4; // bit2
    if ws1 != 0 {
        warn!("Watchdog ws1 second timeout");
    }
    */

    info!("Get WDT0 WCS={:#x}, WCV={}, sys_cnt={}, ", wcs0, wdt0.get_wcv(), wdt0.get_sys_cnt());
    //info!("Get WDT1 WCS={:#x}, WCV={}, sys_cnt={}, ", wcs1, wdt1.get_wcv(), wdt1.get_sys_cnt());

    // 发生了ws0一次超时中断，
    // 喂狗， 预防ws1二次超时复位
    //WDT.lock().feed_wdt();

    unsafe {
        if FEEDS > 0 {
            wdt0.feed_wdt();
            //wdt1.feed_wdt();

            FEEDS -= 1;
        } else {
            warn!("Watchdog ws1 second timeout");
            wdt0.stop();
            //wdt1.stop();
        }
    }
}
