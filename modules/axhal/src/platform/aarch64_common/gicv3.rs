extern crate alloc;

use alloc::boxed::Box;
use crate::{arch::disable_irqs, cpu::this_cpu_id, irq::IrqHandler, mem::phys_to_virt};
use arm_gic_driver::*;
use axconfig::devices::{GICD_PADDR, GICR_PADDR, UART_IRQ};
use core::{panic, ptr::NonNull};
use kspin::SpinNoIrq;
use arm_gicv2::{translate_irq, InterruptType};
#[cfg(feature = "hv")]
use arm_gicv2::GicHypervisorInterface;
use memory_addr::{MemoryAddr, PhysAddr};

use aarch64_cpu::registers::{ICC_SRE_EL2, SCTLR_EL3::I};
use tock_registers::interfaces::{Readable, Writeable};

/// The maximum number of IRQs.
pub const MAX_IRQ_COUNT: usize = 1024;

#[cfg(not(feature = "hv"))]
/// The timer IRQ number.
pub const TIMER_IRQ_NUM: usize = arm_gic_driver::IntId::ppi(14).to_u32() as usize;

#[cfg(feature = "hv")]
/// Non-secure EL2 Physical Timer irq number.
pub const TIMER_IRQ_NUM: usize = arm_gic_driver::IntId::ppi(10).to_u32() as usize;

/// The UART IRQ number.
pub const UART_IRQ_NUM: usize = arm_gic_driver::IntId::spi(UART_IRQ as u32).to_u32() as usize;
/// The IPI IRQ number.
pub const IPI_IRQ_NUM: usize = translate_irq(1, InterruptType::SGI).unwrap();

const GICD_BASE: PhysAddr = pa!(GICD_PADDR);
const GICR_BASE: PhysAddr = pa!(GICR_PADDR);

static GICD: SpinNoIrq<Option<arm_gic_driver::v3::Gic>> = SpinNoIrq::new(None);
static GICR: SpinNoIrq<Option<Box<dyn arm_gic_driver::local::Interface>>> = SpinNoIrq::new(None);

/// Enables or disables the given IRQ.
pub fn set_enable(irq_num: usize, enabled: bool) {
    use arm_gic_driver::local::cap::ConfigLocalIrq;

    let mut gicd = GICD.lock();
    let d = gicd.as_mut().unwrap();

    if irq_num < 32 {
        trace!("GICR set enable: {} {}", irq_num, enabled);

        if enabled {
            d.get_gicr().irq_enable(irq_num.into()).unwrap();
        } else {
            d.get_gicr().irq_disable(irq_num.into()).unwrap();
        }
    } else {
        trace!("GICD set enable: {} {}", irq_num, enabled);

        if enabled {
            d.irq_enable(irq_num.into()).unwrap();
        } else {
            d.irq_disable(irq_num.into()).unwrap();
        }
    }
}

/// Registers an IRQ handler for the given IRQ.
///
/// It also enables the IRQ if the registration succeeds. It returns `false` if
/// the registration failed.
pub fn register_handler(irq_num: usize, handler: IrqHandler) -> bool {
    trace!("register handler irq {}", irq_num);
    crate::irq::register_handler_common(irq_num, handler)
}

/// Fetches the IRQ number.
pub fn fetch_irq() -> usize {
    GICR.lock()
        .as_mut()
        .unwrap()
        .ack()
        .unwrap_or_default()
        .into()
}

/// Dispatches the IRQ.
///
/// This function is called by the common interrupt handler. It looks
/// up in the IRQ handler table and calls the corresponding handler. If
/// necessary, it also acknowledges the interrupt controller after handling.
pub fn dispatch_irq(irq_num: usize) {
    let intid: Option<IrqId>;
    if irq_num == 0 {
        intid = GICR.lock().as_mut().unwrap().ack();
        trace!("interrupt {:?}", intid.unwrap());
    } else {
        intid = Some(IrqId::from(irq_num));
    }
    if let Some(intid) = intid {
        crate::irq::dispatch_irq_common(intid.into());
        GICR.lock().as_mut().unwrap().eoi(intid);
    }
}

/// Reads and returns the value of the given aarch64 system register.
macro_rules! read_sysreg {
    ($name:ident) => {
        {
            let mut value: u64;
            unsafe{::core::arch::asm!(
                concat!("mrs {value:x}, ", ::core::stringify!($name)),
                value = out(reg) value,
                options(nomem, nostack),
            );}
            value
        }
    }
}

/// Writes the given value to the given aarch64 system register.
macro_rules! write_sysreg {
    ($name:ident, $value:expr) => {
        {
            let v: u64 = $value;
            unsafe{::core::arch::asm!(
                concat!("msr ", ::core::stringify!($name), ", {value:x}"),
                value = in(reg) v,
                options(nomem, nostack),
            )}
        }
    }
}

#[cfg(feature = "hv")]
pub fn inject_interrupt(vector: usize) {
    // mask
    const LR_VIRTIRQ_MASK: usize = (1 << 32) - 1;

    let elsr: u64 = read_sysreg!(ich_elrsr_el2);
    let vtr = read_sysreg!(ich_vtr_el2) as usize;
    let lr_num: usize = (vtr & 0xf) + 1;
    let mut free_lr = -1 as isize;
    for i in 0..lr_num {
        // find a free list register
        if (1 << i) & elsr > 0 {
            if free_lr == -1 {
                free_lr = i as isize;
            }
            continue;
        }
        let lr_val = read_lr(i) as usize;
        // if a virtual interrupt is enabled and equals to the physical interrupt irq_id
        if (lr_val & LR_VIRTIRQ_MASK) == vector {
            trace!("virtual irq {} enables again", vector);
        }
    }
    trace!("use free lr {} to inject irq {}", free_lr, vector);

    if free_lr == -1 {
        panic!("No free list register to inject IRQ {}", vector);
    } else {
        let mut val = vector as u64; // vector
        val |= 1 << 60; // group 1
        val |= 1 << 62; // state pending
        // hardware interrupt not supported
        write_lr(free_lr as usize, val);
    }
}

fn read_lr(id: usize) -> u64 {
    let id = id as u64;
    match id {
        //TODO get lr size from gic reg
        0 => read_sysreg!(ich_lr0_el2),
        1 => read_sysreg!(ich_lr1_el2),
        2 => read_sysreg!(ich_lr2_el2),
        3 => read_sysreg!(ich_lr3_el2),
        4 => read_sysreg!(ich_lr4_el2),
        5 => read_sysreg!(ich_lr5_el2),
        6 => read_sysreg!(ich_lr6_el2),
        7 => read_sysreg!(ich_lr7_el2),
        8 => read_sysreg!(ich_lr8_el2),
        9 => read_sysreg!(ich_lr9_el2),
        10 => read_sysreg!(ich_lr10_el2),
        11 => read_sysreg!(ich_lr11_el2),
        12 => read_sysreg!(ich_lr12_el2),
        13 => read_sysreg!(ich_lr13_el2),
        14 => read_sysreg!(ich_lr14_el2),
        15 => read_sysreg!(ich_lr15_el2),
        _ => {
            panic!("invalid lr id {}", id);
        }
    }
}

fn write_lr(id: usize, val: u64) {
    let id = id as u64;
    match id {
        0 => write_sysreg!(ich_lr0_el2, val),
        1 => write_sysreg!(ich_lr1_el2, val),
        2 => write_sysreg!(ich_lr2_el2, val),
        3 => write_sysreg!(ich_lr3_el2, val),
        4 => write_sysreg!(ich_lr4_el2, val),
        5 => write_sysreg!(ich_lr5_el2, val),
        6 => write_sysreg!(ich_lr6_el2, val),
        7 => write_sysreg!(ich_lr7_el2, val),
        8 => write_sysreg!(ich_lr8_el2, val),
        9 => write_sysreg!(ich_lr9_el2, val),
        10 => write_sysreg!(ich_lr10_el2, val),
        11 => write_sysreg!(ich_lr11_el2, val),
        12 => write_sysreg!(ich_lr12_el2, val),
        13 => write_sysreg!(ich_lr13_el2, val),
        14 => write_sysreg!(ich_lr14_el2, val),
        15 => write_sysreg!(ich_lr15_el2, val),
        _ => {
            panic!("invalid lr id {}", id);
        }
    }
}

fn send_sgi_inner(aff3: u8, aff2: u8, aff1: u8, target: u8, vector: usize, to_all: bool) {
    let value = 
        ((vector & 0xF) << 24) |            // vector
        (1 << target) |                     // target bitmap
        ((aff1 as usize) << 16) |           // affinity level 1
        ((aff2 as usize) << 32) |           // affinity level 2
        ((aff3 as usize) << 48) |           // affinity level 3
        ((to_all as usize) << 40);          // interrupt routing mode

    write_sysreg!(icc_sgi1r_el1, value as _);
}

/// Sends Software Generated Interrupt (SGI)(s) (usually IPI) to the given dest CPU.
pub fn send_sgi_one(dest: usize, vector: usize) {
    #[cfg(platform_family = "aarch64-rk3588j")]
    {
        // learnt from hVisor, that rockchip socs follow the 0.0.x.0 affinity scheme
        // while other socs follow 0.0.0.x
        //
        // the best and standard way is reading
        send_sgi_inner(0, 0, dest as _, 0, vector, false);
    }
    #[cfg(not(platform_family = "aarch64-rk3588j"))]
    {
        // the default affinity scheme is 0.0.0.x
        send_sgi_inner(0, 0, 0, dest as _, vector, false);
    }
}

/// Sends a broadcast IPI to all CPUs.
pub fn send_sgi_all(vector: usize) {
    send_sgi_inner(0, 0, 0, 0, vector, true);
}

// dummy implementation
pub struct MyVgic{}

/// Initializes GICD, GICC on the primary CPU.
pub(crate) fn init_primary() {
    info!("Initialize GICv3...");
    let mut gicd = arm_gic_driver::v3::Gic::new(
        NonNull::new(phys_to_virt(GICD_BASE).as_mut_ptr()).unwrap(),
        NonNull::new(phys_to_virt(GICR_BASE).as_mut_ptr()).unwrap(),
        arm_gic_driver::v3::Security::OneNS,
    );

    debug!("Initializing GICD at {:#x}", GICD_BASE);
    gicd.open().unwrap();

    debug!(
        "Initializing GICR for BSP. Global GICR base at {:#x}",
        GICR_BASE
    );
    let mut interface = gicd.cpu_local().unwrap();
    interface.open().unwrap();

    GICD.lock().replace(gicd);
    GICR.lock().replace(interface);

    // SAFETY: Set the SRE[0] bit to 1 to enable Group 1 interrupts.
    // ICC_SRE_EL2.set(0b1);

    // let waker = self[current_cpu().id].WAKER.get();
    // self[current_cpu().id].WAKER.set(waker & !GICR_WAKER_PSLEEP_BIT as u32);
    // while (self[current_cpu().id].WAKER.get() & GICR_WAKER_CASLEEP_BIT as u32) != 0 {}

    // let gicd = arm_gic_driver::v3::Gic::new(
    //     NonNull::new(phys_to_virt(GICD_BASE).as_mut_ptr()).unwrap(),
    //     NonNull::new(phys_to_virt(GICC_BASE).as_mut_ptr()).unwrap(),
    //     arm_gic_driver::v3::Security::OneNS,
    // );
    // let interface = gicd.cpu_interface();

    // GICD.lock().replace(gicd);
    // GICC.lock().replace(interface);

    // disable_irqs();
}

/// Initializes GICR on secondary CPUs.
#[cfg(feature = "smp")]
pub(crate) fn init_secondary() {
    let mut interface = GICD.lock().as_mut().unwrap().cpu_local().unwrap();
    interface.open().unwrap();
    GICR.lock().replace(interface);
}
