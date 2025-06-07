use crate::{arch::disable_irqs, cpu::this_cpu_id, irq::IrqHandler, mem::phys_to_virt};
use arm_gic_driver::*;
use axconfig::devices::{GICC_PADDR, GICD_PADDR, GICR_PADDR, UART_IRQ};
use core::ptr::NonNull;
use kspin::SpinNoIrq;
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

const GICD_BASE: PhysAddr = pa!(GICD_PADDR);
const GICC_BASE: PhysAddr = pa!(GICC_PADDR);
const GICR_BASE: PhysAddr = pa!(GICR_PADDR);

static GICD: SpinNoIrq<Option<arm_gic_driver::v3::Gic>> = SpinNoIrq::new(None);
static GICC: SpinNoIrq<Option<Box<dyn InterfaceCPU>>> = SpinNoIrq::new(None);

/// Enables or disables the given IRQ.
pub fn set_enable(irq_num: usize, enabled: bool) {
    trace!("GICD set enable: {} {}", irq_num, enabled);

    // let mut gicd = GICD.lock();
    // let d = gicd.as_mut().unwrap();
    // if enabled {
    //     d.irq_enable(irq_num.into());
    // } else {
    //     d.irq_disable(irq_num.into());
    // }
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
    GICC.lock()
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
        intid = GICC.lock().as_mut().unwrap().ack();
        info!("interrupt {:?}", intid.unwrap());
    } else {
        intid = Some(IrqId::from(irq_num));
    }
    if let Some(intid) = intid {
        crate::irq::dispatch_irq_common(intid.into());
        GICC.lock().as_mut().unwrap().eoi(intid);
    }
}

const GICR_WAKER_PSLEEP_BIT: usize = 0x2;
const GICR_WAKER_CASLEEP_BIT: usize = 0x4;
const GICR_WAKER_OFFSET: usize = 0x14;

fn wake_up_gicr(cpu_id: usize) {
    let cur_gicr_base = phys_to_virt(GICR_BASE).add(cpu_id * 0x20000);

    debug!("CPU {} GICR base: {:#x}", cpu_id, cur_gicr_base);

    let gicr_waker = cur_gicr_base.add(GICR_WAKER_OFFSET).as_mut_ptr_of::<u32>();

    unsafe {
        // Set the GICR WAKER register to wake up the CPU.
        *gicr_waker &= !(GICR_WAKER_PSLEEP_BIT as u32);

        // Wait until the GICR WAKER register is not in sleep state.
        while (*gicr_waker & GICR_WAKER_CASLEEP_BIT as u32) != 0 {
            // Spin until the CPU is awake.
        }
    }
}

/// Initializes GICD, GICC on the primary CPU.
pub(crate) fn init_primary() {
    warn!("Initialize GICv3...");

    // wake_up_gicr(this_cpu_id());

    // SAFETY: Set the SRE[0] bit to 1 to enable Group 1 interrupts.
    ICC_SRE_EL2.set(0b1);

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

/// Initializes GICC on secondary CPUs.
#[cfg(feature = "smp")]
pub(crate) fn init_secondary() {
    // let interface = GICD.lock().as_mut().unwrap().cpu_interface();
    // GICC.lock().replace(interface);
    // GICC.lock().as_mut().unwrap().setup();
    // wake_up_gicr(this_cpu_id());
}
