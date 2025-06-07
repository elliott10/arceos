extern crate alloc;

use crate::{arch::disable_irqs, irq::IrqHandler, mem::phys_to_virt};
use alloc::boxed::Box;
use arm_gic_driver::*;
use axconfig::devices::{GICD_PADDR, GICR_PADDR, UART_IRQ};
use core::ptr::NonNull;
use kspin::SpinNoIrq;
use memory_addr::PhysAddr;

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

/// Initializes GICD, GICR on the primary CPU.
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

    disable_irqs();
}

/// Initializes GICR on secondary CPUs.
#[cfg(feature = "smp")]
pub(crate) fn init_secondary() {
    let mut interface = GICD.lock().as_mut().unwrap().cpu_local().unwrap();
    interface.open().unwrap();
    GICR.lock().replace(interface);
}
