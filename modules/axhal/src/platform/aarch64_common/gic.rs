use crate::{irq::IrqHandler, mem::phys_to_virt};
use arm_gicv2::{GicCpuInterface, GicDistributor, InterruptType, translate_irq};
use axconfig::devices::{GICC_PADDR, GICD_PADDR, UART_IRQ};
use kspin::SpinNoIrq;
use memory_addr::PhysAddr;

#[cfg(feature = "hv")]
use arm_gicv2::GicHypervisorInterface;
#[cfg(feature = "hv")]
use axconfig::devices::{GICH_PADDR, GICV_PADDR};

/// The maximum number of IRQs.
pub const MAX_IRQ_COUNT: usize = 1024;

#[cfg(not(feature = "hv"))]
/// The timer IRQ number.
pub const TIMER_IRQ_NUM: usize = translate_irq(14, InterruptType::PPI).unwrap();

#[cfg(feature = "hv")]
/// Non-secure EL2 Physical Timer irq number.
pub const TIMER_IRQ_NUM: usize = translate_irq(10, InterruptType::PPI).unwrap();

/// The UART IRQ number.
pub const UART_IRQ_NUM: usize = translate_irq(UART_IRQ, InterruptType::SPI).unwrap();
/// The IPI IRQ number.
pub const IPI_IRQ_NUM: usize = translate_irq(1, InterruptType::SGI).unwrap();

const GICD_BASE: PhysAddr = pa!(GICD_PADDR);
const GICC_BASE: PhysAddr = pa!(GICC_PADDR);

#[cfg(feature = "hv")]
const GICV_BASE: PhysAddr = pa!(GICV_PADDR);
#[cfg(feature = "hv")]
const GICH_BASE: PhysAddr = pa!(GICH_PADDR);

static GICD: SpinNoIrq<GicDistributor> =
    SpinNoIrq::new(GicDistributor::new(phys_to_virt(GICD_BASE).as_mut_ptr()));

// per-CPU, no lock
static GICC: GicCpuInterface = GicCpuInterface::new(phys_to_virt(GICC_BASE).as_mut_ptr());
#[cfg(feature = "hv")]
static GICV: GicCpuInterface = GicCpuInterface::new(phys_to_virt(GICV_BASE).as_mut_ptr());
#[cfg(feature = "hv")]
static GICH: GicHypervisorInterface =
    GicHypervisorInterface::new(phys_to_virt(GICH_BASE).as_mut_ptr());

/// Enables or disables the given IRQ.
pub fn set_enable(irq_num: usize, enabled: bool) {
    debug!("GICD v2 set enable: {} {}", irq_num, enabled);
    GICD.lock().set_enable(irq_num as _, enabled);
}

/// Sets the priority of the given IRQ.
pub fn set_priority(irq_num: usize, priority: usize) {
    GICD.lock().set_priority(irq_num, priority as u8);
}

/// Sends Software Generated Interrupt (SGI)(s) (usually IPI) to the given dest CPU.
pub fn send_sgi_one(dest_cpu_id: usize, irq_num: usize) {
    GICD.lock().send_sgi(dest_cpu_id, irq_num);
}

/// Sends a broadcast IPI to all CPUs.
pub fn send_sgi_all(irq_num: usize) {
    GICD.lock().send_sgi_all_except_self(irq_num);
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
    GICC.iar() as usize
}

/// Dispatches the IRQ.
///
/// This function is called by the common interrupt handler. It looks
/// up in the IRQ handler table and calls the corresponding handler. If
/// necessary, it also acknowledges the interrupt controller after handling.
pub fn dispatch_irq(irq_no: usize) {
    // I know, `irq_no == 0` seems very strange here.
    // The truth is, the previous design of ArceOS in aarch64 DO NOT fetch the IRQ number from GICC,
    // until the function closure passed to `GICC.handle_irq`.
    // So, the `irq_no` is always 0 when `dispatch_irq` is called by `handler_irq` from inside ArceOS.
    //
    // However, when `handler_irq` is called by the arceos-vmm app, the `irq_no` has been fetched from GICC, so we can not use the `handler_irq` directly.
    // Instead, we call `GICC.eoi` and `GICC.dir` manually.
    if irq_no == 0 {
        GICC.handle_irq(|irq_num| crate::irq::dispatch_irq_common(irq_num as _));
    } else {
        crate::irq::dispatch_irq_common(irq_no as _);
        GICC.eoi(irq_no as _);
        GICC.dir(irq_no as _);
    }
}

/// Initializes GICD, GICC on the primary CPU.
pub(crate) fn init_primary() {
    info!("Initialize GICv2...");
    GICD.lock().init();
    GICC.init();

    #[cfg(feature = "hv")]
    {
        GICV.init();
    }
}

#[cfg(feature = "hv")]
pub fn eoi_irq(irq_num: usize) {
    GICC.eoi(irq_num as _);
    GICC.dir(irq_num as _);
}

/// GICv2 variant of [`super::gicv3::eoi_guest_irq`]: GICv2 list registers
/// don't support HW mapping, just complete the physical interrupt.
#[cfg(feature = "hv")]
pub fn eoi_guest_irq(irq_num: usize) {
    eoi_irq(irq_num);
}

#[cfg(feature = "hv")]
pub fn inject_interrupt(vector: usize) {
    let hcr = GICH.get_hcr();
    GICH.set_hcr(hcr | 1 << 0);
    let mut lr = 0;
    lr |= vector << 0;
    lr |= 1 << 19;
    lr |= 1 << 28;
    GICH.set_lr(0, lr as u32);
}

#[cfg(feature = "hv")]
pub fn inject_virtual_sgi(_vector: usize) {}

#[cfg(feature = "hv")]
pub fn disable_guest_timers() {
    unsafe {
        core::arch::asm!(
            "msr cntp_ctl_el0, {0}",
            "msr cntv_ctl_el0, {0}",
            "msr cntp_cval_el0, {1}",
            "msr cntv_cval_el0, {1}",
            in(reg) 0u64,
            in(reg) u64::MAX,
            options(nomem, nostack),
        );
    }
}

#[cfg(feature = "hv")]
pub fn write_icc_sgi1r(_value: u64) {
    // For GICv2, SGI generation uses GICD_SGIR rather than system register
}

#[cfg(feature = "hv")]
pub fn clean_up_guest_local_irqs() {
    disable_guest_timers();
    let hcr = GICH.get_hcr();
    GICH.set_hcr(hcr & !1);
    for i in 0..64 {
        GICH.set_lr(i, 0);
    }
}

#[cfg(feature = "hv")]
pub struct MyVgic {}

#[cfg(feature = "hv")]
impl MyVgic {
    pub fn get_gich() -> &'static GicHypervisorInterface {
        &GICH
    }
    pub fn get_gicd() -> &'static SpinNoIrq<GicDistributor> {
        &GICD
    }
    pub fn get_gicc() -> &'static GicCpuInterface {
        &GICC
    }
    pub fn get_gicv() -> &'static GicCpuInterface {
        &GICV
    }
}

/// Initializes GICC on secondary CPUs.
#[cfg(feature = "smp")]
pub(crate) fn init_secondary() {
    GICC.init();
}
