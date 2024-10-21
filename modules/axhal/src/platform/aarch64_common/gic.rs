use crate::{irq::IrqHandler, mem::phys_to_virt};
use arm_gic::{translate_irq, InterruptType};
use memory_addr::PhysAddr;
use spinlock::SpinNoIrq;

/// The maximum number of IRQs.
pub const MAX_IRQ_COUNT: usize = 1024;

/// The timer IRQ number.
pub const TIMER_IRQ_NUM: usize = translate_irq(14, InterruptType::PPI).unwrap();

/// The UART IRQ number.
pub const UART_IRQ_NUM: usize = translate_irq(axconfig::UART_IRQ, InterruptType::SPI).unwrap();

#[cfg(not(feature = "gicv3"))]
use arm_gic::gic_v2::{GicDistributor, GicCpuInterface};

use arm_gic::gic_v3::{GICD, gicc_get_current_irq, gicc_clear_current_irq, gic_glb_init, gic_cpu_init};

#[cfg(not(feature = "gicv3"))]
    static GICD: SpinNoIrq<GicDistributor> =
        SpinNoIrq::new(GicDistributor::new(phys_to_virt(PhysAddr::from(axconfig::GICD_PADDR)).as_mut_ptr()));

#[cfg(not(feature = "gicv3"))]
    // per-CPU, no lock
    static GICC: GicCpuInterface = GicCpuInterface::new(phys_to_virt(PhysAddr::from(axconfig::GICC_PADDR)).as_mut_ptr());

#[cfg(feature = "gicv3")]
#[export_name = "PLATFORM_GICD_BASE"]
pub static GICD_BASE: usize = axconfig::PHYS_VIRT_OFFSET + axconfig::GICD_PADDR;
#[cfg(feature = "gicv3")]
#[export_name = "PLATFORM_GICR_BASE"]
pub static GICR_BASE: usize = axconfig::PHYS_VIRT_OFFSET + axconfig::GICR_PADDR;

/// Enables or disables the given IRQ.
pub fn set_enable(irq_num: usize, enabled: bool) {
    trace!("GICD set enable: {} {}", irq_num, enabled);

    #[cfg(not(feature = "gicv3"))]
    {
        GICD.lock().set_enable(irq_num as _, enabled);
    }

    #[cfg(feature = "gicv3")]
    {   
        use arm_gic::gic_v3::{gic_set_enable, gic_set_prio};
        use tock_registers::interfaces::Readable;
        gic_set_enable(irq_num, enabled);
        gic_set_prio(irq_num, 0x1);
        GICD.set_route(irq_num, aarch64_cpu::registers::MPIDR_EL1.get() as usize);
    }   
}

/// Send ipi to cpu
pub fn ipi_send(cpu_id: usize, ipi_id: usize) {
    let GIC_SGIS_NUM: usize = 16;
    if ipi_id < GIC_SGIS_NUM {
        #[cfg(not(feature = "gicv3"))]
        warn!("ipi_send unimplemented in gicv2");
        //GICD.send_sgi(Platform::cpuid_to_cpuif(cpu_id), ipi_id);

        #[cfg(feature = "gicv3")]
        GICD.send_sgi(cpu_id, ipi_id);
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

/// Dispatches the IRQ.
///
/// This function is called by the common interrupt handler. It looks
/// up in the IRQ handler table and calls the corresponding handler. If
/// necessary, it also acknowledges the interrupt controller after handling.
pub fn dispatch_irq(_unused: usize) {
    #[cfg(not(feature = "gicv3"))]
    GICC.handle_irq(|irq_num| crate::irq::dispatch_irq_common(irq_num as _));

    #[cfg(feature = "gicv3")]
    if let Some(id) = gicc_get_current_irq() {
        trace!("Got irq: {}", id);
        if id >= 1022 {
            return;
        }
        // let begin = time_current_us();
        crate::irq::dispatch_irq_common(id);
        let handled_by_hypervisor = true;
        // let end = time_current_us();

        gicc_clear_current_irq(id as u32, handled_by_hypervisor);
    }   
}

/// Initializes GICD, GICC on the primary CPU.
pub(crate) fn init_primary() {
    #[cfg(not(feature = "gicv3"))]
    {
        info!("Initialize GICv2...");
        GICD.lock().init();
        GICC.init();
    }

    #[cfg(feature = "gicv3")]
    {
        info!("Initialize GICv3...");

        //if current_cpu().id == 0 
        gic_glb_init();

        gic_cpu_init();
    }
}

/// Initializes GICC on secondary CPUs.
#[cfg(feature = "smp")]
pub(crate) fn init_secondary() {
    #[cfg(not(feature = "gicv3"))]
    GICC.init();

    #[cfg(feature = "gicv3")]
    gic_cpu_init();
}
