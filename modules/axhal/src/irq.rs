//! Interrupt management.

use handler_table::HandlerTable;

use crate::platform::irq::{MAX_IRQ_COUNT, dispatch_irq};
use crate::trap::{IRQ, register_trap_handler};

pub use crate::platform::irq::{register_handler, set_enable};

#[cfg(all(target_arch = "aarch64", feature = "hv"))]
pub use crate::platform::irq::{MyVgic, inject_interrupt};

#[cfg(feature = "ipi")]
pub use crate::platform::irq::{IPI_IRQ_NUM, send_sgi_all, send_sgi_one};

#[cfg(target_arch = "aarch64")]
pub use crate::platform::irq::fetch_irq;

/// The type if an IRQ handler.
pub type IrqHandler = handler_table::Handler;

static IRQ_HANDLER_TABLE: HandlerTable<MAX_IRQ_COUNT> = HandlerTable::new();

/// Deferred task rescheduling while an EL2 VCPU IRQ frame is active.
#[cfg(feature = "deferred-irq-resched")]
#[crate_interface::def_interface]
pub trait IrqReschedIf {
    /// Prevents preemption from switching tasks until the IRQ handler returns.
    fn enter_irq();

    /// Ends the IRQ deferral region without performing a task switch.
    fn exit_irq();
}

/// Platform-independent IRQ dispatching.
#[allow(dead_code)]
pub(crate) fn dispatch_irq_common(irq_num: usize) {
    trace!("IRQ {}", irq_num);
    if !IRQ_HANDLER_TABLE.handle(irq_num) {
        warn!("Unhandled IRQ {}", irq_num);
    }
}

/// Platform-independent IRQ handler registration.
///
/// It also enables the IRQ if the registration succeeds. It returns `false` if
/// the registration failed.
#[allow(dead_code)]
pub(crate) fn register_handler_common(irq_num: usize, handler: IrqHandler) -> bool {
    if irq_num < MAX_IRQ_COUNT && IRQ_HANDLER_TABLE.register_handler(irq_num, handler) {
        set_enable(irq_num, true);
        return true;
    }
    warn!("register handler for IRQ {} failed", irq_num);
    false
}

/// Core IRQ handling routine, registered at `axhal::trap::IRQ`,
/// which dispatches IRQs to registered handlers.
///
/// Note: this function is denoted as public here because it'll be called by the
/// hypervisor for hypervisor reserved IRQ handling.
#[register_trap_handler(IRQ)]
pub fn handler_irq(irq_num: usize) -> bool {
    #[cfg(feature = "deferred-irq-resched")]
    crate_interface::call_interface!(IrqReschedIf::enter_irq);

    let guard = kernel_guard::NoPreempt::new();
    dispatch_irq(irq_num);
    drop(guard);
    // 为了防止drop() 调用的enable_preempt函数，进行任务抢占切换，
    // 而导致当前SP发生改变，将破坏中断处理函数arm_vcpu::HANDLE_CURRENT_IRQ中的SP寄存器，
    // 因此，这里延迟了任务抢占切换。

    #[cfg(feature = "deferred-irq-resched")]
    crate_interface::call_interface!(IrqReschedIf::exit_irq);

    true
}
