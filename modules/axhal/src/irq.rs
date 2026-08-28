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
    let guard = kernel_guard::NoPreempt::new();
    dispatch_irq(irq_num);
    drop(guard); // rescheduling may occur when preemption is re-enabled.
    // 注：drop 将会调用下列函数，进行任务抢占切换，导致当前sp发生改变;
    // 此中断处理函数返回后，会在arm_vcpu::HANDLE_CURRENT_IRQ函数中以错误的SP寄存器，执行了.Lexception_return_el2返回操作，
    // 将导致: arm_vcpu: Exception Class: 0x21 (Instruction Abort). EL2取指异常
    //
    // -> release() (若使能抢占) -> enable_preempt()
    // -> current_check_preempt_pending() -> preempt_resched() -...-> context.rs::context_switch()

    true
}
