//! Minimal PLIC support temporarily disabled.

use kspin::SpinNoIrq;
use lazyinit::LazyInit;

use riscv::register::sie;
// use riscv_plic::{HartContext, InterruptSource, Plic};
use sbi_rt::{HartMask, send_ipi};

use memory_addr::{PhysAddr, pa};

use crate::cpu::this_cpu_id;
use crate::irq::IrqHandler;
use crate::mem::phys_to_virt;

// struct Context {
//     hart_id: usize,
// }

// impl HartContext for Context {
//     fn index(self) -> usize {
//         self.hart_id * 2 + 1
//     }
// }

// impl Context {
//     const fn new(hart_id: usize) -> Self {
//         Self { hart_id }
//     }
// }

// struct Interrupt {
//     irq_num: usize,
// }

// impl InterruptSource for Interrupt {
//     fn id(self) -> core::num::NonZeroU32 {
//         core::num::NonZeroU32::new(self.irq_num as u32).unwrap()
//     }
// }

/// `Interrupt` bit in `scause`
pub(super) const INTC_IRQ_BASE: usize = 1 << (usize::BITS - 1);

/// Supervisor software interrupt in `scause`
#[allow(unused)]
pub(super) const S_SOFT: usize = INTC_IRQ_BASE + 1;

/// Supervisor timer interrupt in `scause`
pub(super) const S_TIMER: usize = INTC_IRQ_BASE + 5;

/// Supervisor external interrupt in `scause`
pub(super) const S_EXT: usize = INTC_IRQ_BASE + 9;

static TIMER_HANDLER: LazyInit<IrqHandler> = LazyInit::new();

static IPI_HANDLER: LazyInit<IrqHandler> = LazyInit::new();

/// The maximum number of IRQs.
pub const MAX_IRQ_COUNT: usize = 1024;

/// The timer IRQ number (supervisor timer interrupt in `scause`).
pub const TIMER_IRQ_NUM: usize = S_TIMER;

/// The IPI IRQ number (supervisor software interrupt in `scause`).
pub const IPI_IRQ_NUM: usize = S_SOFT;

macro_rules! with_cause {
    ($cause: expr,
        @TIMER => $timer_op: expr,
        @IPI => $ipi_op: expr,
        @EXT => $ext_op: expr $(,)?) => {
        match $cause {
            S_TIMER => $timer_op,
            S_SOFT => $ipi_op,
            S_EXT => $ext_op,
            _ => panic!("invalid trap cause: {:#x}", $cause),
        }
    };
}

// const PLIC_BASE: PhysAddr = pa!(axconfig::PLIC_PADDR);

// static PLIC: SpinNoIrq<Plic> = SpinNoIrq::new(Plic::new(phys_to_virt(PLIC_BASE).as_mut_ptr()));

/// Enables or disables the given IRQ.
pub fn set_enable(irq_num: usize, enabled: bool) {
    // TODO: set enable in PLIC
    // let source = Interrupt { irq_num };
    // let context = Context::new(this_cpu_id());

    // if enabled {
    //     PLIC.lock().enable(source, context);
    // } else {
    //     PLIC.lock().disable(source, context);
    // }
}

/// Registers an IRQ handler for the given IRQ.
///
/// It also enables the IRQ if the registration succeeds. It returns `false` if
/// the registration failed.
pub fn register_handler(scause: usize, handler: IrqHandler) -> bool {
    with_cause!(
        scause,
        @TIMER => if !TIMER_HANDLER.is_inited() {
            TIMER_HANDLER.init_once(handler);
            true
        } else {
            false
        },
        @IPI => if !IPI_HANDLER.is_inited() {
            IPI_HANDLER.init_once(handler);
            true
        } else {
            false
        },
        @EXT => crate::irq::register_handler_common(scause & !INTC_IRQ_BASE, handler),
    )
}

/// Sends Software Generated Interrupt (SGI)(s) (usually IPI) to the given dest CPU.
pub fn send_sgi_one(dest_cpu_id: usize, _irq_num: usize) {
    let res = send_ipi(HartMask::from_mask_base(1, dest_cpu_id));
    if res.is_err() {
        warn!("send_sgi_one failed: {:?}", res);
    }
}

/// Sends a broadcast IPI to all CPUs.
pub fn send_sgi_all(_irq_num: usize) {
    for i in 0..axconfig::SMP {
        if i != this_cpu_id() {
            let res = send_ipi(HartMask::from_mask_base(1, i));
            if res.is_err() {
                warn!("send_sgi_all failed: {:?}", res);
                break;
            }
        }
    }
}

/// Dispatches the IRQ.
///
/// This function is called by the common interrupt handler. It looks
/// up in the IRQ handler table and calls the corresponding handler. If
/// necessary, it also acknowledges the interrupt controller after handling.
pub fn dispatch_irq(scause: usize) {
    with_cause!(
        scause,
        @TIMER => {
            trace!("IRQ: timer");
            TIMER_HANDLER();
        },
        @IPI => {
            trace!("IRQ: IPI");
            IPI_HANDLER();
        },
        @EXT => {
            // if let Some(irq_num) = PLIC.lock().claim(Context::new(this_cpu_id())) {
            //     let irq_num = irq_num.get() as usize;
            //     trace!("IRQ: external {}", irq_num);
            //     crate::irq::dispatch_irq_common(irq_num);
            //     PLIC.lock().complete(Context::new(this_cpu_id()), Interrupt { irq_num });
            // }
            crate::irq::dispatch_irq_common(0);
        },
    );
}

pub(super) fn init_percpu() {
    // PLIC.lock().init_by_context(Context::new(this_cpu_id()));

    // enable soft interrupts, timer interrupts, and external interrupts
    unsafe {
        sie::set_ssoft();
        sie::set_stimer();
        sie::set_sext();
    }
}
