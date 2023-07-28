use crate::mem::{virt_to_phys, PhysAddr, VirtAddr};

pub const CPU_HWID: [usize; 8] = [0x00, 0x100, 0x200, 0x300, 0x400, 0x500, 0x600, 0x700];

/// Starts the given secondary CPU with its boot stack.
pub fn start_secondary_cpu(cpu_id: usize, stack_top: PhysAddr) {
    extern "C" {
        fn _start_secondary();
    }
    let entry = virt_to_phys(VirtAddr::from(_start_secondary as usize));
    crate::platform::aarch64_common::psci::psci_cpu_on(
        CPU_HWID[cpu_id],
        entry.as_usize(),
        stack_top.as_usize(),
    );
}
