use crate::mem::{virt_to_phys, PhysAddr, VirtAddr};

/// Hart number of bsta1000b board
pub const MAX_HARTS: usize = 8;
/// CPU HWID from cpu device tree nodes with "reg" property
pub const CPU_HWID: [usize; MAX_HARTS] = [0x00, 0x100, 0x200, 0x300, 0x400, 0x500, 0x600, 0x700];

// rk3588 在_start_secondary 第二核启动时，会卡住，只能尝试重新初始化mmu的页表，后能暂时引导进入内核，但是多核可能会卡住或lazy_init panic
// 调试发现，其uboot会对内核入口地址，手动进行 (0x200000) 2MB 对齐, 并进行relocation重定位到该对齐后的地址。

/// Starts the given secondary CPU with its boot stack.
pub fn start_secondary_cpu(cpu_id: usize, stack_top: PhysAddr) {
    if cpu_id >= MAX_HARTS {
        error!("No support for bsta1000b core {}", cpu_id);
        return;
    }
    extern "C" {
        fn _start_secondary();
    }
    let entry = virt_to_phys(VirtAddr::from(_start_secondary as usize));
    crate::platform::aarch64_common::psci::cpu_on(
        CPU_HWID[cpu_id],
        entry.as_usize(),
        stack_top.as_usize(),
    );
}
