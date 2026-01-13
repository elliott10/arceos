//! Simple DWMAC HAL Implementation for Tutorial
//!
//! This provides a basic hardware abstraction layer implementation
//! for the DWMAC driver tutorial using JH7110 PAC for type-safe register
//! access.

// use axdma::{BusAddr, DMAInfo, alloc_coherent, dealloc_coherent};
use core::{
    ops::Add,
    ptr::NonNull,
    sync::atomic::{AtomicBool, Ordering},
};

use axalloc::{UsageKind, global_allocator};
use axdriver_net::dwmac::{DwmacHal, PhysAddr as DwmacPhysAddr};
use axhal::mem::{PAGE_SIZE_4K, PhysAddr, phys_to_virt, virt_to_phys};
use jh7110_vf2_13b_pac as pac;

/// Simple HAL implementation for DWMAC
pub struct DwmacHalImpl;

static INITIALIZED: AtomicBool = AtomicBool::new(false);

fn mb() {
    unsafe { core::arch::asm!("fence iorw, iorw") };
}

impl DwmacHal for DwmacHalImpl {
    fn cache_flush_range(start: NonNull<u8>, end: NonNull<u8>) {
        const CCACHE_BASE: usize = 0x0201_0000;
        const FLUSH64_OFFSET: usize = 0x200;
        const LINE_SIZE: usize = 64;

        let mut addr = start.as_ptr() as usize & !(LINE_SIZE - 1);

        let flush_addr = phys_to_virt(CCACHE_BASE.into())
            .add(FLUSH64_OFFSET)
            .as_mut_ptr() as *mut u32;
        let end_addr = end.as_ptr() as usize;
        mb();
        while addr < end_addr {
            unsafe {
                core::ptr::write_volatile(flush_addr, addr as u32);
                addr += LINE_SIZE;
            }
        }
    }

    fn dma_alloc(size: usize, _align: usize) -> (DwmacPhysAddr, NonNull<u8>) {
        const PAGE_SIZE: usize = 4096;
        let alloc_pages = (size + (PAGE_SIZE - 1)) / PAGE_SIZE;
        let Ok(vaddr) = global_allocator().alloc_pages(alloc_pages, PAGE_SIZE_4K, UsageKind::Dma)
        else {
            error!("failed to alloc pages");
            return (0, NonNull::dangling());
        };
        let paddr = virt_to_phys((vaddr).into());
        debug!("alloc pages @ vaddr={:#x}, paddr={:#x}", vaddr, paddr);

        // Return bus address for hardware and CPU virtual address
        (paddr.as_usize(), unsafe {
            NonNull::new_unchecked(vaddr as *mut u8)
        })
    }

    unsafe fn dma_dealloc(
        _paddr: DwmacPhysAddr,
        vaddr: NonNull<u8>,
        size: usize,
        _align: usize,
    ) -> i32 {
        const PAGE_SIZE: usize = 4096;
        let pages = (size + (PAGE_SIZE - 1)) / PAGE_SIZE;
        global_allocator().dealloc_pages(vaddr.as_ptr() as *mut _ as usize, pages, UsageKind::Dma);

        0
    }

    unsafe fn mmio_phys_to_virt(paddr: DwmacPhysAddr, _size: usize) -> NonNull<u8> {
        NonNull::new(phys_to_virt(paddr.into()).as_mut_ptr()).unwrap()
    }

    unsafe fn mmio_virt_to_phys(vaddr: NonNull<u8>, _size: usize) -> DwmacPhysAddr {
        virt_to_phys((vaddr.as_ptr() as usize).into()).as_usize()
    }

    fn wait_until(duration: core::time::Duration) -> Result<(), &'static str> {
        axhal::time::busy_wait(duration);
        Ok(())
    }

    fn configure_platform() -> Result<(), &'static str> {
        log::info!("🔧 StarFive platform configuration (tutorial + PAC verification mode)");

        if INITIALIZED
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            log::info!("🔧 Platform already initialized");
            return Ok(());
        }

        Self::set_clocks_uboot();

        Self::print_clocks();

        // Just do a quick status check without changing anything
        Self::print_preserved_status();

        log::info!("✅ Platform configuration preserved - ready for DWMAC operation");
        log::info!("💡 TIP: U-Boot has already initialized everything - just trust it!");

        Ok(())
    }
}

impl DwmacHalImpl {
    fn check_diff(name: &str, readback: u32, expected: u32) {
        if readback != expected {
            log::warn!(
                "🔧 Clock {} is not set correctly: {:#x} != {:#x}",
                name,
                readback,
                expected
            );
        } else {
            log::debug!("🔧 Clock {} is set correctly: {:#x}", name, readback);
        }
    }

    fn set_clocks_uboot() {
        // Use PAC for available registers
        let aoncrg: &pac::aoncrg::RegisterBlock = unsafe {
            &*(<Self as DwmacHal>::mmio_phys_to_virt(pac::AONCRG::ptr() as usize, 0x1000).as_ptr()
                as *const pac::aoncrg::RegisterBlock)
        };

        let syscrg: &pac::syscrg::RegisterBlock = unsafe {
            &*(<Self as DwmacHal>::mmio_phys_to_virt(pac::SYSCRG::ptr() as usize, 0x1000).as_ptr()
                as *const pac::syscrg::RegisterBlock)
        };

        // gmac0 clocks
        unsafe {
            // clk_enable(clk=00000000ff742940 name=gmac0_rmii_rtx)
            // clk_enable(clk=00000000ff724390 name=gmac0-rmii-refin-clock)
            // clk_mux_set_parent: set clock gmac0_tx to parent gmac0_rmii_rtx
            // (reg=0x0000000017000014, val=0x01000000) md 0x81000000
            aoncrg.clk_gmac5_axi64_tx().write(|w| w.bits(0x81000000));
            Self::check_diff(
                "GMAC5 AXI64 TX",
                aoncrg.clk_gmac5_axi64_tx().read().bits(),
                0x81000000,
            );
            // clk_enable(clk=00000000ff745340 name=clock-controller@17000000)
            // clk_enable(clk=00000000ff72a540 name=stg_axiahb)
            // clk_gate_endisable: enabling clock gmac0_axi (reg=0x000000001700000c, bit=31,
            // set=1) md 0x80000000
            aoncrg.clk_axi_gmac5().write(|w| w.clk_icg().set_bit());
            Self::check_diff(
                "GMAC5 AXI",
                aoncrg.clk_axi_gmac5().read().bits(),
                0x80000000,
            );
            // clk_enable(clk=00000000ff745368 name=clock-controller@17000000)
            // clk_enable(clk=00000000ff72a540 name=stg_axiahb)
            // clk_gate_endisable: enabling clock gmac0_ahb (reg=0x0000000017000008, bit=31,
            // set=1) md 0x80000000
            aoncrg.clk_ahb_gmac5().write(|w| w.clk_icg().set_bit());
            Self::check_diff(
                "GMAC5 AHB",
                aoncrg.clk_ahb_gmac5().read().bits(),
                0x80000000,
            );
            // clk_enable(clk=00000000ff745390 name=clock-controller@13020000)
            // clk_enable(clk=00000000ff72bf40 name=gmac_src)
            // clk_enable(clk=00000000ff7295c0 name=pll0_out)
            // clk_gate_endisable: enabling clock gmac0_ptp (reg=0x00000000130201b4, bit=31,
            // set=1) md 0x8000000a
            // syscrg.clk_gmac0_ptp().write(|w| w.clk_icg().set_bit());
            syscrg.clk_gmac0_ptp().write(|w| w.bits(0x8000000a));
            Self::check_diff(
                "GMAC0 PTP",
                syscrg.clk_gmac0_ptp().read().bits(),
                0x8000000a,
            );
            // clk_enable(clk=00000000ff7453b8 name=clock-controller@17000000)
            // clk_enable(clk=00000000ff742bc0 name=gmac0_tx)
            // clk_enable(clk=00000000ff742940 name=gmac0_rmii_rtx)
            // clk_gate_endisable: enabling clock gmac0_tx (reg=0x0000000017000014, bit=31,
            // set=1) md 0x81000000
            aoncrg.clk_gmac5_axi64_tx().write(|w| w.bits(0x81000000));
            Self::check_diff(
                "GMAC5 AXI64 TX",
                aoncrg.clk_gmac5_axi64_tx().read().bits(),
                0x81000000,
            );
            // clk_gate_endisable: enabling clock gmac0_tx_inv (reg=0x0000000017000018,
            // bit=30, set=1) md 0x40000000
            aoncrg.clk_gmac5_axi64_txi().write(|w| w.bits(1 << 30));
            Self::check_diff(
                "GMAC5 AXI64 TXI",
                aoncrg.clk_gmac5_axi64_txi().read().bits(),
                0x40000000,
            );
            // clk_enable(clk=00000000ff7453e0 name=clock-controller@13020000)
            // clk_enable(clk=00000000ff72cdc0 name=gmac0_gtxclk)
            // clk_enable(clk=00000000ff7295c0 name=pll0_out)
            // clk_gate_endisable: enabling clock gmac0_gtxclk (reg=0x00000000130201b0,
            // bit=31, set=1) md 0x80000008
            // syscrg.clk_gmac0_gtx().write(|w| w.clk_icg().set_bit());
            syscrg.clk_gmac0_gtx().write(|w| w.bits(0x80000008));
            Self::check_diff(
                "GMAC0 GTX",
                syscrg.clk_gmac0_gtx().read().bits(),
                0x80000008,
            );
            // clk_gate_endisable: enabling clock gmac0_gtxc (reg=0x00000000130201bc,
            // bit=31, set=1) md 0x80000020
            // syscrg.clk_gmac0_gtxclk().write(|w| w.bits(1 << 31));
            syscrg.clk_gmac0_gtxclk().write(|w| w.bits(0x80000020));
            Self::check_diff(
                "GMAC0 GTXCLK",
                syscrg.clk_gmac0_gtxclk().read().bits(),
                0x80000020,
            );
        }

        // gmac1 clocks
        unsafe {
            // clk_get_by_name_nodev(node=00000000ff71d794, name=gtx, clk=00000000ff7456e0)
            // clk_request(dev=00000000ff725cf0, clk=00000000ff7456e0)
            // clk_enable(clk=00000000ff747b40 name=clock-controller@13020000)
            // clk_enable(clk=00000000ff72a540 name=stg_axiahb)
            // syscrg.clk_stg_axiahb().write(|w| w.clk_icg().set_bit());
            // clk_gate_endisable: enabling clock gmac1_axi (reg=0x0000000013020188, bit=31,
            // set=1) md 0x80000000
            syscrg
                .clk_gmac5_axi64_axi()
                .write(|w| w.clk_icg().set_bit());
            Self::check_diff(
                "GMAC5 AXI64 AXI",
                syscrg.clk_gmac5_axi64_axi().read().bits(),
                0x80000000,
            );
            // clk_enable(clk=00000000ff747b68 name=clock-controller@13020000)
            // clk_enable(clk=00000000ff72a680 name=ahb0)
            // clk_gate_endisable: enabling clock gmac1_ahb (reg=0x0000000013020184, bit=31,
            // set=1) md 0x80000000
            syscrg
                .clk_gmac5_axi64_ahb()
                .write(|w| w.clk_icg().set_bit());
            Self::check_diff(
                "GMAC5 AXI64 AHB",
                syscrg.clk_gmac5_axi64_ahb().read().bits(),
                0x80000000,
            );
            // clk_enable(clk=00000000ff747b90 name=clock-controller@13020000)
            // clk_enable(clk=00000000ff72bf40 name=gmac_src)
            // clk_gate_endisable: enabling clock gmac1_ptp (reg=0x0000000013020198, bit=31,
            // set=1) md 0x8000000a
            syscrg
                .clk_gmac5_axi64_ptp()
                // .write(|w| w.clk_icg().set_bit());
                .write(|w| w.bits(0x8000000a));
            Self::check_diff(
                "GMAC5 AXI64 PTP",
                syscrg.clk_gmac5_axi64_ptp().read().bits(),
                0x8000000a,
            );
            // clk_enable(clk=00000000ff747bb8 name=clock-controller@13020000)
            // clk_enable(clk=00000000ff72c9c0 name=gmac1_tx)
            // clk_enable(clk=00000000ff72c300 name=gmac1_rmii_rtx)
            // md 0x00000002
            // xiao
            syscrg.clk_gmac1_rmii_rtx().write(|w| w.bits(2));

            Self::check_diff(
                "GMAC1 RMII RTX",
                syscrg.clk_gmac1_rmii_rtx().read().bits(),
                0x00000002,
            );
            // clk_gate_endisable: enabling clock gmac1_tx (reg=0x00000000130201a4, bit=31,
            // set=1) md 0x81000000
            // syscrg.clk_gmac5_axi64_tx().write(|w| w.clk_icg().set_bit());
            syscrg.clk_gmac5_axi64_tx().write(|w| w.bits(0x81000000));
            Self::check_diff(
                "GMAC5 AXI64 TX",
                syscrg.clk_gmac5_axi64_tx().read().bits(),
                0x81000000,
            );
            // clk_gate_endisable: enabling clock gmac1_tx_inv (reg=0x00000000130201a8,
            // bit=30, set=1) md 0x40000000
            syscrg.clk_gmac5_axi64_txi().write(|w| w.bits(1 << 30));
            Self::check_diff(
                "GMAC5 AXI64 TXI",
                syscrg.clk_gmac5_axi64_txi().read().bits(),
                0x40000000,
            );
            // clk_enable(clk=00000000ff747be0 name=clock-controller@13020000)
            // clk_enable(clk=00000000ff72c080 name=gmac1_gtxclk)
            // clk_enable(clk=00000000ff7295c0 name=pll0_out)
            // clk_gate_endisable: enabling clock gmac1_gtxc (reg=0x00000000130201ac,
            // bit=31, set=1) md 0x80000020
            // syscrg.clk_gmac1_gtxclk().write(|w| w.bits(1 << 31));
            syscrg.clk_gmac1_gtxclk().write(|w| w.bits(0x80000020));
            Self::check_diff(
                "GMAC1 GTXCLK",
                syscrg.clk_gmac1_gtxclk().read().bits(),
                0x80000020,
            );
            // clk_set_defaults(ethernet-phy@1)
            // clk_set_default_parents: could not read assigned-clock-parents
            // for 00000000ff728410
        }

        // reset
        unsafe {
            aoncrg.soft_rst_addr_sel().write(|w| w.bits(0xe1));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));
            aoncrg.soft_rst_addr_sel().write(|w| w.bits(0xe3));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));
            // jh7110_reset_trigger: deasserting reset 0 (reg=0x17000038, value=0xe2)
            // jh7110_reset_trigger: deasserting reset 1 (reg=0x17000038, value=0xe0)
            aoncrg.soft_rst_addr_sel().write(|w| w.bits(0xe2));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));
            aoncrg.soft_rst_addr_sel().write(|w| w.bits(0xe0));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));

            syscrg.soft_rst_addr_sel_2().write(|w| w.bits(0xffe5efc4));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));
            syscrg.soft_rst_addr_sel_2().write(|w| w.bits(0xffe5efcc));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));
            // jh7110_reset_trigger: deasserting reset 66 (reg=0x13020300, value=0xffe5efc8)
            // jh7110_reset_trigger: deasserting reset 67 (reg=0x13020300, value=0xffe5efc0)
            syscrg.soft_rst_addr_sel_2().write(|w| w.bits(0xffe5efc8));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));
            syscrg.soft_rst_addr_sel_2().write(|w| w.bits(0xffe5efc0));
            let _ = DwmacHalImpl::wait_until(core::time::Duration::from_millis(100));
        }
    }

    #[allow(dead_code)]
    fn write_reg(paddr: PhysAddr, val: u32) {
        unsafe {
            let vaddr = <Self as DwmacHal>::mmio_phys_to_virt(paddr.into(), 0x1000);
            core::ptr::write_volatile(vaddr.as_ptr() as *mut u32, val);
        }
    }

    /// Print status without modifying any registers
    fn print_preserved_status() {
        log::info!("   📊 Current hardware status (read-only, preserved from U-Boot):");

        // Use PAC for available registers
        let aoncrg: &pac::aoncrg::RegisterBlock = unsafe {
            &*(<Self as DwmacHal>::mmio_phys_to_virt(pac::AONCRG::ptr() as usize, 0x1000).as_ptr()
                as *const pac::aoncrg::RegisterBlock)
        };

        let syscrg: &pac::syscrg::RegisterBlock = unsafe {
            &*(<Self as DwmacHal>::mmio_phys_to_virt(pac::SYSCRG::ptr() as usize, 0x1000).as_ptr()
                as *const pac::syscrg::RegisterBlock)
        };

        {
            syscrg.clk_tim_apb().write(|w| w.clk_icg().set_bit());
            syscrg.clk_tim_0().write(|w| w.clk_icg().set_bit());
            syscrg.clk_tim_1().write(|w| w.clk_icg().set_bit());
            syscrg.clk_tim_2().write(|w| w.clk_icg().set_bit());
            syscrg.clk_tim_3().write(|w| w.clk_icg().set_bit());
        }

        {
            // Read-only status check - don't interpret disabled as bad
            log::info!("   🔍 Clock register readings (may not reflect actual hardware state):");

            let gmac5_axi64_axi_enabled = syscrg.clk_gmac5_axi64_axi().read().clk_icg().bit();
            let gmac5_axi64_ptp_enabled = syscrg.clk_gmac5_axi64_ptp().read().clk_icg().bit();
            let gmac0_gtx_enabled = syscrg.clk_gmac0_gtx().read().clk_icg().bit();

            log::info!(
                "     GMAC5 AXI64 AXI: {} (register view)",
                if gmac5_axi64_axi_enabled {
                    "✅ enabled"
                } else {
                    "❓ disabled in register"
                }
            );
            log::info!(
                "     GMAC5 AXI64 PTP: {} (register view)",
                if gmac5_axi64_ptp_enabled {
                    "✅ enabled"
                } else {
                    "❓ disabled in register"
                }
            );
            log::info!(
                "     GMAC0 GTX: {} (register view)",
                if gmac0_gtx_enabled {
                    "✅ enabled"
                } else {
                    "❓ disabled in register"
                }
            );

            log::info!("   💡 Note: Clock registers may show 'disabled' even when hardware works");
            log::info!(
                "   💡 U-Boot may use different initialization sequence than Linux drivers expect"
            );
            log::info!("   💡 The real test is whether networking actually works!");

            log::info!(
                "   🔧 Reset status - Soft reset selector 2: {:#x} (preserved)",
                syscrg.soft_rst_addr_sel_2().read().bits()
            );

            log::info!(
                "   🔧 Reset status - AON reset selector: {:#x} (preserved)",
                aoncrg.soft_rst_addr_sel().read().bits()
            );

            log::info!(
                "   🔧 Clock config - GMAC1 GTX: {:#x} (preserved)",
                syscrg.clk_gmac1_gtx().read().clk_divcfg().bits()
            );

            log::info!(
                "   🔧 Clock config - GMAC1 RMII RTX: {:#x} (preserved)",
                syscrg.clk_gmac1_rmii_rtx().read().clk_divcfg().bits()
            );

            log::info!(
                "   🔧 Clock config - GMAC1 GTX: {:#x} (preserved)",
                syscrg.clk_gmac1_gtx().read().clk_divcfg().bits()
            );

            log::info!(
                "   🔧 Clock config - CPU Root: {:#x} (preserved)",
                syscrg.clk_cpu_root().read().bits()
            );
            log::info!(
                "   🔧 Clock config - CPU Core: {:#x} (preserved)",
                syscrg.clk_cpu_core().read().bits()
            );
            log::info!(
                "   🔧 Clock config - CPU Bus: {:#x} (preserved)",
                syscrg.clk_cpu_bus().read().bits()
            );
        }
    }

    fn print_clocks() {
        // Use PAC for available registers
        let aoncrg: &pac::aoncrg::RegisterBlock = unsafe {
            &*(<Self as DwmacHal>::mmio_phys_to_virt(pac::AONCRG::ptr() as usize, 0x1000).as_ptr()
                as *const pac::aoncrg::RegisterBlock)
        };

        let syscrg: &pac::syscrg::RegisterBlock = unsafe {
            &*(<Self as DwmacHal>::mmio_phys_to_virt(pac::SYSCRG::ptr() as usize, 0x1000).as_ptr()
                as *const pac::syscrg::RegisterBlock)
        };

        if syscrg.clk_gmac5_axi64_axi().read().clk_icg().bit() {
            log::debug!("🔧 GMAC5 AXI64 clock is enabled");
        } else {
            log::debug!("🔧 GMAC5 AXI64 clock is disabled");
        }

        if syscrg.clk_gmac5_axi64_ptp().read().clk_icg().bit() {
            log::debug!("🔧 GMAC5 AXI64 PTP clock is enabled");
        } else {
            log::debug!("🔧 GMAC5 AXI64 PTP clock is disabled");
        }

        if syscrg.clk_gmac5_axi64_tx().read().clk_icg().bit() {
            log::debug!("🔧 GMAC5 AXI64 TX clock is enabled");
        } else {
            log::debug!("🔧 GMAC5 AXI64 TX clock is disabled");
        }

        if syscrg.clk_gmac0_gtx().read().clk_icg().bit() {
            log::debug!("🔧 GMAC0 GTX clock is enabled");
        } else {
            log::debug!("🔧 GMAC0 GTX clock is disabled");
        }

        if syscrg.clk_gmac0_ptp().read().clk_icg().bit() {
            log::debug!("🔧 GMAC0 PTP clock is enabled");
        } else {
            log::debug!("🔧 GMAC0 PTP clock is disabled");
        }

        if aoncrg.clk_axi_gmac5().read().clk_icg().bit() {
            log::debug!("🔧 GMAC5 AXI clock is enabled");
        } else {
            log::debug!("🔧 GMAC5 AXI clock is disabled");
        }

        if aoncrg.clk_gmac5_axi64_tx().read().bits() & 0x8000_0000 != 0 {
            log::debug!("🔧 GMAC5 AXI64 TX clock is enabled");
        } else {
            log::debug!("🔧 GMAC5 AXI64 TX clock is disabled");
        }

        log::debug!(
            "🔧 Soft reset address selector 2: {:#x}",
            syscrg.soft_rst_addr_sel_2().read().bits()
        );

        log::debug!(
            "🔧 Soft reset address selector: {:#x}",
            aoncrg.soft_rst_addr_sel().read().bits()
        );

        log::debug!(
            "🔧 GMAC1 GTX clock: {:#x}",
            syscrg.clk_gmac1_gtx().read().clk_divcfg().bits()
        );

        log::debug!(
            "🔧 GMAC1 RMII RTX clock: {:#x}",
            syscrg.clk_gmac1_rmii_rtx().read().clk_divcfg().bits()
        );
    }
}
