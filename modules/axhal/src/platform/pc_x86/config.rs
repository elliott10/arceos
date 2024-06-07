use crate::mem::MemRegionFlags;
use bitflags::bitflags;
use core::fmt::Debug;
use core::{mem::size_of, slice};
use page_table_entry::MappingFlags;

const _CONFIG_SIGNATURE: [u8; 6] = *b"AOSSYS";
const _CONFIG_REVISION: u16 = 10;
pub const PER_CPU_ARRAY_PTR: *mut HvSystemConfig = __core_end as _;

const HV_CELL_NAME_MAXLEN: usize = 31;
const HV_MAX_IOMMU_UNITS: usize = 8;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct MemFlags(u64);

bitflags! {
    impl MemFlags: u64 {
        const READ          = 1 << 0;
        const WRITE         = 1 << 1;
        const EXECUTE       = 1 << 2;
        const DMA           = 1 << 3;
        const IO            = 1 << 4;
        const NO_HUGEPAGES  = 1 << 8;
        const USER          = 1 << 9;
    }
}

impl From<MemFlags> for MemRegionFlags {
    fn from(f: MemFlags) -> Self {
        let mut ret = MemRegionFlags::empty();
        if f.contains(MemFlags::READ) {
            ret |= Self::READ;
        }
        if f.contains(MemFlags::WRITE) {
            ret |= Self::WRITE;
        }
        if f.contains(MemFlags::EXECUTE) {
            ret |= Self::EXECUTE;
        }
        if f.contains(MemFlags::IO) {
            ret |= Self::DEVICE;
        }
        if f.contains(MemFlags::DMA) {
            ret |= Self::DMA;
        }
        if f.contains(MemFlags::USER) {
            ret |= Self::USER;
        }
        ret
    }
}

impl From<MemFlags> for MappingFlags {
    fn from(f: MemFlags) -> Self {
        let mut ret = MappingFlags::empty();
        if f.contains(MemFlags::READ) {
            ret |= Self::READ;
        }
        if f.contains(MemFlags::WRITE) {
            ret |= Self::WRITE;
        }
        if f.contains(MemFlags::EXECUTE) {
            ret |= Self::EXECUTE;
        }
        if f.contains(MemFlags::IO) {
            ret |= Self::DEVICE;
        }
        if f.contains(MemFlags::USER) {
            ret |= Self::USER;
        }
        ret
    }
}

extern "C" {
    fn __core_end();
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
struct HvConsole {
    address: u64,
    size: u32,
    console_type: u16,
    flags: u16,
    divider: u32,
    gate_nr: u32,
    clock_reg: u64,
}

/// The jailhouse cell configuration.
///
/// @note Keep Config._HEADER_FORMAT in jailhouse-cell-linux in sync with this
/// structure.
/// Sync with jailhouse/include/jailhouse/cell-config.h::jailhouse_cell_desc
#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvCellDesc {
    signature: [u8; 5],
    architecture: u8,
    revision: u16,

    name: [u8; HV_CELL_NAME_MAXLEN + 1],
    id: u32, // set by the driver
    flags: u32,

    pub cpu_set_size: u32,
    pub num_memory_regions: u32,
    pub num_cache_regions: u32,
    pub num_irqchips: u32,
    pub num_pio_regions: u32,
    pub num_pci_devices: u32,
    pub num_pci_caps: u32,
    pub num_stream_ids: u32,

    vpci_irq_base: u32,

    cpu_reset_address: u64,
    msg_reply_timeout: u64,

    console: HvConsole,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvMemoryRegion {
    pub phys_start: u64,
    pub virt_start: u64,
    pub size: u64,
    pub flags: MemFlags,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvCacheRegion {
    start: u32,
    size: u32,
    cache_type: u8,
    _padding: u8,
    flags: u16,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvIrqChip {
    address: u64,
    id: u32,
    pin_base: u32,
    pin_bitmap: [u32; 4],
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvPio {
    base: u16,
    length: u16,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvPciDevice {
    pci_device_type: u8,
    iommu: u8,
    domain: u16,
    bdf: u16,
    bar_mask: [u32; 6],
    caps_start: u16,
    num_caps: u16,
    num_msi_vectors: u8,
    msi_64bits: u8,
    msi_maskable: u8,
    num_msix_vectors: u16,
    msix_region_size: u16,
    msix_address: u64,
    /// First memory region index of shared memory device.
    shmem_regions_start: u32,
    /// ID of shared memory device (0..shmem_peers-1).
    shmem_dev_id: u8,
    /// Maximum number of peers connected via this shared memory device.
    shmem_peers: u8,
    /// PCI subclass and interface ID of shared memory device.
    shmem_protocol: u16,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvPciCapability {
    id: u16,
    start: u16,
    len: u16,
    flags: u16,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
struct HvIommu {
    iommu_type: u32,
    base: u64,
    size: u32,
    iommu_union: IommuTipvu, // union of IommuAmd and IommuTipvu
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
struct IommuAmd {
    bdf: u16,
    base_cap: u8,
    msi_cap: u8,
    features: u32,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
struct IommuTipvu {
    tlb_base: u64,
    tlb_size: u32,
}

#[cfg(target_arch = "x86_64")]
#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
struct ArchPlatformInfo {
    pm_timer_address: u16,
    apic_mode: u8,
    padding: u8,
    vtd_interrupt_limit: u32,
    tsc_khz: u32,
    apic_khz: u32,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
struct ArmPlatformInfo {
    maintenance_irq: u8,
    gic_version: u8,
    padding: [u8; 2],
    gicd_base: u64,
    gicc_base: u64,
    gich_base: u64,
    gicv_base: u64,
    gicr_base: u64,
}

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
struct PlatformInfo {
    pci_mmconfig_base: u64,
    pci_mmconfig_end_bus: u8,
    pci_is_virtual: u8,
    pci_domain: u16,
    iommu_units: [HvIommu; HV_MAX_IOMMU_UNITS],
    arch: ArmPlatformInfo, // union of ArchPlatformInfo and ArmPlatformInfo
}

/// General descriptor of the system.
#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct HvSystemConfig {
    pub signature: [u8; 5],
    pub architecture: u8,
    pub revision: u16,
    flags: u32,

    /// Jailhouse's location in memory
    pub hypervisor_memory: HvMemoryRegion,
    debug_console: HvConsole,
    platform_info: PlatformInfo,
    pub root_cell: HvCellDesc,
    // CellConfigLayout placed here.
}

pub struct CellConfig<'a> {
    desc: &'a HvCellDesc,
}

impl HvCellDesc {
    pub const fn config(&self) -> CellConfig {
        CellConfig::from(self)
    }

    pub const fn config_size(&self) -> usize {
        self.cpu_set_size as usize
            + self.num_memory_regions as usize * size_of::<HvMemoryRegion>()
            + self.num_cache_regions as usize * size_of::<HvCacheRegion>()
            + self.num_irqchips as usize * size_of::<HvIrqChip>()
            + self.num_pio_regions as usize * size_of::<HvPio>()
            + self.num_pci_devices as usize * size_of::<HvPciDevice>()
            + self.num_pci_caps as usize * size_of::<HvPciCapability>()
            + self.num_stream_ids as usize * size_of::<u32>()
    }
}

impl HvSystemConfig {
    pub fn get<'a>() -> &'a Self {
        unsafe { &*super::consts::hv_config_ptr() }
    }

    pub const fn size(&self) -> usize {
        size_of::<Self>() + self.root_cell.config_size()
    }
}

impl<'a> CellConfig<'a> {
    const fn from(desc: &'a HvCellDesc) -> Self {
        Self { desc }
    }

    fn config_ptr<T>(&self) -> *const T {
        unsafe { (self.desc as *const HvCellDesc).add(1) as _ }
    }

    pub const fn size(&self) -> usize {
        self.desc.config_size()
    }

    pub fn cpu_set(&self) -> &[u64] {
        // XXX: data may unaligned, which cause panic on debug mode. Same below.
        // See: https://doc.rust-lang.org/src/core/slice/mod.rs.html#6435-6443
        unsafe { slice::from_raw_parts(self.config_ptr(), self.desc.cpu_set_size as usize / 8) }
    }

    pub fn mem_regions(&self) -> &[HvMemoryRegion] {
        unsafe {
            let ptr = self.cpu_set().as_ptr_range().end as _;
            slice::from_raw_parts(ptr, self.desc.num_memory_regions as usize)
        }
    }
}

// impl Debug for CellConfig<'_> {
//     fn fmt(&self, f: &mut Formatter) -> Result {
//         let name = self.desc.name;
//         let mut len = 0;
//         while name[len] != 0 {
//             len += 1;
//         }
//         f.debug_struct("CellConfig")
//             .field("name", &core::str::from_utf8(&name[..len]))
//             .field("size", &self.size())
//             .field("mem_regions", &self.mem_regions())
//             .finish()
//     }
// }
