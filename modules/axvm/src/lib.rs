//! [ArceOS](https://github.com/rcore-os/arceos) virtial machine monitor management module.
//!
//! This module provides primitives for VM management, including VM
//! creation, two-stage memory management, device emulation, etc.
//!
//! Currently `axvm` actes like a wrapper for [hypercraft](https://github.com/arceos-hypervisor/hypercraft.git).
//!
//! !!! This module is WORK-IN-PROCESS !!!

#![cfg_attr(not(test), no_std)]
#![feature(doc_cfg)]
#![feature(doc_auto_cfg)]

extern crate alloc;

#[macro_use]
extern crate log;

mod config;
#[cfg(target_arch = "x86_64")]
mod device;
mod mm;

mod arch;

mod page_table;
mod irq;
mod hvc;

// mod vm;
// pub use vm::*;

pub use arch::{PerCpu, VCpu};


/// To be removed.
mod linux;
pub use linux::config_boot_linux;

pub use axhal::mem::{phys_to_virt, virt_to_phys, PhysAddr};
pub use page_table::GuestPageTable;

#[cfg(feature = "type1_5")]
pub use hypercraft::LinuxContext;
pub use hypercraft::GuestPageTableTrait;
pub use hypercraft::HyperCraftHal;
pub use hypercraft::HyperError as Error;
pub use hypercraft::HyperResult as Result;
pub use hypercraft::{
    GuestPhysAddr, GuestVirtAddr, HostPhysAddr, HostVirtAddr, HyperCallMsg, VmExitInfo,
};
pub use hypercraft::{PerCpuDevices, PerVmDevices, VmxExitReason};