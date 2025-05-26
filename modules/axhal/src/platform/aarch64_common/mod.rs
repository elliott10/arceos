#[cfg(not(feature = "hv"))]
mod boot;
#[cfg(feature = "hv")]
// Todo: maybe we can enter el2 in arm_vcpu?
mod boot_el2;

pub mod generic_timer;
#[cfg(not(platform_family = "aarch64-raspi"))]
pub mod psci;

#[cfg(all(feature = "irq", feature = "gicv3"))]
pub mod gicv3;
#[cfg(all(feature = "irq", feature = "gicv3"))]
pub use gicv3 as gic;

#[cfg(all(feature = "irq", not(feature = "gicv3")))]
pub mod gic;

#[cfg(not(any(
    platform_family = "aarch64-bsta1000b",
    platform_family = "aarch64-rk3588j"
)))]
pub mod pl011;
