mod boot;

pub mod generic_timer;
pub mod psci;

#[cfg(feature = "irq")]
pub mod gic;

#[cfg(any(
    platform_family = "aarch64-qemu-virt",
    platform_family = "aarch64-raspi4"
))]
pub mod pl011;
