mod boot;

pub mod generic_timer;
pub mod psci;

#[cfg(feature = "irq")]
pub mod gic;

#[cfg(any(
    feature = "platform-qemu-virt-aarch64",
    feature = "platform-raspi4-aarch64"
))]
pub mod pl011;
