#![allow(dead_code)]

//! ARM Power State Coordination Interface.

use core::arch::asm;

fn psci_hvc_call(func: usize, arg0: usize, arg1: usize, arg2: usize) -> usize {
    let ret;
    unsafe {
        asm!(
            "hvc #0",
            inlateout("x0") func as usize => ret,
            in("x1") arg0,
            in("x2") arg1,
            in("x3") arg2,
        )
    }
    ret
}

/// Shutdown the whole system, including all CPUs.
pub fn system_off() -> ! {
    info!("Shutting down...");
    psci_hvc_call(axconfig::PSCI_CPU_OFF, 0, 0, 0);
    warn!("It should shutdown!");
    loop {
        crate::arch::halt();
    }
}

/// Starts a secondary CPU with the given ID.
///
/// When the CPU is started, it will jump to the given entry and set the
/// corresponding register to the given argument.
pub fn cpu_on(id: usize, entry: usize, arg: usize) {
    debug!("Starting core {}...", id);
    assert_eq!(psci_hvc_call(axconfig::PSCI_CPU_ON, id, entry, arg), 0);
    debug!("Started core {}!", id);
}

/// arm,psci method: smc
fn arm_smccc_smc(fn_id: usize, arg0: usize, arg1: usize, arg2: usize) -> usize {
    let mut ret;
    unsafe {
        asm!(
            "smc #0",
            inlateout("x0") fn_id => ret,
            in("x1") arg0,
            in("x2") arg1,
            in("x3") arg2,
        )
    }
    ret
}

pub fn invoke_psci_fn(fn_id: usize, arg0: usize, arg1: usize, arg2: usize) -> usize {
    // SMCCC_CONDUIT_SMC = 1
    arm_smccc_smc(fn_id, arg0, arg1, arg2)
}

pub fn psci_0_1_cpu_on(fn_ids: u32, cpuid: usize, entry_point: usize, arg: usize) -> i32 {
    let err: i32 = invoke_psci_fn(fn_ids as usize, cpuid, entry_point, arg) as i32;
    psci_errno_tran(err)
}

pub fn psci_0_1_cpu_off(fn_ids: u32, state: u32) -> i32 {
    let err: i32 = invoke_psci_fn(fn_ids as usize, state as usize, 0, 0) as i32;
    psci_errno_tran(err)
}

pub fn psci_0_1_cpu_suspend(fn_ids: u32, state: u32, entry_point: usize) -> i32 {
    let err: i32 = invoke_psci_fn(fn_ids as usize, state as usize, entry_point, 0) as i32;
    psci_errno_tran(err)
}
pub fn psci_0_1_migrate(fn_ids: u32, cpuid: usize) -> i32 {
    let err: i32 = invoke_psci_fn(fn_ids as usize, cpuid, 0, 0) as i32;
    psci_errno_tran(err)
}

/// PSCI v0.1
/// cpu entry_point = phys_addr(secondary_entry)
/// eg: psci_0_1_cpu_on, fn_id: 0xc4000003, cpu on core 256, secondary_entry: 0x80cb51f8
pub fn psci_cpu_on(cpuid: usize, entry_point: usize, arg: usize) -> i32 {
    info!("PSCI CPU {} ON ...", cpuid);
    let err = psci_0_1_cpu_on(axconfig::PSCI_CPU_ON as u32, cpuid, entry_point, arg);
    if err != 0 {
        error!("failed to boot CPU {} ({})", cpuid, err);
    }
    err
}

pub fn psci_cpu_off(cpuid: usize, entry_point: usize) {
    const PSCI_POWER_STATE_TYPE_STANDBY: u32 = 0;
    const PSCI_POWER_STATE_TYPE_POWER_DOWN: u32 = 1;
    const PSCI_0_2_POWER_STATE_TYPE_SHIFT: u32 = 16;
    let state: u32 = PSCI_POWER_STATE_TYPE_POWER_DOWN << PSCI_0_2_POWER_STATE_TYPE_SHIFT;
    psci_0_1_cpu_off(axconfig::PSCI_CPU_OFF as u32, state);
}

pub fn psci_errno_tran(errno: i32) -> i32 {
    //EINVAL
    let mut ret = -22;
    let errno_tran = match errno {
        0 => {
            ret = 0;
            PsciRetNo::PSCI_RET_SUCCESS
        }
        // EOPNOTSUPP
        -1 => {
            ret = -95;
            PsciRetNo::PSCI_RET_NOT_SUPPORTED
        }
        -2 => {
            ret = -22;
            PsciRetNo::PSCI_RET_INVALID_PARAMS
        }
        // EPERM
        -3 => {
            ret = -1;
            PsciRetNo::PSCI_RET_DENIED
        }
        -4 => PsciRetNo::PSCI_RET_ALREADY_ON,
        -5 => PsciRetNo::PSCI_RET_ON_PENDING,
        -6 => PsciRetNo::PSCI_RET_INTERNAL_FAILURE,
        -7 => PsciRetNo::PSCI_RET_NOT_PRESENT,
        -8 => PsciRetNo::PSCI_RET_DISABLED,
        -9 => {
            ret = -22;
            PsciRetNo::PSCI_RET_INVALID_ADDRESS
        }
        _ => {
            warn!("Unknown psci errno: {}", errno);
            PsciRetNo::PSCI_RET_INVALID_PARAMS
        }
    };
    info!("PSCI return: {:?}", errno_tran);
    ret
}

/// PSCI return values, inclusive of all PSCI versions
#[derive(Debug)]
pub enum PsciRetNo {
    PSCI_RET_SUCCESS = 0,
    PSCI_RET_NOT_SUPPORTED = -1,
    PSCI_RET_INVALID_PARAMS = -2,
    PSCI_RET_DENIED = -3,
    PSCI_RET_ALREADY_ON = -4,
    PSCI_RET_ON_PENDING = -5,
    PSCI_RET_INTERNAL_FAILURE = -6,
    PSCI_RET_NOT_PRESENT = -7,
    PSCI_RET_DISABLED = -8,
    PSCI_RET_INVALID_ADDRESS = -9,
}
