#![allow(dead_code)]
//! ARM Power State Coordination Interface.

/* PSCI v0.2 interface */
pub const PSCI_0_2_FN_BASE: u32 = 0x84000000;
pub const PSCI_0_2_64BIT: u32 = 0x40000000;
pub const PSCI_0_2_FN_PSCI_VERSION: u32 = PSCI_0_2_FN_BASE + 0;
pub const PSCI_0_2_FN_CPU_SUSPEND: u32 = PSCI_0_2_FN_BASE + 1;
pub const PSCI_0_2_FN_CPU_OFF: u32 = PSCI_0_2_FN_BASE + 2;
pub const PSCI_0_2_FN_CPU_ON: u32 = PSCI_0_2_FN_BASE + 3;
pub const PSCI_0_2_FN_MIGRATE: u32 = PSCI_0_2_FN_BASE + 5;
pub const PSCI_0_2_FN_SYSTEM_OFF: u32 = PSCI_0_2_FN_BASE + 8;
pub const PSCI_0_2_FN_SYSTEM_RESET: u32 = PSCI_0_2_FN_BASE + 9;
pub const PSCI_0_2_FN64_CPU_SUSPEND: u32 = PSCI_0_2_FN_BASE + PSCI_0_2_64BIT + 1;
pub const PSCI_0_2_FN64_CPU_ON: u32 = PSCI_0_2_FN_BASE + PSCI_0_2_64BIT + 3;
pub const PSCI_0_2_FN64_MIGRATE: u32 = PSCI_0_2_FN_BASE + PSCI_0_2_64BIT + 5;

/// arm,psci method: smc
/// when SMCCC_CONDUIT_SMC = 1
fn arm_smccc_smc(fn_id: usize, arg0: usize, arg1: usize, arg2: usize) -> usize {
    let mut ret;
    unsafe {
        core::arch::asm!(
            "smc #0",
            inlateout("x0") fn_id => ret,
            in("x1") arg0,
            in("x2") arg1,
            in("x3") arg2,
        )
    }
    ret
}

/// psci "hvc" method call
fn psci_hvc_call(func: usize, arg0: usize, arg1: usize, arg2: usize) -> usize {
    let ret;
    unsafe {
        core::arch::asm!(
            "hvc #0",
            inlateout("x0") func as usize => ret,
            in("x1") arg0,
            in("x2") arg1,
            in("x3") arg2,
        )
    }
    ret
}

fn psci_call(fn_id: usize, arg0: usize, arg1: usize, arg2: usize) -> Result<i32, PsciRetNo> {
    let ret = match axconfig::PSCI_METHOD {
        "smc" => arm_smccc_smc(fn_id, arg0, arg1, arg2),
        "hvc" | _ => psci_hvc_call(fn_id, arg0, arg1, arg2),
    };

    let retno = psci_errno_tran(ret as i32);
    if retno == PsciRetNo::PsciRetSuccess {
        Ok(0)
    } else {
        Err(retno)
    }
}

/// Shutdown the whole system, including all CPUs.
pub fn system_off() -> ! {
    info!("Shutting down...");
    let _ = psci_call(PSCI_0_2_FN_SYSTEM_OFF as usize, 0, 0, 0);
    warn!("It should shutdown!");
    loop {
        crate::arch::halt();
    }
}

/// Starts a secondary CPU with the given ID.
/// When the CPU is started, it will jump to the given entry and set the
/// corresponding register to the given argument.
///
/// PSCI v0.1
/// cpu entry_point: phys_addr(secondary_entry)
pub fn psci_cpu_on(cpuid: usize, entry_point: usize, arg: usize) -> i32 {
    info!("Starting CPU {} ON ...", cpuid);
    let err = psci_call(PSCI_0_2_FN64_CPU_ON as usize, cpuid, entry_point, arg);
    match err {
        Err(e) => {
            error!("failed to boot CPU {} ({:?})", cpuid, e);
            e as i32
        }
        Ok(_) => {
            info!("Started CPU {}", cpuid);
            0
        }
    }
}

pub fn psci_cpu_off(_cpuid: usize) {
    const PSCI_POWER_STATE_TYPE_STANDBY: u32 = 0;
    const PSCI_POWER_STATE_TYPE_POWER_DOWN: u32 = 1;
    const PSCI_0_2_POWER_STATE_TYPE_SHIFT: u32 = 16;
    let state: u32 = PSCI_POWER_STATE_TYPE_POWER_DOWN << PSCI_0_2_POWER_STATE_TYPE_SHIFT;
    let _ = psci_call(PSCI_0_2_FN_CPU_OFF as usize, state as usize, 0, 0);
}

pub fn psci_errno_tran(errno: i32) -> PsciRetNo {
    //EINVAL
    let mut ret = -22;
    let errno_tran = match errno {
        0 => {
            ret = 0;
            PsciRetNo::PsciRetSuccess
        }
        // EOPNOTSUPP
        -1 => {
            ret = -95;
            PsciRetNo::PsciRetNotSupported
        }
        -2 => {
            ret = -22;
            PsciRetNo::PsciRetInvalidParams
        }
        // EPERM
        -3 => {
            ret = -1;
            PsciRetNo::PsciRetDenied
        }
        -4 => PsciRetNo::PsciRetAlreadyOn,
        -5 => PsciRetNo::PsciRetOnPending,
        -6 => PsciRetNo::PsciRetInternalFailure,
        -7 => PsciRetNo::PsciRetNotPresent,
        -8 => PsciRetNo::PsciRetDisabled,
        -9 => {
            ret = -22;
            PsciRetNo::PsciRetInvalidAddress
        }
        _ => {
            warn!("Unknown psci errno: {}", errno);
            PsciRetNo::PsciRetInvalidParams
        }
    };
    info!("return={}, {:?}", ret, errno_tran);
    errno_tran
}

/// PSCI return values, inclusive of all PSCI versions
#[derive(PartialEq, Debug)]
pub enum PsciRetNo {
    PsciRetSuccess = 0,
    PsciRetNotSupported = -1,
    PsciRetInvalidParams = -2,
    PsciRetDenied = -3,
    PsciRetAlreadyOn = -4,
    PsciRetOnPending = -5,
    PsciRetInternalFailure = -6,
    PsciRetNotPresent = -7,
    PsciRetDisabled = -8,
    PsciRetInvalidAddress = -9,
}
