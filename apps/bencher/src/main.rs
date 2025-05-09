#![cfg_attr(feature = "axstd", no_std)]
#![cfg_attr(feature = "axstd", no_main)]

#[macro_use]
#[cfg(feature = "axstd")]
extern crate axstd as std;

#[cfg(target_arch = "aarch64")]
mod cycle;

#[macro_use]
#[path = "bencher.rs"]
mod bencher;
use bencher::*;

use std::thread;

fn bench_spawn() {
    let warmup = 0;
    let iter = if cfg!(feature = "axstd") {
        500_000
    } else {
        200_000
    };

    let mut b = Bencher::new("spawn");
    for _ in 0..warmup {
        let t = thread::spawn(|| {});
        t.join().unwrap();
    }

    for _ in 0..iter {
        b.bench_once(|| thread::spawn(|| {})).join().unwrap();
    }
    b.show();

    // b.reset(0, 0);
    // for _ in 0..iter {
    //     b.bench_once(|| thread::spawn(|| {}).join()).unwrap();
    // }
    // b.show();
}

#[cfg(feature = "axstd")]
fn bench_condvar() {
    use std::os::arceos::api::task;
    use std::sync::{Arc, Mutex};

    let iter = 5_000_000;
    let pair = Arc::new((Mutex::new(false), task::AxWaitQueueHandle::new()));
    let pair2 = Arc::clone(&pair);

    thread::spawn(move || {
        let (lock, wq) = &*pair2;
        for _ in 0..iter / 2 {
            while *lock.lock() {
                task::ax_wait_queue_wait(&wq, || true, None);
            }
            *lock.lock() = true;
            task::ax_wait_queue_wake(&wq, 1);
        }
    });

    let (lock, wq) = &*pair;
    let start = now_tsc();
    for _ in 0..iter / 2 {
        while !*lock.lock() {
            task::ax_wait_queue_wait(&wq, || true, None);
        }
        *lock.lock() = false;
        task::ax_wait_queue_wake(&wq, 1);
    }
    let end = now_tsc();

    Bencher::new("condvar").reset(iter, end - start).show();
}

#[cfg(not(feature = "axstd"))]
fn bench_condvar() {
    use std::sync::{Arc, Condvar, Mutex};

    let iter = 5_000_000;
    let pair = Arc::new((Mutex::new(false), Condvar::new()));
    let pair2 = Arc::clone(&pair);

    thread::spawn(move || {
        let (lock, wq) = &*pair2;
        for _ in 0..iter / 2 {
            let mut var = lock.lock().unwrap();
            while *var {
                var = wq.wait(var).unwrap();
            }
            *var = true;
            wq.notify_one();
        }
    });

    let (lock, wq) = &*pair;
    let start = now_tsc();
    for _ in 0..iter / 2 {
        let mut var = lock.lock().unwrap();
        while !*var {
            var = wq.wait(var).unwrap();
        }
        *var = false;
        wq.notify_one();
    }
    let end = now_tsc();

    Bencher::new("condvar").reset(iter, end - start).show();
}

/// 创建两个线程，每次yield都会切到另一线程;
/// 每个线程分别yield (iter/2)次;
/// 单次yield的时间 = 总时间/iter
fn bench_switch(iter: u64) {
    thread::spawn(move || {
        for _ in 0..iter / 2 {
            thread::yield_now();
        }
    });

    let start = now_tsc();
    let cpu_cycle_start = cycle::cpu_cycle();

    for _ in 0..iter / 2 {
        // 当前任务主动放弃CPU使用，主动切换到另一个就绪的任务
        thread::yield_now();
    }

    let cpu_cycle_end = cycle::cpu_cycle();
    let end = now_tsc();

    let cpu_cycle = cpu_cycle_end - cpu_cycle_start;
    let timer_sum = end - start;
    let cpu_freq = cycle::cpu_freq(cpu_cycle, timer_sum);

    CPUFRQ_HZ.store(cpu_freq, core::sync::atomic::Ordering::Relaxed);

    println!("CPU Freq = {}Hz now.", cpu_freq);

    Bencher::new("switch").reset(iter, end - start).show();
}

#[cfg_attr(feature = "axstd", no_mangle)]
fn main() {
    println!("Bencher init UART7 ...\n");

    /*
    let bus_ioc: usize = 0xffff_0000_fd5f8000;
    dw_apb_uart::DW8250::iomux_uart7_m2(bus_ioc);
    */

    // rk3588 UART7
    let uart_base: usize = 0xffff_0000_feba0000;
    let mut uart = dw_apb_uart::DW8250::new(uart_base);
    uart.minit();
    {
        uart.putchar(b'\n');
        uart.putchar(b'\r');
        uart.putchar(b'B');
        uart.putchar(b'e');
        uart.putchar(b'n');
        uart.putchar(b'c');
        uart.putchar(b'h');
        uart.putchar(b'e');
        uart.putchar(b'r');
        uart.putchar(b'\n');
        uart.putchar(b'\r');
    }

    println!("Bencher start ...\n");

    Bencher::new("rdtsc")
        .bench_many(|| now_tsc(), 10000, 100_000_000)
        .show();

    bench_spawn();

    //bench_condvar();

    ///////// 评测task调度切换开销
    println!("\nBencher: task switch ...");
    #[cfg(target_arch = "aarch64")]
    println!("AARCH64 Generic Timer Registers: CNTFRQ_EL0={}, CNTVCT_EL0={}", timer_freq(), now_tsc());

    // 评测切换的次数

    cycle::isb();
    println!("CPU{} cycle={}, timer cnt={}", cycle::get_cpu_id(), cycle::cpu_cycle(), cycle::timer_cnt());
    println!("PMUSERENR_EL0={:#x}, PMCNTENSET_EL0={:#x}, PMCR_EL0={:#x}", cycle::armv8_pmuserenr(), cycle::armv8_pmcntenset(), cycle::armv8_pmcr());
    cycle::enable_cpu_cycle();

    // 每1亿次切换将输出一次GPIO UART信号
    println!("After every 100 million task switches, a GPIO UART signal will be output");
    let switch_count = 100_000_000;

    // User access PMU
    cycle::reset_pmu_all();
    cycle::enable_pmu_all();
    let mut cpu_cycle = 0;

    let iter = 100;
    for i in 0..iter {
        println!("\n---------\nBencher: {} task switch count = {}\n", i, switch_count);

        {
            uart.putchar(b'H');
            uart.putchar(b'i');
            uart.putchar(b'\n');
            uart.putchar(b'\r');
        }

        let timer_start = cycle::timer_cnt();
        let cpu_cycle_start = cycle::cpu_cycle();

        bench_switch(switch_count);

        let cpu_cycle_end = cycle::cpu_cycle();
        let timer_end = cycle::timer_cnt();

        cpu_cycle = cpu_cycle_end - cpu_cycle_start;
        let timer_sum = timer_end - timer_start;

        let timer_freq = cycle::timer_freq();
        let s_sum = timer_sum / timer_freq;
        let ns_sum = timer_sum * (1_000_000_000 / timer_freq);

        let cpu_freq = cycle::cpu_freq(cpu_cycle, timer_sum);
        println!("\nIn {}s, {}ns, Average cost CPU cycles: {}", s_sum, ns_sum, cpu_cycle/switch_count);
        println!("CPU Freq = {}Hz, CPU Cycle Counter = {} from {} to {}", cpu_freq, cpu_cycle, cpu_cycle_start, cpu_cycle_end);
    }

    println!("\nBencher end");
}
