#![cfg_attr(feature = "axstd", no_std)]
#![cfg_attr(feature = "axstd", no_main)]

#[macro_use]
#[cfg(feature = "axstd")]
extern crate axstd as std;

#[cfg(target_arch = "aarch64")]
mod cycle;

#[cfg(not(feature = "qemu"))]
mod gpio;

#[macro_use]
#[path = "bencher.rs"]
mod bencher;
use bencher::*;

use std::thread::{self, sleep};
use std::time::Duration;

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

#[allow(dead_code)]
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

    Bencher::new("condvar").reset(iter, end - start, 0).show();
}

#[allow(dead_code)]
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
        for _i in 0..iter / 2 {
            //println!("1 THREAD, switch {}", i);

            // GPIO Output： 低电平
            #[cfg(not(feature = "qemu"))]
            {
                gpio::gpio3_output_low();
            }

            thread::yield_now();
        }
    });

    let mut bencher_switch = Bencher::new("switch");
    let mut sum_cpu_cycle = 0;
    let mut sum_tsc = 0;

    #[cfg(not(feature = "qemu"))]
    {
        println!("2 THREADS switching GPIO3_C6 output between low and high");
        gpio::gpio3_output_low();
        gpio::gpio3_output_high();
        gpio::gpio3_output_low();
        gpio::gpio3_output_high();
    }
    //println!("Start the task thread switching test ...");

    for _i in 0..iter / 2 {
        //println!("0 THREAD, switch {}", i);

        let tsc_start = now_tsc();
        let cpu_cycle_start = cycle::cpu_cycle();

        // 首先运行；当前任务主动放弃CPU使用，主动切换到另一个就绪的任务
        thread::yield_now();

        let cpu_cycle_end = cycle::cpu_cycle();
        let tsc_end = now_tsc();

        // GPIO Output 高电平
        #[cfg(not(feature = "qemu"))]
        {
            gpio::gpio3_output_high();
        }

        let cpu_cycle = cpu_cycle_end - cpu_cycle_start;
        let tsc = tsc_end - tsc_start;
        sum_cpu_cycle += cpu_cycle;
        sum_tsc += tsc;
        bencher_switch.set_a_cpu_cycle(cpu_cycle / 2);
        bencher_switch.set_max_tsc(tsc / 2);
    }

    bencher_switch.reset(iter, sum_tsc, sum_cpu_cycle).show();
}

#[cfg_attr(feature = "axstd", no_mangle)]
fn main() {
    #[cfg(not(feature = "qemu"))]
    {
        gpio::init_uart7();
        gpio::gpio3_led_red_on();
    }

    println!("Bencher start ...\n");

    // User access PMU
    cycle::enable_cpu_cycle();

    cycle::reset_pmu_all();
    cycle::enable_pmu_all();
    cycle::isb();

    let timer_start = cycle::timer_cnt();
    let cpu_cycle_start = cycle::cpu_cycle();

    Bencher::new("rdtsc")
        .bench_many(|| now_tsc(), 10000, 100_000_000)
        .show();

    bench_spawn();

    //bench_condvar();

    let cpu_cycle_end = cycle::cpu_cycle();
    let timer_end = cycle::timer_cnt();

    let cpu_cycle = cpu_cycle_end - cpu_cycle_start;
    let timer_sum = timer_end - timer_start;

    let timer_freq = cycle::timer_freq();
    let s_sum = timer_sum / timer_freq;
    let ns_sum = timer_sum * (1_000_000_000 / timer_freq);

    let cpu_freq = cycle::cpu_freq(cpu_cycle, timer_sum);
    CPUFRQ_HZ.store(cpu_freq, core::sync::atomic::Ordering::Relaxed);

    println!(
        "\nCPU Freq = {}Hz, CPU Cycle Counter = {} from {} to {}, In {}s, {}ns",
        cpu_freq, cpu_cycle, cpu_cycle_start, cpu_cycle_end, s_sum, ns_sum
    );

    /********** 评测task调度切换开销 **********/
    println!("\nBencher: task switch ...");
    #[cfg(target_arch = "aarch64")]
    println!(
        "AARCH64 Generic Timer Registers: CNTFRQ_EL0={}, CNTVCT_EL0={}",
        timer_freq,
        now_tsc()
    );

    // 评测切换的次数

    cycle::isb();
    println!(
        "CPU{} cycle={}, timer cnt={}",
        cycle::get_cpu_id(),
        cycle::cpu_cycle(),
        cycle::timer_cnt()
    );
    println!(
        "PMUSERENR_EL0={:#x}, PMCNTENSET_EL0={:#x}, PMCR_EL0={:#x}",
        cycle::armv8_pmuserenr(),
        cycle::armv8_pmcntenset(),
        cycle::armv8_pmcr()
    );

    #[cfg(not(feature = "qemu"))]
    {
        gpio::gpio3_clock_gate_enable();
        gpio::iomux_gpio3_c6_gpio();

        gpio::gpio3_clear_all();
        gpio::gpio3_led_green_on();

        gpio::gpio_ver_id_get(gpio::GPIO3_BASE);
        gpio::gpio_ext_port_signals_get(gpio::GPIO3_BASE);
    }

    // 每1亿次切换将输出一次GPIO UART信号
    println!("After every 100 million task switches, a GPIO UART signal will be output");
    let switch_count = 100_000_000;
    let iter = 100;

    for i in 0..iter {
        println!(
            "\n---------\nBencher: {} task switch count = {}",
            i, switch_count
        );

        /*
        #[cfg(not(feature = "qemu"))]
        {
            gpio::uart7_put_hi();
        } */

        bench_switch(switch_count);
    }

    println!("\nBencher end");
}
