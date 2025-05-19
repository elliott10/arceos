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

    Bencher::new("condvar").reset(iter, end - start, 0).show();
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
        for i in 0..iter / 2 {
            //println!("1 THREAD, switch {}", i);
            thread::yield_now();
        }
    });

    let mut bencher_switch = Bencher::new("switch");
    let mut sum_cpu_cycle = 0;
    let mut sum_tsc = 0;

    for i in 0..iter / 2 {
        //println!("0 THREAD, switch {}", i);

        let tsc_start = now_tsc();
        let cpu_cycle_start = cycle::cpu_cycle();

        // 首先运行；当前任务主动放弃CPU使用，主动切换到另一个就绪的任务
        thread::yield_now();

        let cpu_cycle_end = cycle::cpu_cycle();
        let tsc_end = now_tsc();

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
        println!("Bencher init UART7 ...\n");

        let bus_ioc: usize = 0xffff_0000_fd5f8000;
        unsafe {
            // BUS_IOC_GPIO4D_IOMUX_SEL_L 0x0098 = 0x00005500
            let addr = bus_ioc + 0x0098;
            let iomux_gpio4d = core::ptr::read_volatile(addr as *mut u32);

            let addr = bus_ioc + 0x2c; // BUS_IOC_GPIO1B_IOMUX_SEL_H
            let iomux_gpio1b = core::ptr::read_volatile(addr as *mut u32);

            println!(
                "Read BUS_IOC, gpio4d default ={:#x}, UART7 iomux = {:#x}\n",
                iomux_gpio4d, iomux_gpio1b
            );

            // CRU clock
            let cru_base: usize = 0xffff_0000_fd7c0000;
            let clk_sel = core::ptr::read_volatile((cru_base + 0x03D4) as *mut u32); // CRU_CLKSEL_CON53 clk_uart7_src_sel = 0x6
            let clk_frac = core::ptr::read_volatile((cru_base + 0x03D8) as *mut u32); // CRU_CLKSEL_CON54 clk_uart7_frac = 0x1403de
            let sclk_sel = core::ptr::read_volatile((cru_base + 0x03DC) as *mut u32); // CRU_CLKSEL_CON55 sclk_uart7_sel = 0x6

            let pclk_en = core::ptr::read_volatile((cru_base + 0x0830) as *mut u32); // CRU_GATE_CON12 pclk_uart7_en bit8 = 0
            let sclk_en = core::ptr::read_volatile((cru_base + 0x0834) as *mut u32); // CRU_GATE_CON13 sclk_uart7_en bit15 = 0

            println!("Read CRU, clk_uart7_src_sel={:#x}, clk_uart7_frac={:#x}, sclk_uart7_sel={:#x} \n pclk_uart7_en={:#x}, sclk_uart7_en={:#x},\n",
     clk_sel, clk_frac, sclk_sel, pclk_en,  sclk_en);

            // UART7串口无法在rustshyper中正常使用，发现主要是 PCLK_UART7 pclk_uart7_en和SCLK_UART7, sclk_uart7_en 时钟CRU GATE
            // 被Linux重置disable关闭了，寄存器变成：pclk_uart7_en=0xfff7, sclk_uart7_en=0xfffe，只有在Linux访问这个uart7时如cat /dev/ttyS7, 该uart7的CRU GATE才会被临时使能；
            // 另，在Linux内核用的dtb中，serial node中的status="okay"，才能在linux中创建有对应的串口设备，如/dev/ttyS7
            // 在rk3588板子的内核，若要让内核启动并登录到某个串口，指定的串口地址/串口号需要设置这两处：一处在内核启动参数的earlycon，另外是设备树中的fiq debug设备；

            core::ptr::write_volatile(
                (cru_base + 0x0830) as *mut u32,
                (pclk_en & !(1 << 8)) | ((1 << 8) << 16),
            ); // CRU_GATE_CON12, PCLK_UART7, pclk_uart7_en bit8

            core::ptr::write_volatile(
                (cru_base + 0x0834) as *mut u32,
                (sclk_en & !(1 << 15)) | ((1 << 15) << 16),
            );

            let pclk_en = core::ptr::read_volatile((cru_base + 0x0830) as *mut u32); // CRU_GATE_CON12 pclk_uart7_en bit8 = 0
            let sclk_en = core::ptr::read_volatile((cru_base + 0x0834) as *mut u32); // CRU_GATE_CON13 sclk_uart7_en bit15 = 0
            println!(
                "PCLK_UART7, pclk_uart7_en={:#x}, SCLK_UART7, sclk_uart7_en={:#x}",
                pclk_en, sclk_en
            );
        };

        dw_apb_uart::DW8250::iomux_uart7_m2(bus_ioc);

        /////////// rk3588 UART7
        let uart_base: usize = 0xffff_0000_feba0000;
        let mut uart = dw_apb_uart::DW8250::new(uart_base);
        unsafe {
            let addr = uart_base + 0xf8; // UART Component Version = 0x3430322A
            let ucv = core::ptr::read_volatile(addr as *mut u32);

            let addr = uart_base + 0x8; // Interrupt Identity Register = 0x1
            let iir = core::ptr::read_volatile(addr as *mut u32);

            let addr = uart_base + 0x7c; // UART Status Register = 0x6
            let usr = core::ptr::read_volatile(addr as *mut u32);

            println!(
                "Read UART7, UCV={:#x}, USR={:#x}, IIR={:#x}\n",
                ucv, usr, iir
            );
        };
        uart.minit();

        println!("Bencher output to UART7\n");
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

        ///////// LED
        // Blue: GPIO1_D5, 61
        // Red:* GPIO3_B2, 106, num=10
        // Green:* GPIO3_C0, 112, num=16
        // GPIO_ACTIVE_HIGH 表示高电平有效;
        //
        // GPIO单独物理引脚: (GPIO3_C6_u, RK_PC6=22, Output High)
        let GPIO3: usize = 0xffff_0000_fec40000;
        // LED Red
        dw_apb_uart::DW8250::gpio_output(GPIO3, 10, true);
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

    ///////// 评测task调度切换开销
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

    // 每1亿次切换将输出一次GPIO UART信号
    println!("After every 100 million task switches, a GPIO UART signal will be output");
    let switch_count = 100_000_000;
    let iter = 100;

    #[cfg(not(feature = "qemu"))]
    {
        let GPIO3: usize = 0xffff_0000_fec40000;

        // Turn off all LEDs
        dw_apb_uart::DW8250::gpio_output_clear(GPIO3);
        // LED Green
        dw_apb_uart::DW8250::gpio_output(GPIO3, 16, true);
    }

    for i in 0..iter {
        println!(
            "\n---------\nBencher: {} task switch count = {}\n",
            i, switch_count
        );
        #[cfg(not(feature = "qemu"))]
        {
            let uart_base: usize = 0xffff_0000_feba0000;
            let mut uart = dw_apb_uart::DW8250::new(uart_base);

            uart.putchar(b'H');
            uart.putchar(b'i');
            uart.putchar(b'\n');
            uart.putchar(b'\r');
        }

        bench_switch(switch_count);
    }

    println!("\nBencher end");
}
