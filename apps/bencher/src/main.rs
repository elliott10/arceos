#![cfg_attr(feature = "axstd", no_std)]
#![cfg_attr(feature = "axstd", no_main)]

#[macro_use]
#[cfg(feature = "axstd")]
extern crate axstd as std;

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

fn bench_switch() {
    let iter = 10_000_000;
    thread::spawn(move || {
        for _ in 0..iter / 2 {
            thread::yield_now();
        }
    });

    let start = now_tsc();
    for _ in 0..iter / 2 {
        thread::yield_now();
    }
    let end = now_tsc();

    Bencher::new("switch").reset(iter, end - start).show();
}

#[cfg_attr(feature = "axstd", no_mangle)]
fn main() {
    println!("Bencher start ...\n");

    Bencher::new("rdtsc")
        .bench_many(|| now_tsc(), 10000, 100_000_000)
        .show();

    bench_spawn();
    bench_switch();
    bench_condvar();

    println!("\nBencher end");
}
