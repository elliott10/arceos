#[cfg(target_arch = "x86_64")]
const TSC_FREQ_MHZ: u64 = 4000;

pub const fn div_round(n: u64, d: u64) -> u64 {
    (n + d / 2) / d
}

#[cfg(target_arch = "x86_64")]
#[inline]
pub fn now_tsc() -> u64 {
    unsafe { core::arch::x86_64::__rdtscp(&mut 0) }
}

#[cfg(target_arch = "x86_64")]
#[inline]
pub fn now_ns() -> u64 {
    now_tsc() * 1000 / TSC_FREQ_MHZ
}

#[cfg(target_arch = "x86_64")]
pub fn ticks_to_nanos(ticks: u64) -> u64 {
    ticks * 1_000 / TSC_FREQ_MHZ
}

#[cfg(target_arch = "aarch64")]
use aarch64_cpu::registers::{CNTPCT_EL0, CNTFRQ_EL0, Readable};

#[cfg(target_arch = "aarch64")]
#[inline]
pub fn now_tsc() -> u64 {
    CNTPCT_EL0.get()
}

#[cfg(target_arch = "aarch64")]
#[inline]
pub fn now_ns() -> u64 {
    let freq = CNTFRQ_EL0.get() as u64;
    now_tsc() * 1_000_000_000 / freq
}

#[cfg(target_arch = "aarch64")]
pub fn ticks_to_nanos(ticks: u64) -> u64 {
    let freq = CNTFRQ_EL0.get() as u64;
    ticks * 1_000_000_000 / freq
}

pub struct Bencher {
    name: &'static str,
    count: u64,
    sum_tsc: u64,
}

impl Bencher {
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            count: 0,
            sum_tsc: 0,
        }
    }

    #[inline]
    pub fn bench_fn(f: impl FnOnce()) -> u64 {
        let start = now_tsc();
        f();
        now_tsc() - start
    }

    #[inline]
    pub fn add_result(&mut self, tsc: u64) {
        self.count += 1;
        self.sum_tsc += tsc;
    }

    #[inline]
    pub fn bench_once<T>(&mut self, f: impl FnOnce() -> T) -> T {
        let start = now_tsc();
        let res = f();
        self.add_result(now_tsc() - start);
        res
    }

    pub fn bench_many<T>(&mut self, f: impl Fn() -> T, warmup: usize, run: usize) -> &mut Self {
        for _ in 0..warmup {
            let _ = f();
        }

        let start = now_tsc();
        for _ in 0..run {
            let _ = f();
        }
        let elapsed = now_tsc() - start;

        self.count += run as u64;
        self.sum_tsc += elapsed;
        self
    }

    // Maybe - xiaoluoyuan@163.com
    pub fn reset(&mut self, run: u64, elapsed: u64) -> &mut Self {
        self.count += run as u64;
        self.sum_tsc += elapsed;
        self
    }

    pub fn show(&self) {
        println!("Benchmark: {}", self.name);
        println!("  Iterations: {}", self.count);
        if self.count == 0 {
            return;
        }
        println!("  Average cycles: {}", div_round(self.sum_tsc, self.count));
        println!(
            "  Average nanoseconds: {}",
            div_round(ticks_to_nanos(self.sum_tsc), self.count)
        );
    }
}

// macro_rules! bench_expr {
//     ($f:expr) => {{
//         let start = now_ns();
//         $f;
//         now_ns() - start
//     }};
// }

// macro_rules! bench {
//     ($f:expr, $name:expr, $iter:expr) => {{
//         // warmup
//         for _ in 0..10000 {
//             $f();
//         }

//         let elapsed = bench_expr!({
//             for _ in 0..$iter {
//                 $f();
//             }
//         });
//         println!("Benchmark: {}", $name);
//         println!("  Iterations: {}", $iter);
//         println!(
//             "  Elapsed: {}.{:03} s",
//             elapsed.as_secs(),
//             elapsed.subsec_millis()
//         );
//         println!("  Latency: {} ns", elapsed.as_nanos() / $iter as u128);
//     }};
// }
