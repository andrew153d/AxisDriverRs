//! Wall-clock time since boot, driven by the SysTick exception.

use core::sync::atomic::{AtomicU32, Ordering};

use atsamd_hal::ehal::delay::DelayNs;
use cortex_m::peripheral::{syst::SystClkSource, SCB, SYST};
use cortex_m_rt::exception;

use crate::board::CPU_HZ;

const TICKS_PER_MILLI: u32 = CPU_HZ / 1_000;
const TICKS_PER_MICRO: u32 = CPU_HZ / 1_000_000;

static MILLIS: AtomicU32 = AtomicU32::new(0);

/// Starts the clock.
pub fn start(mut syst: SYST) {
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(TICKS_PER_MILLI - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

#[exception]
fn SysTick() {
    MILLIS.fetch_add(1, Ordering::Relaxed);
}

/// Milliseconds since boot, wrapping after 49 days.
pub fn millis() -> u32 {
    MILLIS.load(Ordering::Relaxed)
}

/// Microseconds since boot, wrapping after 71 minutes.
pub fn micros() -> u32 {
    cortex_m::interrupt::free(|_| {
        let mut millis = MILLIS.load(Ordering::Relaxed);
        let mut ticks = SYST::get_current();
        // With interrupts off the millisecond count may be one tick behind.
        // If so, the counter has wrapped and must be read again to match.
        if SCB::is_pendst_pending() {
            millis = millis.wrapping_add(1);
            ticks = SYST::get_current();
        }
        let elapsed_ticks = TICKS_PER_MILLI - 1 - ticks;
        millis
            .wrapping_mul(1_000)
            .wrapping_add(elapsed_ticks / TICKS_PER_MICRO)
    })
}

/// Whether `period_ms` has passed since `*last_ms`; if so, restarts the
/// period from now.
pub fn every(period_ms: u32, last_ms: &mut u32) -> bool {
    let now = millis();
    let elapsed = now.wrapping_sub(*last_ms) >= period_ms;
    if elapsed {
        *last_ms = now;
    }
    elapsed
}

/// Busy-waiting delay.
pub struct Delay;

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_us(ns.div_ceil(1_000));
    }

    fn delay_us(&mut self, us: u32) {
        let start = micros();
        while micros().wrapping_sub(start) < us {}
    }
}
