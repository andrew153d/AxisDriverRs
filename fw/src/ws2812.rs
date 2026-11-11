//! Bit-banged driver for the single WS2812 ("NeoPixel") on the board.

use core::arch::asm;

use atsamd_hal::pac;
use axis_core::types::Rgb;
use cortex_m::peripheral::DWT;

use crate::board::{PixelPin, CPU_HZ};

/// The pixel is on PB05.
const PORT_GROUP: usize = 1;
const PIN_MASK: u32 = 1 << 5;

/// Time the line must stay low for the pixel to latch its data.
const RESET_CYCLES: u32 = CPU_HZ / 1_000_000 * 50;

pub struct Ws2812 {
    _pin: PixelPin,
}

impl Ws2812 {
    pub fn new(pin: PixelPin) -> Self {
        Self { _pin: pin }
    }

    pub fn write(&mut self, color: Rgb) {
        // The pixel expects green first, most significant bit first.
        let frame = u32::from(color.g) << 24 | u32::from(color.r) << 16 | u32::from(color.b) << 8;

        // SAFETY: only this driver writes PB05, which it owns. The other
        // bits of the port are written back unchanged, and cannot change in
        // between because interrupts are disabled.
        let output = unsafe { (*pac::Port::ptr()).group(PORT_GROUP).out().as_ptr() };
        cortex_m::interrupt::free(|_| unsafe {
            let low = output.read_volatile() & !PIN_MASK;
            write_frame(output, low | PIN_MASK, low, frame);
        });

        // Hold the line low so the pixel latches, and so that a write right
        // after this one starts a new frame.
        let start = DWT::cycle_count();
        while DWT::cycle_count().wrapping_sub(start) < RESET_CYCLES {}
    }
}

/// Clocks out the top 24 bits of `frame`, writing `high` and `low` to the
/// port output register at `output`.
///
/// Bits are timed against the DWT cycle counter at 120 MHz: every bit takes
/// 150 cycles (1.25 us), of which the line is high for 20 cycles (a zero) or
/// 95 cycles (a one), plus the overhead of the loop.
///
/// # Safety
///
/// `output` must point to a port output register, the cycle counter must be
/// running and interrupts must be disabled.
#[inline(never)]
unsafe fn write_frame(output: *mut u32, high: u32, low: u32, frame: u32) {
    const DWT_CYCCNT: usize = 0xE000_1004;

    asm!(
        // r5 = start of the first bit
        "ldr r5, [r4]",
        "adds r5, #150",
        "mov r8, #24",
        // Wait for the start of the bit, then drive the line high. All
        // comparisons are on signed differences so that the cycle counter
        // wrapping around cannot stall the loop.
        "2:",
        "ldr r9, [r4]",
        "subs.w r9, r5, r9",
        "bgt 2b",
        "str r1, [r0]",
        // r5 = start of the next bit
        "ldr r5, [r4]",
        "adds r5, #150",
        "lsls r3, r3, #1",
        "bcs 4f",
        // Zero: stay high until 130 cycles remain.
        "3:",
        "ldr r9, [r4]",
        "sub.w r9, r5, r9",
        "cmp r9, #130",
        "bgt 3b",
        "b 5f",
        // One: stay high until 55 cycles remain.
        "4:",
        "ldr r9, [r4]",
        "sub.w r9, r5, r9",
        "cmp r9, #55",
        "bgt 4b",
        "5:",
        "str r2, [r0]",
        "subs.w r8, r8, #1",
        "bne 2b",
        in("r0") output,
        in("r1") high,
        in("r2") low,
        inout("r3") frame => _,
        in("r4") DWT_CYCCNT,
        lateout("r5") _,
        lateout("r8") _,
        lateout("r9") _,
        options(nostack),
    );
}
