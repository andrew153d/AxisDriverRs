//! The addressable status pixel.

use axis_core::led::{LedSequence, QueueFull};
use axis_core::types::Rgb;

use crate::clock;
use crate::ws2812::Ws2812;

/// How long the pixel flashes for each message received or sent.
const ACTIVITY_FLASH_MS: u32 = 10;

pub struct StatusPixel {
    driver: Ws2812,
    sequence: LedSequence,
}

impl StatusPixel {
    pub fn new(driver: Ws2812) -> Self {
        Self {
            driver,
            sequence: LedSequence::new(),
        }
    }

    pub fn color(&self) -> Rgb {
        self.sequence.color()
    }

    /// Changes the color immediately.
    pub fn set_color(&mut self, color: Rgb) {
        self.sequence.set_color(color);
        self.driver.write(color);
    }

    /// Queues `color` to be shown for `duration_ms`.
    pub fn push(&mut self, color: Rgb, duration_ms: u32) -> Result<(), QueueFull> {
        self.sequence.push(color, duration_ms)
    }

    /// Queues a flash of `color`, if there is room for it.
    pub fn flash(&mut self, color: Rgb, duration_ms: u32) {
        self.sequence.flash(color, duration_ms);
    }

    /// Queues the flash that signals a message being received or sent.
    pub fn flash_activity(&mut self, color: Rgb) {
        self.flash(color, ACTIVITY_FLASH_MS);
    }

    /// Shows the next queued color once the current one has had its time.
    pub fn poll(&mut self) {
        if let Some(color) = self.sequence.poll(clock::millis()) {
            self.driver.write(color);
        }
    }
}
