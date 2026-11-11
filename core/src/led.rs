//! Sequencing of the addressable status LED.

use heapless::Deque;

use crate::protocol::Rgb;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct LedStep {
    color: Rgb,
    duration_ms: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QueueFull;

/// A queue of colors to show one after another, each for a given time.
///
/// The last color stays on once the queue runs empty.
pub struct LedSequence<const N: usize = 32> {
    steps: Deque<LedStep, N>,
    color: Rgb,
    /// When the current step started and how long it lasts.
    step_started_ms: u32,
    step_duration_ms: u32,
}

impl<const N: usize> Default for LedSequence<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> LedSequence<N> {
    pub const fn new() -> Self {
        Self {
            steps: Deque::new(),
            color: Rgb::BLACK,
            step_started_ms: 0,
            step_duration_ms: 0,
        }
    }

    /// The color currently shown.
    pub fn color(&self) -> Rgb {
        self.color
    }

    /// Changes the color immediately. Queued steps still play afterwards.
    pub fn set_color(&mut self, color: Rgb) {
        self.color = color;
    }

    /// Queues `color` to be shown for `duration_ms`.
    pub fn push(&mut self, color: Rgb, duration_ms: u32) -> Result<(), QueueFull> {
        self.steps
            .push_back(LedStep { color, duration_ms })
            .map_err(|_| QueueFull)
    }

    /// Queues a flash of `color` followed by darkness.
    ///
    /// Used for activity indication, so it is silently skipped while the
    /// queue is too full to hold both steps.
    pub fn flash(&mut self, color: Rgb, duration_ms: u32) {
        if self.steps.len() + 2 <= N {
            let _ = self.push(color, duration_ms);
            let _ = self.push(Rgb::BLACK, 1);
        }
    }

    /// Advances the sequence, returning the new color when it changes to the
    /// next step.
    pub fn poll(&mut self, now_ms: u32) -> Option<Rgb> {
        if now_ms.wrapping_sub(self.step_started_ms) < self.step_duration_ms {
            return None;
        }
        let step = self.steps.pop_front()?;
        self.color = step.color;
        self.step_started_ms = now_ms;
        self.step_duration_ms = step.duration_ms;
        Some(step.color)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plays_steps_for_their_duration() {
        let mut led = LedSequence::<4>::new();
        led.push(Rgb::RED, 100).unwrap();
        led.push(Rgb::BLUE, 50).unwrap();

        assert_eq!(led.poll(1000), Some(Rgb::RED));
        assert_eq!(led.poll(1099), None);
        assert_eq!(led.color(), Rgb::RED);
        assert_eq!(led.poll(1100), Some(Rgb::BLUE));
        assert_eq!(led.poll(1150), None);
        assert_eq!(led.color(), Rgb::BLUE);
    }

    #[test]
    fn flash_ends_dark_and_never_overflows() {
        let mut led = LedSequence::<3>::new();
        led.flash(Rgb::GREEN, 10);
        led.flash(Rgb::GREEN, 10);
        assert_eq!(led.poll(0), Some(Rgb::GREEN));
        assert_eq!(led.poll(10), Some(Rgb::BLACK));
        assert_eq!(led.poll(11), None);
    }

    #[test]
    fn reports_a_full_queue() {
        let mut led = LedSequence::<1>::new();
        led.push(Rgb::RED, 1).unwrap();
        assert_eq!(led.push(Rgb::RED, 1), Err(QueueFull));
    }
}
