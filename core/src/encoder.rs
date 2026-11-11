//! Shaft position and velocity tracking from the magnetic angle sensor.

use libm::{fmodf, roundf};

use crate::motion::EncoderFeedback;

/// Sensor noise amplitude in degrees when the shaft is still. The filtered
/// angle only follows the raw angle directly once it is this far away.
const WINDOW_RADIUS_DEG: f32 = 0.4;
/// Low-pass coefficient for the angle while it is inside the window.
const WINDOW_ALPHA: f32 = 0.4;
/// Low-pass coefficient for the velocity.
const VELOCITY_ALPHA: f32 = 0.1;
/// How long to count samples for when measuring the update rate.
const RATE_WINDOW_US: u32 = 2_000_000;

/// Wraps an angle into `0.0..360.0`.
pub fn wrap_degrees(angle: f32) -> f32 {
    let angle = fmodf(angle, 360.0);
    if angle < 0.0 {
        angle + 360.0
    } else {
        angle
    }
}

/// Turns a stream of raw `0..360` degree readings into a filtered,
/// multi-turn shaft position and velocity.
pub struct ShaftTracker {
    /// Filtered single-turn angle.
    angle: f32,
    full_turns: i32,
    /// Added to the measured position by [`set_position`](Self::set_position).
    offset: f32,
    position: f32,
    /// Degrees per second.
    velocity: f32,
    last_update_us: u32,
    update_rate_hz: f32,
    rate_window_start_us: u32,
    rate_window_samples: u32,
}

impl ShaftTracker {
    /// Starts tracking from a first reading. `expected_rate_hz` is reported
    /// as the update rate until it has been measured.
    pub fn new(angle_deg: f32, now_us: u32, expected_rate_hz: f32) -> Self {
        let angle = wrap_degrees(angle_deg);
        Self {
            angle,
            full_turns: 0,
            offset: 0.0,
            position: angle,
            velocity: 0.0,
            last_update_us: now_us,
            update_rate_hz: expected_rate_hz,
            rate_window_start_us: now_us,
            rate_window_samples: 0,
        }
    }

    /// Feeds in a new raw reading.
    pub fn update(&mut self, angle_deg: f32, now_us: u32) {
        // The sensor is only good for about a tenth of a degree.
        let raw = roundf(wrap_degrees(angle_deg) * 10.0) / 10.0;

        // Sliding window filter. Work on the shortest signed distance
        // between the angles so that the 360 -> 0 seam needs no special
        // handling.
        let error = wrap_degrees(raw - self.angle + 180.0) - 180.0;
        let previous = self.angle;
        let filtered = if error > WINDOW_RADIUS_DEG {
            previous + error - WINDOW_RADIUS_DEG
        } else if error < -WINDOW_RADIUS_DEG {
            previous + error + WINDOW_RADIUS_DEG
        } else {
            previous + WINDOW_ALPHA * error
        };
        self.angle = wrap_degrees(filtered);

        // Crossing the seam is a full turn.
        let delta = self.angle - previous;
        if delta < -180.0 {
            self.full_turns += 1;
        } else if delta > 180.0 {
            self.full_turns -= 1;
        }

        let position = self.angle + 360.0 * self.full_turns as f32 + self.offset;
        let elapsed_s = now_us.wrapping_sub(self.last_update_us) as f32 / 1_000_000.0;
        if elapsed_s > 0.0 {
            let velocity = (position - self.position) / elapsed_s;
            self.velocity += VELOCITY_ALPHA * (velocity - self.velocity);
        }
        self.position = position;
        self.last_update_us = now_us;

        self.rate_window_samples += 1;
        let window_us = now_us.wrapping_sub(self.rate_window_start_us);
        if window_us >= RATE_WINDOW_US {
            self.update_rate_hz = self.rate_window_samples as f32 * 1_000_000.0 / window_us as f32;
            self.rate_window_start_us = now_us;
            self.rate_window_samples = 0;
        }
    }

    /// Multi-turn shaft position in degrees.
    pub fn position_degrees(&self) -> f32 {
        self.position
    }

    /// Redefines the current shaft position.
    pub fn set_position_degrees(&mut self, degrees: f32) {
        self.offset += degrees - self.position;
        self.position = degrees;
    }

    pub fn velocity_deg_per_s(&self) -> f32 {
        self.velocity
    }

    /// Measured number of updates per second.
    pub fn update_rate_hz(&self) -> f32 {
        self.update_rate_hz
    }

    pub fn feedback(&self) -> EncoderFeedback {
        EncoderFeedback {
            position_degrees: self.position,
            velocity_deg_per_s: self.velocity,
            update_rate_hz: self.update_rate_hz,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_US: u32 = 10_000;

    fn assert_close(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} is not within {tolerance} of {expected}"
        );
    }

    /// Rotates at a constant speed for a number of samples.
    fn rotate(tracker: &mut ShaftTracker, now: &mut u32, angle: &mut f32, step: f32, samples: u32) {
        for _ in 0..samples {
            *angle += step;
            *now += SAMPLE_US;
            tracker.update(wrap_degrees(*angle), *now);
        }
    }

    #[test]
    fn wraps_angles() {
        assert_eq!(wrap_degrees(370.0), 10.0);
        assert_eq!(wrap_degrees(-10.0), 350.0);
        assert_eq!(wrap_degrees(360.0), 0.0);
    }

    #[test]
    fn ignores_noise_at_standstill() {
        let mut tracker = ShaftTracker::new(90.0, 0, 100.0);
        let mut now = 0;
        for i in 0..200 {
            now += SAMPLE_US;
            tracker.update(if i % 2 == 0 { 90.2 } else { 89.8 }, now);
        }
        assert_close(tracker.position_degrees(), 90.0, 0.2);
        assert_close(tracker.velocity_deg_per_s(), 0.0, 5.0);
    }

    #[test]
    fn counts_full_turns_in_both_directions() {
        let (mut now, mut angle) = (0, 350.0);
        let mut tracker = ShaftTracker::new(angle, now, 100.0);

        // 2.5 turns forwards at 2 degrees per sample.
        rotate(&mut tracker, &mut now, &mut angle, 2.0, 450);
        assert_close(tracker.position_degrees(), 350.0 + 900.0, 0.5);
        assert_close(tracker.velocity_deg_per_s(), 200.0, 2.0);

        // ...and 5 turns back.
        rotate(&mut tracker, &mut now, &mut angle, -2.0, 900);
        assert_close(tracker.position_degrees(), 350.0 - 900.0, 0.5);
        assert_close(tracker.velocity_deg_per_s(), -200.0, 2.0);
    }

    #[test]
    fn set_position_rebases_without_a_velocity_spike() {
        let (mut now, mut angle) = (0, 10.0);
        let mut tracker = ShaftTracker::new(angle, now, 100.0);
        tracker.set_position_degrees(1000.0);
        rotate(&mut tracker, &mut now, &mut angle, 1.0, 100);
        assert_close(tracker.position_degrees(), 1100.0, 0.5);
        assert_close(tracker.velocity_deg_per_s(), 100.0, 1.0);
    }

    #[test]
    fn measures_the_update_rate() {
        let (mut now, mut angle) = (0, 0.0);
        let mut tracker = ShaftTracker::new(angle, now, 50.0);
        assert_eq!(tracker.update_rate_hz(), 50.0);
        rotate(&mut tracker, &mut now, &mut angle, 0.0, 250);
        assert_close(tracker.update_rate_hz(), 100.0, 0.5);
        assert_eq!(tracker.feedback().update_rate_hz, tracker.update_rate_hz());
    }
}
