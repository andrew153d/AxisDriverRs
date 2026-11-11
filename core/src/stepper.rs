//! Step timing for a step/direction driver.
//!
//! A port of the AccelStepper algorithm used by the original firmware, which
//! implements David Austin's "Generate stepper-motor speed profiles in real
//! time". The equation numbers below refer to that article.
//!
//! [`Stepper`] only does the bookkeeping: [`Stepper::run`] and
//! [`Stepper::run_speed`] report when a step is due and the caller pulses the
//! pins. Times are wrapping microsecond timestamps.

use libm::{fabsf, sqrtf};

const MICROS_PER_SECOND: f32 = 1_000_000.0;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    /// Position increases.
    Clockwise,
    /// Position decreases.
    CounterClockwise,
}

#[derive(Clone, Debug)]
pub struct Stepper {
    /// Steps.
    position: i32,
    target: i32,
    /// Steps per second, negative when moving counter-clockwise.
    speed: f32,
    max_speed: f32,
    /// Steps per second squared.
    acceleration: f32,
    /// Time between steps; zero when stopped.
    step_interval_us: u32,
    last_step_us: u32,
    direction: Direction,
    /// Position on the speed ramp: positive while accelerating, negative
    /// while decelerating, zero when stopped (`n` in the article).
    ramp_step: i32,
    /// Interval of the first step of a ramp (`c0`).
    initial_interval_us: f32,
    /// Interval of the latest step of the ramp (`cn`).
    ramp_interval_us: f32,
    /// Interval at maximum speed.
    min_interval_us: f32,
}

impl Stepper {
    pub fn new(max_speed: f32, acceleration: f32) -> Self {
        let mut stepper = Self {
            position: 0,
            target: 0,
            speed: 0.0,
            max_speed: 0.0,
            acceleration: 0.0,
            step_interval_us: 0,
            last_step_us: 0,
            direction: Direction::CounterClockwise,
            ramp_step: 0,
            initial_interval_us: 0.0,
            ramp_interval_us: 0.0,
            min_interval_us: 1.0,
        };
        stepper.set_acceleration(acceleration);
        stepper.set_max_speed(max_speed);
        stepper
    }

    pub fn position(&self) -> i32 {
        self.position
    }

    pub fn target(&self) -> i32 {
        self.target
    }

    pub fn distance_to_go(&self) -> i32 {
        self.target.wrapping_sub(self.position)
    }

    pub fn speed(&self) -> f32 {
        self.speed
    }

    pub fn max_speed(&self) -> f32 {
        self.max_speed
    }

    pub fn acceleration(&self) -> f32 {
        self.acceleration
    }

    /// Time between steps at the current speed; zero when stopped.
    pub fn step_interval_us(&self) -> u32 {
        self.step_interval_us
    }

    /// Time left until the next step is due, or `None` when stopped.
    pub fn time_to_next_step_us(&self, now_us: u32) -> Option<u32> {
        (self.step_interval_us != 0).then(|| {
            self.step_interval_us
                .saturating_sub(now_us.wrapping_sub(self.last_step_us))
        })
    }

    /// Sets the target for [`run`](Self::run).
    pub fn move_to(&mut self, target: i32) {
        if self.target != target {
            self.target = target;
            self.compute_new_speed();
        }
    }

    /// Sets the target relative to the current position.
    pub fn move_by(&mut self, steps: i32) {
        self.move_to(self.position.wrapping_add(steps));
    }

    /// Redefines the current position and stops the motor dead.
    pub fn set_current_position(&mut self, position: i32) {
        self.position = position;
        self.target = position;
        self.ramp_step = 0;
        self.step_interval_us = 0;
        self.speed = 0.0;
    }

    pub fn set_max_speed(&mut self, max_speed: f32) {
        let max_speed = fabsf(max_speed);
        if self.max_speed == max_speed {
            return;
        }
        self.max_speed = max_speed;
        self.min_interval_us = MICROS_PER_SECOND / max_speed;
        if self.ramp_step > 0 {
            self.ramp_step = self.steps_to_stop();
            self.compute_new_speed();
        }
    }

    pub fn set_acceleration(&mut self, acceleration: f32) {
        let acceleration = fabsf(acceleration);
        if acceleration == 0.0 || self.acceleration == acceleration {
            return;
        }
        // Keep the same speed on the new, steeper or shallower ramp.
        self.ramp_step = (self.ramp_step as f32 * (self.acceleration / acceleration)) as i32;
        self.initial_interval_us = 0.676 * sqrtf(2.0 / acceleration) * MICROS_PER_SECOND; // Equation 15
        self.acceleration = acceleration;
        self.compute_new_speed();
    }

    /// Sets the constant speed for [`run_speed`](Self::run_speed), limited to
    /// the maximum speed.
    pub fn set_speed(&mut self, speed: f32) {
        if speed == self.speed {
            return;
        }
        let speed = speed.clamp(-self.max_speed, self.max_speed);
        if speed == 0.0 {
            self.step_interval_us = 0;
        } else {
            self.step_interval_us = fabsf(MICROS_PER_SECOND / speed) as u32;
            self.direction = if speed > 0.0 {
                Direction::Clockwise
            } else {
                Direction::CounterClockwise
            };
        }
        self.speed = speed;
    }

    /// Advances one step at the current speed if it is due, returning the
    /// direction of the step to output.
    pub fn run_speed(&mut self, now_us: u32) -> Option<Direction> {
        if self.step_interval_us == 0
            || now_us.wrapping_sub(self.last_step_us) < self.step_interval_us
        {
            return None;
        }
        self.position = match self.direction {
            Direction::Clockwise => self.position.wrapping_add(1),
            Direction::CounterClockwise => self.position.wrapping_sub(1),
        };
        self.last_step_us = now_us;
        Some(self.direction)
    }

    /// Advances one step towards the target if it is due, accelerating and
    /// decelerating as needed. Returns the direction of the step to output.
    pub fn run(&mut self, now_us: u32) -> Option<Direction> {
        let step = self.run_speed(now_us);
        if step.is_some() {
            self.compute_new_speed();
        }
        step
    }

    /// Steps needed to stop from the current speed (Equation 16).
    fn steps_to_stop(&self) -> i32 {
        (self.speed * self.speed / (2.0 * self.acceleration)) as i32
    }

    /// Picks the interval to the next step of a move towards the target.
    fn compute_new_speed(&mut self) {
        let distance = self.distance_to_go();
        let steps_to_stop = self.steps_to_stop();

        if distance == 0 && steps_to_stop <= 1 {
            // At the target and slow enough to stop.
            self.step_interval_us = 0;
            self.speed = 0.0;
            self.ramp_step = 0;
            return;
        }

        if distance != 0 {
            let towards_target = if distance > 0 {
                Direction::Clockwise
            } else {
                Direction::CounterClockwise
            };
            let moving_away = self.direction != towards_target;
            let must_brake = steps_to_stop >= distance.saturating_abs();

            if self.ramp_step > 0 {
                // Accelerating: start to decelerate in time to stop at the
                // target, or straight away when heading the wrong way.
                if must_brake || moving_away {
                    self.ramp_step = -steps_to_stop;
                }
            } else if self.ramp_step < 0 && !must_brake && !moving_away {
                // Decelerating, but the target has moved further away.
                self.ramp_step = -self.ramp_step;
            }
        }

        if self.ramp_step == 0 {
            // First step from standstill.
            self.ramp_interval_us = self.initial_interval_us;
            self.direction = if distance > 0 {
                Direction::Clockwise
            } else {
                Direction::CounterClockwise
            };
        } else {
            // Equation 13
            self.ramp_interval_us -=
                2.0 * self.ramp_interval_us / (4.0 * self.ramp_step as f32 + 1.0);
            self.ramp_interval_us = self.ramp_interval_us.max(self.min_interval_us);
        }
        self.ramp_step += 1;
        self.step_interval_us = self.ramp_interval_us as u32;
        self.speed = MICROS_PER_SECOND / self.ramp_interval_us;
        if self.direction == Direction::CounterClockwise {
            self.speed = -self.speed;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Runs the stepper against a simulated clock until it has been stopped
    /// at its target for a while, returning the times of all steps.
    fn run_to_target(stepper: &mut Stepper) -> Vec<u32> {
        let mut steps = Vec::new();
        let mut idle_ticks = 0;
        let mut now = 0;
        while idle_ticks < 1000 {
            now += 10;
            assert!(now < 60_000_000, "move did not finish");
            if stepper.run(now).is_some() {
                steps.push(now);
            }
            if stepper.distance_to_go() == 0 && stepper.speed() == 0.0 {
                idle_ticks += 1;
            }
        }
        steps
    }

    #[test]
    fn moves_to_the_target_and_stops() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        stepper.move_to(1600);
        let steps = run_to_target(&mut stepper);

        assert_eq!(steps.len(), 1600);
        assert_eq!(stepper.position(), 1600);
        assert_eq!(stepper.step_interval_us(), 0);
    }

    #[test]
    fn moves_backwards() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        stepper.move_to(-500);
        let mut directions = Vec::new();
        let mut now = 0;
        while stepper.distance_to_go() != 0 {
            now += 10;
            directions.extend(stepper.run(now));
        }
        assert_eq!(stepper.position(), -500);
        assert!(directions.iter().all(|&d| d == Direction::CounterClockwise));
    }

    #[test]
    fn follows_a_trapezoidal_profile() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        stepper.move_to(1600);
        let steps = run_to_target(&mut stepper);
        let intervals: Vec<u32> = steps.windows(2).map(|pair| pair[1] - pair[0]).collect();

        // Never faster than the maximum speed (1250 us per step)...
        let fastest = *intervals.iter().min().unwrap();
        assert!(
            (1240..=1260).contains(&fastest),
            "fastest step {fastest} us"
        );
        // ...cruising at it in the middle...
        assert!(intervals[800] <= 1260);
        // ...and ramping at both ends.
        assert!(intervals[0] > 4 * fastest);
        assert!(intervals[intervals.len() - 1] > 4 * fastest);
        // Accelerating to 800 steps/s at 3200 steps/s^2 takes 100 steps.
        assert!(intervals[60] > 1300);
        assert!(intervals[140] <= 1260);
    }

    #[test]
    fn runs_at_constant_speed() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        stepper.set_speed(-400.0);
        assert_eq!(stepper.step_interval_us(), 2500);

        let mut steps = 0;
        for now in (0..=1_000_000).step_by(50) {
            if let Some(direction) = stepper.run_speed(now) {
                assert_eq!(direction, Direction::CounterClockwise);
                steps += 1;
            }
        }
        assert!((399..=401).contains(&steps), "{steps} steps in one second");
        assert_eq!(stepper.position(), -steps);
    }

    #[test]
    fn limits_speed_to_the_maximum() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        stepper.set_speed(5000.0);
        assert_eq!(stepper.speed(), 800.0);
        stepper.set_speed(0.0);
        assert_eq!(stepper.step_interval_us(), 0);
        assert_eq!(stepper.run_speed(1_000_000), None);
    }

    #[test]
    fn reports_time_to_the_next_step() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        assert_eq!(stepper.time_to_next_step_us(0), None);

        stepper.set_speed(100.0);
        assert!(stepper.run_speed(50_000).is_some());
        assert_eq!(stepper.time_to_next_step_us(52_000), Some(8_000));
        assert_eq!(stepper.time_to_next_step_us(70_000), Some(0));
    }

    #[test]
    fn survives_timestamp_wraparound() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        stepper.set_speed(800.0);
        assert!(stepper.run_speed(u32::MAX - 100).is_some());
        assert_eq!(stepper.run_speed(u32::MAX), None);
        assert!(stepper.run_speed(1200).is_some());
    }

    #[test]
    fn retargeting_mid_move_reverses_smoothly() {
        let mut stepper = Stepper::new(800.0, 3200.0);
        stepper.move_to(1000);
        let mut now = 0;
        while stepper.position() < 300 {
            now += 10;
            stepper.run(now);
        }
        stepper.move_to(0);
        let mut overshoot = 0;
        while stepper.distance_to_go() != 0 || stepper.speed() != 0.0 {
            now += 10;
            stepper.run(now);
            overshoot = overshoot.max(stepper.position());
        }
        assert_eq!(stepper.position(), 0);
        // It has to brake first: stopping from 800 steps/s takes 100 steps.
        assert!((380..=420).contains(&overshoot), "peaked at {overshoot}");
    }
}
