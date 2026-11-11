//! Step timing: when to step, in which direction, to follow a [`Goal`]
//! without exceeding the speed and acceleration [`Limits`].
//!
//! # Model
//!
//! The motor moves with constant acceleration between steps, so the speed
//! `v` at successive steps obeys `v1^2 = v0^2 +- 2a` (one step travelled),
//! and the step takes `2 / (v0 + v1)` seconds. Keeping the *square* of the
//! speed as the state makes the ramp exact with one square root per step:
//! accelerating adds `2a`, braking subtracts it, and stopping from speed `v`
//! takes `v^2 / 2a` steps.
//!
//! # Time
//!
//! Times are wrapping `u32` tick counts of a free-running timer. Every step
//! is scheduled relative to when the previous one was *due*, not when it was
//! taken, so a late caller shifts individual steps but never slows the
//! motor down.

use libm::{fabsf, sqrtf};

/// Fastest step rate the profile will ask for, whatever the limits say.
pub const MAX_STEP_RATE: f32 = 50_000.0;
/// Longest time between steps. Slower speeds are rounded up to this so that
/// tick differences always fit in half the range of the timer.
const MAX_INTERVAL_TICKS: f32 = (1u32 << 30) as f32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    /// Position increases.
    Clockwise,
    /// Position decreases.
    CounterClockwise,
}

impl Direction {
    /// The direction that makes the position change by `delta`.
    fn of(delta: f32) -> Self {
        if delta < 0.0 {
            Self::CounterClockwise
        } else {
            Self::Clockwise
        }
    }

    fn sign(self) -> i32 {
        match self {
            Self::Clockwise => 1,
            Self::CounterClockwise => -1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    /// Steps per second.
    pub max_speed: f32,
    /// Steps per second squared.
    pub acceleration: f32,
}

/// What the motor should be doing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Goal {
    /// Brake to a standstill.
    Stop,
    /// Run at a constant speed in steps per second; negative is
    /// counter-clockwise.
    Velocity(f32),
    /// Go to a position.
    Position {
        target: i32,
        /// Speed to travel at, if lower than the maximum speed.
        cruise_speed: f32,
        /// Speed to arrive with. Zero stops at the target; anything else
        /// expects a new goal on arrival that carries on in the same
        /// direction.
        exit_speed: f32,
    },
}

impl Goal {
    /// Go to `target` and stop there.
    pub fn stop_at(target: i32) -> Self {
        Self::Position {
            target,
            cruise_speed: f32::INFINITY,
            exit_speed: 0.0,
        }
    }
}

/// The next step, already decided.
#[derive(Clone, Copy, Debug)]
struct Planned {
    due: u32,
    direction: Direction,
    /// Speed squared once the step has been taken.
    speed_sq: f32,
}

#[derive(Clone, Debug)]
pub struct Profile {
    ticks_per_second: f32,
    limits: Limits,
    goal: Goal,
    /// Steps.
    position: i32,
    /// Square of the speed at the last step; zero when stopped.
    speed_sq: f32,
    /// Direction of the last step.
    direction: Direction,
    /// When the last step was due or, when stopped, when the goal was set.
    last_step: u32,
    next: Option<Planned>,
}

impl Profile {
    /// `ticks_per_second` is the rate of the timer that timestamps are taken
    /// from.
    pub fn new(ticks_per_second: u32, limits: Limits) -> Self {
        Self {
            ticks_per_second: ticks_per_second as f32,
            limits,
            goal: Goal::Stop,
            position: 0,
            speed_sq: 0.0,
            direction: Direction::Clockwise,
            last_step: 0,
            next: None,
        }
    }

    pub fn position(&self) -> i32 {
        self.position
    }

    /// Current speed in steps per second; negative is counter-clockwise.
    pub fn velocity(&self) -> f32 {
        sqrtf(self.speed_sq) * self.direction.sign() as f32
    }

    /// Whether the motor is at a standstill with nothing left to do.
    pub fn is_stopped(&self) -> bool {
        self.next.is_none()
    }

    pub fn limits(&self) -> Limits {
        self.limits
    }

    /// Changes the limits. A motor running faster than a new, lower maximum
    /// speed brakes down to it.
    pub fn set_limits(&mut self, limits: Limits, now: u32) {
        self.limits = limits;
        self.replan(now);
    }

    pub fn goal(&self) -> Goal {
        self.goal
    }

    pub fn set_goal(&mut self, goal: Goal, now: u32) {
        self.goal = goal;
        self.replan(now);
    }

    /// Stops dead, without braking, and drops the goal.
    pub fn halt(&mut self) {
        self.goal = Goal::Stop;
        self.speed_sq = 0.0;
        self.next = None;
    }

    /// Redefines the current position. Call [`halt`](Self::halt) first if
    /// the motor should not carry on towards a position goal.
    pub fn set_position(&mut self, position: i32) {
        self.position = position;
    }

    /// When the next step is due, if there is one.
    pub fn next_step_due(&self) -> Option<u32> {
        self.next.map(|next| next.due)
    }

    /// Takes the next step if it is due, returning its direction for the
    /// caller to output. Takes at most one step per call.
    pub fn poll(&mut self, now: u32) -> Option<Direction> {
        let next = self.next?;
        let lateness = now.wrapping_sub(next.due);
        if lateness as i32 > 0 || lateness == 0 {
            self.position = self.position.wrapping_add(next.direction.sign());
            self.direction = next.direction;
            self.speed_sq = next.speed_sq;
            // Stay on the ideal timeline unless hopelessly late, in which
            // case catching up would mean a burst of steps.
            let late_limit = (self.ticks_per_second / 1_000.0) as u32;
            self.last_step = if lateness > late_limit { now } else { next.due };
            self.plan_next();
            Some(next.direction)
        } else {
            None
        }
    }

    /// Reconsiders the pending step after the goal or the limits changed.
    fn replan(&mut self, now: u32) {
        if self.speed_sq == 0.0 {
            // Starting from standstill: the first step is timed from now.
            self.last_step = now;
        }
        self.plan_next();
    }

    fn plan_next(&mut self) {
        self.next = self.decide().map(|(direction, speed_sq)| Planned {
            due: self
                .last_step
                .wrapping_add(self.interval_ticks(self.speed_sq, speed_sq)),
            direction,
            speed_sq,
        });
        if self.next.is_none() {
            self.speed_sq = 0.0;
        }
    }

    /// Ticks a step takes when the speed squared goes from `from` to `to`.
    fn interval_ticks(&self, from: f32, to: f32) -> u32 {
        let speed_sum = sqrtf(from) + sqrtf(to);
        let seconds = if speed_sum > 0.0 {
            2.0 / speed_sum
        } else {
            // A single step from rest to rest: accelerate for half a step,
            // brake for the other half.
            2.0 / sqrtf(self.limits.acceleration)
        };
        (seconds * self.ticks_per_second).clamp(1.0, MAX_INTERVAL_TICKS) as u32
    }

    /// Chooses the direction of the next step and the speed squared to have
    /// after it, or `None` to stand still.
    fn decide(&self) -> Option<(Direction, f32)> {
        // Change in speed squared over one step at full acceleration.
        let ramp = 2.0 * self.limits.acceleration;
        if ramp <= 0.0 {
            return None;
        }
        let max_speed = self.limits.max_speed.clamp(0.0, MAX_STEP_RATE);
        let speed_sq = self.speed_sq;
        // Slow enough to stop, or turn around, without another step.
        let can_stop = speed_sq <= ramp;
        let brake = (!can_stop).then_some((self.direction, speed_sq - ramp));
        let moving_against = |direction| speed_sq > 0.0 && self.direction != direction;

        match self.goal {
            Goal::Stop => brake,

            Goal::Velocity(velocity) => {
                let direction = Direction::of(velocity);
                let target_sq = square(fabsf(velocity).min(max_speed));
                if target_sq == 0.0 {
                    brake
                } else if moving_against(direction) {
                    // Brake first, then set off the other way.
                    brake.or(Some((direction, target_sq.min(ramp))))
                } else {
                    Some((direction, approach(speed_sq, target_sq, ramp)))
                }
            }

            Goal::Position {
                target,
                cruise_speed,
                exit_speed,
            } => {
                let cruise_sq = square(cruise_speed.clamp(0.0, max_speed));
                let distance = target.wrapping_sub(self.position);
                if distance == 0 || cruise_sq == 0.0 {
                    // Arrived, unless it is going too fast to stop here. It
                    // then runs past, and comes back once it has stopped.
                    return brake;
                }
                let direction = Direction::of(distance as f32);
                if moving_against(direction) {
                    return brake.or(Some((direction, cruise_sq.min(ramp))));
                }

                // After this step there must still be room to brake to the
                // exit speed by the time the target is reached.
                let steps_after = (distance.unsigned_abs() - 1) as f32;
                let exit_sq = square(exit_speed.clamp(0.0, cruise_speed));
                let limit_sq = cruise_sq.min(exit_sq + ramp * steps_after);
                Some((direction, approach(speed_sq, limit_sq, ramp)))
            }
        }
    }
}

fn square(x: f32) -> f32 {
    x * x
}

/// Moves `from` towards `to` by at most `max_change`.
fn approach(from: f32, to: f32, max_change: f32) -> f32 {
    if to > from {
        to.min(from + max_change)
    } else {
        to.max(from - max_change)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One tick per microsecond.
    const TICK_HZ: u32 = 1_000_000;
    const LIMITS: Limits = Limits {
        max_speed: 800.0,
        acceleration: 3200.0,
    };

    fn profile() -> Profile {
        Profile::new(TICK_HZ, LIMITS)
    }

    /// Polls exactly when each step is due and returns the due times of the
    /// steps taken until the profile stops or `until` is reached.
    fn run(profile: &mut Profile, now: &mut u32, until: u32) -> Vec<(u32, Direction)> {
        let mut steps = Vec::new();
        while let Some(due) = profile.next_step_due() {
            if due.wrapping_sub(until) as i32 > 0 {
                break;
            }
            *now = due;
            steps.push((due, profile.poll(*now).expect("step is due")));
        }
        *now = (*now).max(until);
        steps
    }

    fn intervals(steps: &[(u32, Direction)]) -> Vec<u32> {
        steps.windows(2).map(|pair| pair[1].0 - pair[0].0).collect()
    }

    fn assert_close(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} is not within {tolerance} of {expected}"
        );
    }

    #[test]
    fn stands_still_without_a_goal() {
        let mut profile = profile();
        assert!(profile.is_stopped());
        assert_eq!(profile.poll(1_000_000), None);
        assert_eq!(profile.next_step_due(), None);
    }

    #[test]
    fn moves_to_a_position_and_stops_exactly_there() {
        for target in [1, 2, 3, 7, 100, 199, 200, 201, 1600, -1, -2, -500] {
            let mut profile = profile();
            let mut now = 0;
            profile.set_goal(Goal::stop_at(target), now);
            let steps = run(&mut profile, &mut now, u32::MAX / 2);

            assert_eq!(steps.len(), target.unsigned_abs() as usize, "to {target}");
            assert_eq!(profile.position(), target);
            assert!(profile.is_stopped());
            assert_eq!(profile.velocity(), 0.0);
        }
    }

    #[test]
    fn position_move_is_a_symmetric_trapezoid() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::stop_at(1600), now);
        let mut steps = run(&mut profile, &mut now, u32::MAX / 2);
        // Include the time the first step takes from the start at zero.
        steps.insert(0, (0, Direction::Clockwise));
        let intervals = intervals(&steps);
        assert_eq!(intervals.len(), 1600);

        // Cruises at the maximum speed: 1250 us per step.
        assert_eq!(*intervals.iter().min().unwrap(), 1250);
        assert_eq!(intervals[800], 1250);
        // Reaching 800 steps/s at 3200 steps/s^2 takes 100 steps, both ways.
        assert!(intervals[99] > 1250);
        assert_eq!(intervals[100], 1250);
        assert_eq!(intervals[1599 - 100], 1250);
        assert!(intervals[1599 - 99] > 1250);
        // The way down mirrors the way up.
        for i in 0..100 {
            let up = intervals[i] as i32;
            let down = intervals[intervals.len() - 1 - i] as i32;
            assert!((up - down).abs() <= 1, "step {i}: {up} vs {down}");
        }
        // Total time: 0.25 s up, 0.25 s down, 1400 steps at 800 steps/s.
        let total = steps.last().unwrap().0 as f32 / 1e6;
        assert_close(total, 0.25 + 0.25 + 1400.0 / 800.0, 0.002);
    }

    #[test]
    fn short_moves_never_reach_the_maximum_speed() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::stop_at(50), now);
        let steps = run(&mut profile, &mut now, u32::MAX / 2);
        // Peak speed after 25 steps is sqrt(2 * 3200 * 25) = 400 steps/s.
        let fastest = *intervals(&steps).iter().min().unwrap();
        assert!((2450..=2600).contains(&fastest), "{fastest} us");
    }

    #[test]
    fn accelerates_at_the_configured_rate() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(800.0), now);
        run(&mut profile, &mut now, 125_000);
        // Half way up the 0.25 s ramp.
        assert_close(profile.velocity(), 400.0, 15.0);
        run(&mut profile, &mut now, 250_000);
        assert_close(profile.velocity(), 800.0, 15.0);
        run(&mut profile, &mut now, 300_000);
        assert_eq!(profile.velocity(), 800.0);
    }

    #[test]
    fn holds_a_constant_velocity_without_drift() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(-250.0), now);
        run(&mut profile, &mut now, 1_000_000);
        let start = profile.position();
        let steps = run(&mut profile, &mut now, 11_000_000);
        // Ten seconds at 250 steps/s, to the step.
        assert!(
            (2499..=2501).contains(&steps.len()),
            "{} steps",
            steps.len()
        );
        assert_eq!(profile.position(), start - steps.len() as i32);
        assert!(steps
            .iter()
            .all(|step| step.1 == Direction::CounterClockwise));
    }

    #[test]
    fn late_polling_does_not_slow_the_motor() {
        let mut profile = profile();
        profile.set_goal(Goal::Velocity(500.0), 0);
        // Poll with up to 300 us of jitter, well under the 2 ms interval.
        let mut steps = 0;
        let mut now = 0u32;
        let mut jitter = 0u32;
        while now < 10_000_000 {
            jitter = (jitter * 73 + 41) % 300;
            now = profile.next_step_due().unwrap() + jitter;
            steps += profile.poll(now).is_some() as u32;
        }
        // 0.156 s to accelerate, during which it covers half the distance.
        let expected = 500.0 * (10.0 - 0.156 / 2.0);
        assert_close(steps as f32, expected, 3.0);
    }

    #[test]
    fn hopelessly_late_polling_does_not_cause_a_burst() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(800.0), now);
        run(&mut profile, &mut now, 1_000_000);

        // Nobody polls for 100 ms.
        now += 100_000;
        assert!(profile.poll(now).is_some());
        let due = profile.next_step_due().unwrap();
        assert_eq!(due, now + 1250);
        assert_eq!(profile.poll(now), None);
    }

    #[test]
    fn reverses_through_standstill() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(800.0), now);
        run(&mut profile, &mut now, 500_000);
        let turned_at = profile.position();

        profile.set_goal(Goal::Velocity(-800.0), now);
        let steps = run(&mut profile, &mut now, 1_500_000);
        // Keeps going clockwise for the 100 steps it takes to brake.
        let overshoot = steps
            .iter()
            .take_while(|step| step.1 == Direction::Clockwise)
            .count();
        assert!((98..=101).contains(&overshoot), "{overshoot} steps");
        assert!(profile.position() < turned_at);
        assert_eq!(profile.velocity(), -800.0);
        // Never faster than the limit on the way.
        assert!(intervals(&steps).iter().all(|&interval| interval >= 1250));
    }

    #[test]
    fn brakes_to_a_stop() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(800.0), now);
        run(&mut profile, &mut now, 500_000);

        profile.set_goal(Goal::Stop, now);
        let steps = run(&mut profile, &mut now, u32::MAX / 2);
        assert!((98..=101).contains(&steps.len()), "{} steps", steps.len());
        assert!(profile.is_stopped());
        assert_eq!(profile.velocity(), 0.0);
    }

    #[test]
    fn halt_stops_dead() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(800.0), now);
        run(&mut profile, &mut now, 500_000);
        profile.halt();
        assert!(profile.is_stopped());
        assert_eq!(profile.poll(now + 1_000_000), None);
    }

    #[test]
    fn retargeting_behind_brakes_then_returns() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::stop_at(1000), now);
        while profile.position() < 300 {
            now = profile.next_step_due().unwrap();
            profile.poll(now);
        }
        profile.set_goal(Goal::stop_at(0), now);
        let mut peak = 0;
        while !profile.is_stopped() {
            now = profile.next_step_due().unwrap();
            profile.poll(now);
            peak = peak.max(profile.position());
        }
        assert_eq!(profile.position(), 0);
        assert!((398..=401).contains(&peak), "peaked at {peak}");
    }

    #[test]
    fn target_too_close_to_stop_at_is_overshot_and_returned_to() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(800.0), now);
        run(&mut profile, &mut now, 500_000);
        let target = profile.position() + 10;

        profile.set_goal(Goal::stop_at(target), now);
        let steps = run(&mut profile, &mut now, u32::MAX / 2);
        assert_eq!(profile.position(), target);
        assert!(profile.is_stopped());
        // 100 steps to stop, 90 of them past the target, then 90 back.
        assert!((185..=195).contains(&steps.len()), "{} steps", steps.len());
    }

    #[test]
    fn cruise_speed_caps_a_position_move() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(
            Goal::Position {
                target: 400,
                cruise_speed: 200.0,
                exit_speed: 0.0,
            },
            now,
        );
        let steps = run(&mut profile, &mut now, u32::MAX / 2);
        assert_eq!(profile.position(), 400);
        assert_eq!(*intervals(&steps).iter().min().unwrap(), 5000);
    }

    #[test]
    fn arrives_at_the_exit_speed() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(
            Goal::Position {
                target: 400,
                cruise_speed: 800.0,
                exit_speed: 300.0,
            },
            now,
        );
        while profile.position() < 400 {
            now = profile.next_step_due().unwrap();
            profile.poll(now);
        }
        assert_close(profile.velocity(), 300.0, 12.0);
    }

    #[test]
    fn lowering_the_maximum_speed_brakes_down_to_it() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(800.0), now);
        run(&mut profile, &mut now, 500_000);
        profile.set_limits(
            Limits {
                max_speed: 400.0,
                ..LIMITS
            },
            now,
        );
        let steps = run(&mut profile, &mut now, 1_000_000);
        assert_eq!(profile.velocity(), 400.0);
        // Braking from 800 to 400 takes (800^2 - 400^2) / 6400 = 75 steps.
        let braking = intervals(&steps).iter().filter(|&&i| i < 2500).count();
        assert!((73..=77).contains(&braking), "{braking} steps");
    }

    #[test]
    fn slow_velocities_are_exact() {
        let mut profile = profile();
        let mut now = 0;
        profile.set_goal(Goal::Velocity(4.0), now);
        let steps = run(&mut profile, &mut now, 10_000_000);
        assert!((39..=40).contains(&steps.len()), "{} steps", steps.len());
        assert_eq!(*intervals(&steps).last().unwrap(), 250_000);
    }

    #[test]
    fn survives_timer_wraparound() {
        let mut profile = profile();
        let mut now = u32::MAX - 200_000;
        profile.set_goal(Goal::stop_at(800), now);
        let mut steps = 0;
        while let Some(due) = profile.next_step_due() {
            // Not due yet just before, due exactly on time.
            assert_eq!(profile.poll(due.wrapping_sub(1)), None);
            now = due;
            steps += profile.poll(now).is_some() as u32;
        }
        assert_eq!(steps, 800);
        assert!(now < u32::MAX / 2, "the timer wrapped during the move");
    }

    #[test]
    fn refuses_to_move_without_acceleration_or_speed() {
        for limits in [
            Limits {
                max_speed: 0.0,
                ..LIMITS
            },
            Limits {
                acceleration: 0.0,
                ..LIMITS
            },
        ] {
            let mut profile = Profile::new(TICK_HZ, limits);
            profile.set_goal(Goal::stop_at(100), 0);
            assert!(profile.is_stopped());
            profile.set_goal(Goal::Velocity(100.0), 0);
            assert!(profile.is_stopped());
        }
    }
}
