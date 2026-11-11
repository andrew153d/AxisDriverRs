//! The motor control state machine.
//!
//! [`Motion`] decides *when* the motor steps in each [`MotorState`]; it owns
//! no hardware. The firmware calls [`Motion::on_timer`] from the step timer
//! interrupt, pulses the step pin when told to, and sleeps for the returned
//! time.

use heapless::Deque;
use libm::fabsf;

use crate::protocol::{HomeDirection, MotorState, PositionMode};
use crate::stepper::{Direction, Stepper};

/// Microsteps per revolution: a 200 step motor driven at 8 microsteps.
pub const STEPS_PER_REVOLUTION: i32 = 200 * 8;
const DEGREES_PER_STEP: f32 = 360.0 / STEPS_PER_REVOLUTION as f32;

const DEFAULT_MAX_SPEED: f32 = 800.0;
const DEFAULT_ACCELERATION: f32 = 3200.0;
const DEFAULT_HOMING_SPEED: u32 = 100;
const DEFAULT_HOMING_THRESHOLD: u16 = 2;

/// How often to check for new work while the motor is stopped.
const IDLE_POLL_US: u32 = 1_000;
/// Shortest time [`Motion::on_timer`] asks to sleep, bounding the interrupt
/// rate.
const MIN_WAKE_US: u32 = 20;
/// Longest time [`Motion::on_timer`] asks to sleep, so that new commands
/// take effect promptly even at very low speeds.
const MAX_WAKE_US: u32 = 10_000;

/// Number of velocity/step segments that can be queued.
pub const PATH_CAPACITY: usize = 1024;
/// Distance to back off the end stop after the coarse homing pass.
const HOMING_BACKOFF_STEPS: i32 = 50;

pub fn degrees_to_steps(degrees: f64) -> i32 {
    (degrees / 360.0 * f64::from(STEPS_PER_REVOLUTION)) as i32
}

pub fn steps_to_degrees(steps: i32) -> f64 {
    f64::from(steps) * 360.0 / f64::from(STEPS_PER_REVOLUTION)
}

/// What the step timer interrupt has to do after [`Motion::on_timer`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Tick {
    /// Pulse the step pin in this direction.
    pub step: Option<Direction>,
    /// Call [`Motion::on_timer`] again after this long.
    pub wake_in_us: u32,
}

/// Shaft measurements the homing sequence relies on.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EncoderFeedback {
    pub velocity_deg_per_s: f32,
    /// Encoder samples per second.
    pub update_rate_hz: f32,
}

/// One leg of a path: run at `speed` until `steps` is reached.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PathSegment {
    mode: PositionMode,
    /// Steps per second.
    speed: u32,
    steps: i32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PathFull;

/// Sensorless homing drives into the end stop and watches the encoder for
/// the shaft stalling, first quickly and then once more slowly.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HomingPhase {
    /// Run at the homing speed until the shaft stalls.
    Coarse,
    /// Back off the end stop.
    BackOff,
    /// Approach again slowly, at half a step per encoder sample.
    Fine,
}

struct Homing {
    direction: HomeDirection,
    /// Steps per second.
    speed: u32,
    /// StallGuard threshold, kept for the driver.
    threshold: u16,
    phase: HomingPhase,
    /// Whether the shaft got up to speed in the current phase, so that the
    /// standstill before it starts moving is not mistaken for a stall.
    reached_speed: bool,
    /// Shaft speed below which the fine pass counts as stalled.
    fine_limit_deg_per_s: f32,
    homed: bool,
}

impl Homing {
    /// Sign of the step direction that runs towards the end stop.
    fn sign(&self) -> f32 {
        match self.direction {
            HomeDirection::Clockwise => -1.0,
            HomeDirection::CounterClockwise => 1.0,
        }
    }
}

pub struct Motion {
    stepper: Stepper,
    state: MotorState,
    /// Target of the position mode, in steps.
    target: i32,
    /// Requested speed of the velocity mode, in steps per second.
    target_velocity: f64,
    path: Deque<PathSegment, PATH_CAPACITY>,
    /// Position at which the current path segment ends.
    path_segment_end: i32,
    path_finished: bool,
    homing: Homing,
    encoder: EncoderFeedback,
}

impl Default for Motion {
    fn default() -> Self {
        Self::new()
    }
}

impl Motion {
    pub fn new() -> Self {
        Self {
            stepper: Stepper::new(DEFAULT_MAX_SPEED, DEFAULT_ACCELERATION),
            state: MotorState::Off,
            target: 0,
            target_velocity: 0.0,
            path: Deque::new(),
            path_segment_end: 0,
            path_finished: false,
            homing: Homing {
                direction: HomeDirection::CounterClockwise,
                speed: DEFAULT_HOMING_SPEED,
                threshold: DEFAULT_HOMING_THRESHOLD,
                phase: HomingPhase::Coarse,
                reached_speed: false,
                fine_limit_deg_per_s: 0.0,
                homed: false,
            },
            encoder: EncoderFeedback::default(),
        }
    }

    pub fn state(&self) -> MotorState {
        self.state
    }

    pub fn set_state(&mut self, state: MotorState) {
        match state {
            MotorState::VelocityStep if self.state != MotorState::VelocityStep => {
                self.stepper.set_speed(0.0);
                self.path_segment_end = self.stepper.position();
            }
            MotorState::Home => {
                self.homing.phase = HomingPhase::Coarse;
                self.homing.reached_speed = false;
                self.homing.homed = false;
            }
            _ => {}
        }
        self.state = state;
    }

    /// Whether the driver's power stage should be enabled.
    pub fn outputs_enabled(&self) -> bool {
        self.state != MotorState::Off
    }

    /// Advances the state machine. Call from the step timer interrupt with
    /// the current time.
    pub fn on_timer(&mut self, now_us: u32) -> Tick {
        let step = match self.state {
            MotorState::Off | MotorState::IdleOn => None,
            MotorState::Position => {
                let step = self.stepper.run(now_us);
                if self.stepper.distance_to_go() == 0 {
                    self.state = MotorState::IdleOn;
                }
                step
            }
            MotorState::Velocity => self.stepper.run_speed(now_us),
            MotorState::VelocityStep => self.run_path(now_us),
            MotorState::Home => self.run_homing(now_us),
        };

        let wake_in_us = self
            .stepper
            .time_to_next_step_us(now_us)
            .map_or(IDLE_POLL_US, |us| us.clamp(MIN_WAKE_US, MAX_WAKE_US));
        Tick { step, wake_in_us }
    }

    fn run_path(&mut self, now_us: u32) -> Option<Direction> {
        let position = self.stepper.position();
        if position == self.path_segment_end {
            let Some(segment) = self.path.pop_front() else {
                self.stepper.set_speed(0.0);
                self.state = MotorState::IdleOn;
                self.path_finished = true;
                return None;
            };
            self.path_segment_end = match segment.mode {
                PositionMode::Absolute => segment.steps,
                PositionMode::Relative => position.wrapping_add(segment.steps),
            };
            let speed = segment.speed as f32;
            self.stepper.set_speed(if position > self.path_segment_end {
                -speed
            } else {
                speed
            });
        }
        self.stepper.run_speed(now_us)
    }

    fn run_homing(&mut self, now_us: u32) -> Option<Direction> {
        let sign = self.homing.sign();
        // Shaft speed towards the end stop.
        let approach_speed = self.encoder.velocity_deg_per_s * sign;

        match self.homing.phase {
            HomingPhase::Coarse => {
                // Stalled once the shaft turns at less than half the
                // commanded speed.
                let commanded = self.homing.speed as f32;
                let stall_limit = commanded * DEGREES_PER_STEP * 0.5;
                let moving = fabsf(approach_speed) > stall_limit;
                if moving || approach_speed < -2.0 {
                    self.homing.reached_speed = true;
                }
                if moving || !self.homing.reached_speed {
                    self.stepper.set_speed(sign * commanded);
                    self.stepper.run_speed(now_us)
                } else {
                    self.stepper.set_speed(0.0);
                    self.stepper.move_by(-(sign as i32) * HOMING_BACKOFF_STEPS);
                    self.homing.phase = HomingPhase::BackOff;
                    None
                }
            }
            HomingPhase::BackOff => {
                let step = self.stepper.run(now_us);
                if self.stepper.distance_to_go() == 0
                    && fabsf(self.encoder.velocity_deg_per_s) < 1.0
                {
                    self.homing.fine_limit_deg_per_s =
                        self.fine_homing_speed() * DEGREES_PER_STEP * 0.3;
                    self.homing.phase = HomingPhase::Fine;
                    self.homing.reached_speed = false;
                }
                step
            }
            HomingPhase::Fine => {
                let moving = fabsf(approach_speed) > self.homing.fine_limit_deg_per_s;
                if moving {
                    self.homing.reached_speed = true;
                }
                if moving || !self.homing.reached_speed {
                    self.stepper.set_speed(sign * self.fine_homing_speed());
                    self.stepper.run_speed(now_us)
                } else {
                    self.stepper.set_speed(0.0);
                    self.homing.homed = true;
                    self.state = MotorState::IdleOn;
                    None
                }
            }
        }
    }

    /// Speed of the fine homing pass in steps per second: slow enough for
    /// the encoder to sample every half step.
    fn fine_homing_speed(&self) -> f32 {
        fabsf(self.encoder.update_rate_hz) / 2.0
    }

    pub fn set_encoder_feedback(&mut self, feedback: EncoderFeedback) {
        self.encoder = feedback;
    }

    /// Steps per second.
    pub fn max_speed(&self) -> u32 {
        self.stepper.max_speed() as u32
    }

    pub fn set_max_speed(&mut self, steps_per_s: u32) {
        self.stepper.set_max_speed(steps_per_s as f32);
    }

    /// Steps per second squared.
    pub fn acceleration(&self) -> u32 {
        self.stepper.acceleration() as u32
    }

    pub fn set_acceleration(&mut self, steps_per_s2: u32) {
        self.stepper.set_acceleration(steps_per_s2 as f32);
    }

    pub fn position_degrees(&self) -> f64 {
        steps_to_degrees(self.stepper.position())
    }

    /// Redefines the current position without moving.
    pub fn set_position_degrees(&mut self, degrees: f64) {
        self.target = degrees_to_steps(degrees);
        self.stepper.set_current_position(self.target);
    }

    pub fn target_degrees(&self) -> f64 {
        steps_to_degrees(self.target)
    }

    /// Starts a move to an absolute position.
    pub fn move_to_degrees(&mut self, degrees: f64) {
        self.move_to(degrees_to_steps(degrees));
    }

    /// Starts a move relative to the previous target, so that successive
    /// relative moves do not accumulate rounding or timing errors.
    pub fn move_by_degrees(&mut self, degrees: f64) {
        self.move_to(self.target.wrapping_add(degrees_to_steps(degrees)));
    }

    fn move_to(&mut self, target: i32) {
        self.set_state(MotorState::Position);
        self.target = target;
        self.stepper.move_to(target);
    }

    /// Steps per second.
    pub fn target_velocity(&self) -> f64 {
        self.target_velocity
    }

    /// Runs at a constant velocity in steps per second, limited to the
    /// maximum speed.
    pub fn set_target_velocity(&mut self, steps_per_s: f64) {
        self.state = MotorState::Velocity;
        self.target_velocity = steps_per_s;
        self.stepper.set_speed(steps_per_s as f32);
    }

    /// Queues a path segment: run at `velocity` steps per second until
    /// `steps` is reached. Start the path with
    /// [`MotorState::VelocityStep`].
    pub fn add_path_segment(
        &mut self,
        velocity: i32,
        steps: i32,
        mode: PositionMode,
    ) -> Result<(), PathFull> {
        let segment = PathSegment {
            mode,
            speed: velocity.unsigned_abs(),
            steps,
        };
        self.path.push_back(segment).map_err(|_| PathFull)
    }

    /// Returns `true` once after a path has run to completion.
    pub fn take_path_finished(&mut self) -> bool {
        core::mem::take(&mut self.path_finished)
    }

    pub fn home_direction(&self) -> HomeDirection {
        self.homing.direction
    }

    pub fn set_home_direction(&mut self, direction: HomeDirection) {
        self.homing.direction = direction;
    }

    /// Steps per second.
    pub fn homing_speed(&self) -> u32 {
        self.homing.speed
    }

    pub fn set_homing_speed(&mut self, steps_per_s: u32) {
        self.homing.speed = steps_per_s;
    }

    pub fn home_threshold(&self) -> u16 {
        self.homing.threshold
    }

    pub fn set_home_threshold(&mut self, threshold: u16) {
        self.homing.threshold = threshold;
    }

    /// Whether the last homing sequence ran to completion.
    pub fn is_homed(&self) -> bool {
        self.homing.homed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drives `motion` the way the step timer interrupt does, returning the
    /// net number of steps taken.
    fn run_for(motion: &mut Motion, now_us: &mut u32, duration_us: u32) -> i32 {
        let end = *now_us + duration_us;
        let mut steps = 0;
        while *now_us < end {
            let tick = motion.on_timer(*now_us);
            steps += match tick.step {
                Some(Direction::Clockwise) => 1,
                Some(Direction::CounterClockwise) => -1,
                None => 0,
            };
            assert!((MIN_WAKE_US..=MAX_WAKE_US).contains(&tick.wake_in_us));
            *now_us += tick.wake_in_us;
        }
        steps
    }

    #[test]
    fn converts_between_degrees_and_steps() {
        assert_eq!(degrees_to_steps(360.0), 1600);
        assert_eq!(degrees_to_steps(-90.0), -400);
        assert_eq!(steps_to_degrees(1600), 360.0);
        assert_eq!(steps_to_degrees(1), 0.225);
    }

    #[test]
    fn starts_switched_off_and_idle() {
        let mut motion = Motion::new();
        assert_eq!(motion.state(), MotorState::Off);
        assert!(!motion.outputs_enabled());
        let tick = motion.on_timer(0);
        assert_eq!(
            tick,
            Tick {
                step: None,
                wake_in_us: IDLE_POLL_US
            }
        );
    }

    #[test]
    fn position_move_ends_idle_at_the_target() {
        let mut motion = Motion::new();
        let mut now = 0;
        motion.move_to_degrees(90.0);
        assert_eq!(motion.state(), MotorState::Position);
        assert!(motion.outputs_enabled());

        assert_eq!(run_for(&mut motion, &mut now, 2_000_000), 400);
        assert_eq!(motion.state(), MotorState::IdleOn);
        assert_eq!(motion.position_degrees(), 90.0);
        assert_eq!(motion.target_degrees(), 90.0);
    }

    #[test]
    fn relative_moves_accumulate_on_the_target() {
        let mut motion = Motion::new();
        let mut now = 0;
        motion.move_by_degrees(45.0);
        motion.move_by_degrees(45.0);
        assert_eq!(motion.target_degrees(), 90.0);
        assert_eq!(run_for(&mut motion, &mut now, 2_000_000), 400);
    }

    #[test]
    fn velocity_mode_steps_at_the_requested_rate() {
        let mut motion = Motion::new();
        let mut now = 0;
        motion.set_target_velocity(-200.0);
        assert_eq!(motion.state(), MotorState::Velocity);
        let steps = run_for(&mut motion, &mut now, 1_000_000);
        assert!((-201..=-199).contains(&steps), "{steps} steps");
    }

    #[test]
    fn slow_steps_are_not_delayed_by_the_wake_limit() {
        let mut motion = Motion::new();
        let mut now = 0;
        // 40 steps/s is 25 ms per step, longer than MAX_WAKE_US.
        motion.set_target_velocity(40.0);
        let steps = run_for(&mut motion, &mut now, 1_000_000);
        assert!((39..=41).contains(&steps), "{steps} steps");
    }

    #[test]
    fn plays_back_a_path() {
        let mut motion = Motion::new();
        let mut now = 0;
        motion
            .add_path_segment(400, 200, PositionMode::Absolute)
            .unwrap();
        motion
            .add_path_segment(-800, -300, PositionMode::Relative)
            .unwrap();
        motion.set_state(MotorState::VelocityStep);

        // 200 steps at 400 steps/s take half a second.
        run_for(&mut motion, &mut now, 450_000);
        assert_eq!(motion.state(), MotorState::VelocityStep);
        assert!((175.0..=185.0).contains(&(motion.position_degrees() / 0.225)));
        assert!(!motion.take_path_finished());
        // Then 300 steps back at 800 steps/s.
        run_for(&mut motion, &mut now, 500_000);
        assert_eq!(motion.position_degrees(), steps_to_degrees(200 - 300));
        assert_eq!(motion.state(), MotorState::IdleOn);
        assert!(motion.take_path_finished());
        assert!(!motion.take_path_finished());
    }

    #[test]
    fn path_queue_is_bounded() {
        let mut motion = Motion::new();
        for _ in 0..PATH_CAPACITY {
            motion
                .add_path_segment(100, 1, PositionMode::Relative)
                .unwrap();
        }
        assert_eq!(
            motion.add_path_segment(100, 1, PositionMode::Relative),
            Err(PathFull)
        );
    }

    #[test]
    fn homing_runs_until_the_shaft_stalls_twice() {
        let mut motion = Motion::new();
        let mut now = 0;
        let moving = |velocity_deg_per_s| EncoderFeedback {
            velocity_deg_per_s,
            update_rate_hz: 100.0,
        };
        motion.set_home_direction(HomeDirection::Clockwise);
        motion.set_state(MotorState::Home);
        assert!(!motion.is_homed());

        // Coarse pass: 100 steps/s is 22.5 deg/s, clockwise is negative.
        motion.set_encoder_feedback(moving(0.0));
        assert!(run_for(&mut motion, &mut now, 100_000) < 0);
        motion.set_encoder_feedback(moving(-22.5));
        assert!(run_for(&mut motion, &mut now, 100_000) < 0);

        // The shaft hits the end stop; the motor backs off and waits for
        // the shaft to settle.
        motion.set_encoder_feedback(moving(5.0));
        assert_eq!(
            run_for(&mut motion, &mut now, 1_000_000),
            HOMING_BACKOFF_STEPS
        );

        // Fine pass at 50 steps/s (11.25 deg/s) until it stalls again.
        let position = motion.position_degrees();
        motion.set_encoder_feedback(moving(0.0));
        assert!(run_for(&mut motion, &mut now, 100_000) < 0);
        motion.set_encoder_feedback(moving(-11.25));
        assert!(run_for(&mut motion, &mut now, 100_000) < 0);
        assert_eq!(motion.state(), MotorState::Home);
        motion.set_encoder_feedback(moving(0.0));
        assert_eq!(run_for(&mut motion, &mut now, 100_000), 0);

        assert!(motion.position_degrees() < position);
        assert_eq!(motion.state(), MotorState::IdleOn);
        assert!(motion.is_homed());
    }
}
