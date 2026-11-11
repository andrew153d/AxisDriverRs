//! The motor control state machine.
//!
//! [`Motion`] turns the host's commands and the [`MotorState`] into goals
//! for the speed [`Profile`], and supervises the multi-stage modes: path
//! playback and homing. It owns no hardware. The firmware calls
//! [`Motion::on_timer`] from the step timer interrupt, pulses the step pin
//! when told to, and sets the timer for the returned time.
//!
//! All motion, in every mode, is limited to the configured maximum speed
//! and acceleration.

use heapless::Deque;
use libm::fabsf;

use crate::profile::{Direction, Goal, Limits, Profile};
use crate::protocol::{HomeDirection, MotorState, PositionMode};

/// Microsteps per revolution: a 200 step motor driven at 8 microsteps.
pub const STEPS_PER_REVOLUTION: i32 = 200 * 8;
const DEGREES_PER_STEP: f32 = 360.0 / STEPS_PER_REVOLUTION as f32;

const DEFAULT_LIMITS: Limits = Limits {
    max_speed: 800.0,
    acceleration: 3200.0,
};
const DEFAULT_HOMING_SPEED: u32 = 100;
const DEFAULT_HOMING_THRESHOLD: u16 = 2;

/// How often [`Motion::on_timer`] wants to run while no step is due, to
/// supervise the current mode.
const SUPERVISION_HZ: u32 = 1_000;

/// Number of path segments that can be queued.
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
    /// Timer value at which to call [`Motion::on_timer`] again. It may
    /// already have passed, in which case call again straight away.
    pub wake_at: u32,
}

/// Shaft measurements the homing sequence relies on.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EncoderFeedback {
    pub velocity_deg_per_s: f32,
    /// Encoder samples per second.
    pub update_rate_hz: f32,
}

/// One leg of a path as queued by the host: travel at `speed` to `steps`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PathSegment {
    mode: PositionMode,
    /// Steps per second.
    speed: u32,
    steps: i32,
}

impl PathSegment {
    /// The position this segment ends at when it starts from `start`.
    fn end(&self, start: i32) -> i32 {
        match self.mode {
            PositionMode::Absolute => self.steps,
            PositionMode::Relative => start.wrapping_add(self.steps),
        }
    }
}

/// The path segment being travelled.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ActiveSegment {
    start: i32,
    end: i32,
    speed: f32,
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
    /// Speed of the fine pass, in steps per second.
    fine_speed: f32,
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
    profile: Profile,
    /// Timer ticks between supervision runs.
    supervision_ticks: u32,
    /// Time of the latest [`on_timer`](Self::on_timer) call, which commands
    /// are timed from.
    now: u32,
    state: MotorState,
    /// Target of the position mode, in steps.
    target: i32,
    /// Requested speed of the velocity mode, in steps per second.
    target_velocity: f64,
    path: Deque<PathSegment, PATH_CAPACITY>,
    segment: Option<ActiveSegment>,
    path_finished: bool,
    homing: Homing,
    encoder: EncoderFeedback,
}

impl Motion {
    /// `ticks_per_second` is the rate of the timer whose values are passed
    /// to [`on_timer`](Self::on_timer).
    pub fn new(ticks_per_second: u32) -> Self {
        Self {
            profile: Profile::new(ticks_per_second, DEFAULT_LIMITS),
            supervision_ticks: ticks_per_second / SUPERVISION_HZ,
            now: 0,
            state: MotorState::Off,
            target: 0,
            target_velocity: 0.0,
            path: Deque::new(),
            segment: None,
            path_finished: false,
            homing: Homing {
                direction: HomeDirection::CounterClockwise,
                speed: DEFAULT_HOMING_SPEED,
                threshold: DEFAULT_HOMING_THRESHOLD,
                phase: HomingPhase::Coarse,
                reached_speed: false,
                fine_speed: 0.0,
                homed: false,
            },
            encoder: EncoderFeedback::default(),
        }
    }

    /// Advances the state machine. Call from the step timer interrupt with
    /// the current timer value, and again right before issuing commands so
    /// that they are timed from the present.
    pub fn on_timer(&mut self, now: u32) -> Tick {
        self.now = now;
        let step = self.profile.poll(now);
        self.supervise();

        let supervision = now.wrapping_add(self.supervision_ticks);
        let wake_at = match self.profile.next_step_due() {
            Some(due) if (due.wrapping_sub(supervision) as i32) < 0 => due,
            _ => supervision,
        };
        Tick { step, wake_at }
    }

    pub fn state(&self) -> MotorState {
        self.state
    }

    /// Whether the driver's power stage should be enabled.
    pub fn outputs_enabled(&self) -> bool {
        self.state != MotorState::Off
    }

    pub fn set_state(&mut self, state: MotorState) {
        match state {
            // Cutting the power stops the motor whatever the profile says.
            MotorState::Off => self.profile.halt(),
            MotorState::IdleOn => self.profile.set_goal(Goal::Stop, self.now),
            MotorState::Position => self.profile.set_goal(Goal::stop_at(self.target), self.now),
            MotorState::Velocity => self
                .profile
                .set_goal(Goal::Velocity(self.target_velocity as f32), self.now),
            MotorState::VelocityStep => {
                if self.state != MotorState::VelocityStep {
                    self.segment = None;
                }
            }
            MotorState::Home => {
                self.homing.phase = HomingPhase::Coarse;
                self.homing.reached_speed = false;
                self.homing.homed = false;
                let velocity = self.homing.sign() * self.homing.speed as f32;
                self.profile.set_goal(Goal::Velocity(velocity), self.now);
            }
        }
        self.state = state;
        self.supervise();
    }

    /// Moves the multi-stage modes along.
    fn supervise(&mut self) {
        match self.state {
            MotorState::Off | MotorState::IdleOn | MotorState::Velocity => {}
            MotorState::Position => {
                if self.profile.is_stopped() {
                    self.state = MotorState::IdleOn;
                }
            }
            MotorState::VelocityStep => self.supervise_path(),
            MotorState::Home => self.supervise_homing(),
        }
    }

    fn supervise_path(&mut self) {
        let position = self.profile.position();
        if self.segment.is_some_and(|segment| segment.end != position) {
            return;
        }
        // At the end of a segment, or not started yet: take the next one
        // that actually goes somewhere.
        self.segment = None;
        while let Some(next) = self.path.pop_front() {
            let end = next.end(position);
            if end != position && next.speed != 0 {
                self.segment = Some(ActiveSegment {
                    start: position,
                    end,
                    speed: next.speed as f32,
                });
                self.aim_at_segment();
                return;
            }
        }
        // Out of segments. The last one brakes to a stop at its end.
        if self.profile.is_stopped() {
            self.state = MotorState::IdleOn;
            self.path_finished = true;
        }
    }

    /// Points the profile at the end of the active path segment.
    fn aim_at_segment(&mut self) {
        let Some(segment) = self.segment else {
            return;
        };
        // Carry speed into the next segment if it continues the same way;
        // otherwise stop at the end of this one.
        let exit_speed = self.path.front().map_or(0.0, |next| {
            let continues = (next.end(segment.end) > segment.end) == (segment.end > segment.start);
            let turns_back = next.end(segment.end) == segment.end;
            if continues && !turns_back {
                segment.speed.min(next.speed as f32)
            } else {
                0.0
            }
        });
        self.profile.set_goal(
            Goal::Position {
                target: segment.end,
                cruise_speed: segment.speed,
                exit_speed,
            },
            self.now,
        );
    }

    fn supervise_homing(&mut self) {
        let sign = self.homing.sign();
        // Shaft speed towards the end stop.
        let approach_speed = self.encoder.velocity_deg_per_s * sign;

        match self.homing.phase {
            HomingPhase::Coarse => {
                let commanded = (self.homing.speed as f32).min(self.profile.limits().max_speed);
                if self.shaft_has_stalled(approach_speed, commanded) {
                    // The shaft is against the end stop: there is nothing
                    // to brake.
                    self.profile.halt();
                    let back_off = -(sign as i32) * HOMING_BACKOFF_STEPS;
                    let target = self.profile.position().wrapping_add(back_off);
                    self.profile.set_goal(Goal::stop_at(target), self.now);
                    self.homing.phase = HomingPhase::BackOff;
                }
            }
            HomingPhase::BackOff => {
                if self.profile.is_stopped() && fabsf(self.encoder.velocity_deg_per_s) < 1.0 {
                    // Slow enough for the encoder to sample every half step.
                    self.homing.fine_speed = fabsf(self.encoder.update_rate_hz) / 2.0;
                    self.homing.phase = HomingPhase::Fine;
                    self.homing.reached_speed = false;
                    self.profile
                        .set_goal(Goal::Velocity(sign * self.homing.fine_speed), self.now);
                }
            }
            HomingPhase::Fine => {
                if self.shaft_has_stalled(approach_speed, self.homing.fine_speed * 0.6) {
                    self.profile.halt();
                    self.homing.homed = true;
                    self.state = MotorState::IdleOn;
                }
            }
        }
    }

    /// Whether the shaft, having been up to speed, now turns at less than
    /// half of `commanded` steps per second.
    fn shaft_has_stalled(&mut self, approach_speed: f32, commanded: f32) -> bool {
        let stall_limit = commanded * DEGREES_PER_STEP * 0.5;
        let moving = fabsf(approach_speed) > stall_limit;
        // Bouncing back off the end stop also counts as having arrived.
        if moving || approach_speed < -2.0 {
            self.homing.reached_speed = true;
        }
        self.homing.reached_speed && !moving
    }

    pub fn set_encoder_feedback(&mut self, feedback: EncoderFeedback) {
        self.encoder = feedback;
    }

    /// Steps per second.
    pub fn max_speed(&self) -> u32 {
        self.profile.limits().max_speed as u32
    }

    pub fn set_max_speed(&mut self, steps_per_s: u32) {
        let limits = Limits {
            max_speed: steps_per_s as f32,
            ..self.profile.limits()
        };
        self.profile.set_limits(limits, self.now);
    }

    /// Steps per second squared.
    pub fn acceleration(&self) -> u32 {
        self.profile.limits().acceleration as u32
    }

    /// Zero is ignored: the motor could never start or stop.
    pub fn set_acceleration(&mut self, steps_per_s2: u32) {
        if steps_per_s2 == 0 {
            return;
        }
        let limits = Limits {
            acceleration: steps_per_s2 as f32,
            ..self.profile.limits()
        };
        self.profile.set_limits(limits, self.now);
    }

    pub fn position_degrees(&self) -> f64 {
        steps_to_degrees(self.profile.position())
    }

    /// Redefines the current position. A moving motor is stopped dead.
    pub fn set_position_degrees(&mut self, degrees: f64) {
        self.profile.halt();
        self.target = degrees_to_steps(degrees);
        self.profile.set_position(self.target);
        self.segment = None;
        if self.state != MotorState::Off {
            self.state = MotorState::IdleOn;
        }
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
        self.target = target;
        self.set_state(MotorState::Position);
    }

    /// Steps per second.
    pub fn target_velocity(&self) -> f64 {
        self.target_velocity
    }

    /// Accelerates to a constant velocity in steps per second, limited to
    /// the maximum speed.
    pub fn set_target_velocity(&mut self, steps_per_s: f64) {
        self.target_velocity = steps_per_s;
        self.set_state(MotorState::Velocity);
    }

    /// Queues a path segment: travel at `velocity` steps per second until
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
        self.path.push_back(segment).map_err(|_| PathFull)?;
        if self.state == MotorState::VelocityStep && self.path.len() == 1 {
            // The segment being travelled was going to stop at its end; it
            // may now be able to run on into this one.
            self.aim_at_segment();
        }
        Ok(())
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

    /// One tick per microsecond.
    const TICK_HZ: u32 = 1_000_000;

    /// Drives `motion` the way the step timer interrupt does, returning the
    /// net number of steps taken.
    fn run_for(motion: &mut Motion, now: &mut u32, duration: u32) -> i32 {
        let end = *now + duration;
        let mut steps = 0;
        loop {
            let tick = motion.on_timer(*now);
            steps += match tick.step {
                Some(Direction::Clockwise) => 1,
                Some(Direction::CounterClockwise) => -1,
                None => 0,
            };
            // Never asks to sleep past the next supervision run.
            assert!(tick.wake_at.wrapping_sub(*now) <= TICK_HZ / SUPERVISION_HZ);
            if tick.wake_at >= end {
                *now = end;
                return steps;
            }
            *now = (*now).max(tick.wake_at);
        }
    }

    fn position_steps(motion: &Motion) -> i32 {
        degrees_to_steps(motion.position_degrees())
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
        let mut motion = Motion::new(TICK_HZ);
        assert_eq!(motion.state(), MotorState::Off);
        assert!(!motion.outputs_enabled());
        assert_eq!(
            motion.on_timer(5_000),
            Tick {
                step: None,
                wake_at: 6_000
            }
        );
    }

    #[test]
    fn position_move_ends_idle_at_the_target() {
        let mut motion = Motion::new(TICK_HZ);
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
    fn a_move_to_the_current_position_goes_straight_to_idle() {
        let mut motion = Motion::new(TICK_HZ);
        motion.move_to_degrees(0.0);
        assert_eq!(motion.state(), MotorState::IdleOn);
    }

    #[test]
    fn relative_moves_accumulate_on_the_target() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion.move_by_degrees(45.0);
        motion.move_by_degrees(45.0);
        assert_eq!(motion.target_degrees(), 90.0);
        assert_eq!(run_for(&mut motion, &mut now, 2_000_000), 400);
    }

    #[test]
    fn velocity_mode_ramps_up_then_holds_the_rate() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion.set_target_velocity(-400.0);
        assert_eq!(motion.state(), MotorState::Velocity);

        // 0.125 s to accelerate, covering half the distance of full speed.
        let ramp = run_for(&mut motion, &mut now, 125_000);
        assert!((-27..=-23).contains(&ramp), "{ramp} steps");
        let steady = run_for(&mut motion, &mut now, 1_000_000);
        assert!((-401..=-399).contains(&steady), "{steady} steps");
    }

    #[test]
    fn velocity_is_limited_to_the_maximum_speed() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion.set_target_velocity(10_000.0);
        assert_eq!(motion.target_velocity(), 10_000.0);
        run_for(&mut motion, &mut now, 1_000_000);
        let steps = run_for(&mut motion, &mut now, 1_000_000);
        assert!((799..=801).contains(&steps), "{steps} steps");
    }

    #[test]
    fn idle_brakes_to_a_stop_and_off_stops_dead() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion.set_target_velocity(800.0);
        run_for(&mut motion, &mut now, 1_000_000);

        motion.set_state(MotorState::IdleOn);
        let braking = run_for(&mut motion, &mut now, 1_000_000);
        assert!((98..=101).contains(&braking), "{braking} steps");
        assert_eq!(run_for(&mut motion, &mut now, 100_000), 0);

        motion.set_target_velocity(800.0);
        run_for(&mut motion, &mut now, 1_000_000);
        motion.set_state(MotorState::Off);
        assert!(!motion.outputs_enabled());
        assert_eq!(run_for(&mut motion, &mut now, 100_000), 0);
    }

    #[test]
    fn redefining_the_position_stops_and_idles() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion.move_to_degrees(360.0);
        run_for(&mut motion, &mut now, 500_000);

        motion.set_position_degrees(10.0);
        assert_eq!(motion.state(), MotorState::IdleOn);
        assert_eq!(motion.position_degrees(), 9.9);
        assert_eq!(motion.target_degrees(), 9.9);
        assert_eq!(run_for(&mut motion, &mut now, 500_000), 0);
    }

    #[test]
    fn plays_back_a_path_and_stops_at_its_end() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion
            .add_path_segment(400, 200, PositionMode::Absolute)
            .unwrap();
        motion
            .add_path_segment(-800, -300, PositionMode::Relative)
            .unwrap();
        motion.set_state(MotorState::VelocityStep);

        // Out to 200...
        let mut peak = 0;
        while motion.state() == MotorState::VelocityStep {
            run_for(&mut motion, &mut now, 1_000);
            peak = peak.max(position_steps(&motion));
            assert!(now < 5_000_000, "path did not finish");
            if !motion.take_path_finished() {
                continue;
            }
            break;
        }
        // ...and back 300, without overshooting either end.
        assert_eq!(peak, 200);
        assert_eq!(position_steps(&motion), -100);
        assert_eq!(motion.state(), MotorState::IdleOn);
        assert!(!motion.take_path_finished());
        assert_eq!(run_for(&mut motion, &mut now, 100_000), 0);
    }

    #[test]
    fn path_segments_in_the_same_direction_run_into_each_other() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        for _ in 0..4 {
            motion
                .add_path_segment(400, 200, PositionMode::Relative)
                .unwrap();
        }
        motion.set_state(MotorState::VelocityStep);

        // 0.125 s to accelerate; from then on a steady 400 steps/s across
        // the segment boundaries at 200, 400 and 600.
        run_for(&mut motion, &mut now, 200_000);
        let steps = run_for(&mut motion, &mut now, 1_500_000);
        assert!((599..=601).contains(&steps), "{steps} steps");

        run_for(&mut motion, &mut now, 1_000_000);
        assert_eq!(position_steps(&motion), 800);
        assert!(motion.take_path_finished());
    }

    #[test]
    fn a_segment_added_in_time_keeps_the_path_moving() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion
            .add_path_segment(400, 400, PositionMode::Relative)
            .unwrap();
        motion.set_state(MotorState::VelocityStep);
        run_for(&mut motion, &mut now, 500_000);

        motion
            .add_path_segment(400, 400, PositionMode::Relative)
            .unwrap();
        // Runs through 400 at full speed instead of stopping there.
        let steps = run_for(&mut motion, &mut now, 1_000_000);
        assert!((399..=401).contains(&steps), "{steps} steps");
    }

    #[test]
    fn empty_and_zero_segments_are_skipped() {
        let mut motion = Motion::new(TICK_HZ);
        let mut now = 0;
        motion
            .add_path_segment(0, 500, PositionMode::Relative)
            .unwrap();
        motion
            .add_path_segment(400, 0, PositionMode::Relative)
            .unwrap();
        motion
            .add_path_segment(400, 40, PositionMode::Relative)
            .unwrap();
        motion.set_state(MotorState::VelocityStep);
        assert_eq!(run_for(&mut motion, &mut now, 1_000_000), 40);
        assert!(motion.take_path_finished());
    }

    #[test]
    fn an_empty_path_finishes_immediately() {
        let mut motion = Motion::new(TICK_HZ);
        motion.set_state(MotorState::VelocityStep);
        assert_eq!(motion.state(), MotorState::IdleOn);
        assert!(motion.take_path_finished());
    }

    #[test]
    fn path_queue_is_bounded() {
        let mut motion = Motion::new(TICK_HZ);
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
        let mut motion = Motion::new(TICK_HZ);
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
        // At most the one step that was already due is still taken.
        motion.set_encoder_feedback(moving(0.0));
        assert!(run_for(&mut motion, &mut now, 100_000) >= -1);

        assert!(motion.position_degrees() < position);
        assert_eq!(motion.state(), MotorState::IdleOn);
        assert!(motion.is_homed());
    }
}
