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
//!
//! # Closed loop control
//!
//! The driver is told to step and the shaft normally follows, to within a
//! fraction of a full step. When the load is too much for the motor the
//! rotor drops out of step and comes to rest a whole number of electrical
//! cycles (four full steps) away from where it was told to go; at speed it
//! usually stalls altogether.
//!
//! With closed loop control enabled, every encoder sample is compared with
//! the commanded position. Once the two are more than most of a cycle apart
//! the motor is stopped, the shaft is given time to come to rest, the
//! commanded position is corrected by the number of cycles that were lost,
//! and the interrupted motion is taken up again from standstill. So moves
//! end where they were told to, and a shaft that is forced away from where
//! it is held comes back.
//!
//! Corrections are always whole cycles because the rotor cannot rest
//! anywhere else, and because the stepping itself is more accurate than the
//! magnetic encoder: smaller differences are left alone.

use heapless::Deque;
use libm::{fabsf, roundf};

use crate::profile::{Direction, Goal, Limits, Profile};
use crate::types::{HomeDirection, MotorState, PositionMode};

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

/// Microsteps in one electrical cycle: four full steps. A rotor that drops
/// out of step ends up a multiple of this away.
const CYCLE_STEPS: i32 = STEPS_PER_REVOLUTION / 50;
/// How far the shaft may be from the commanded position, in steps, before
/// it counts as having lost steps. Less than a cycle, so that losing a
/// single one is noticed, but well above what load, encoder error and
/// noise add up to.
const SLIP_DETECTION_STEPS: f32 = 0.75 * CYCLE_STEPS as f32;
/// The shaft is at rest once it moves less than this many degrees between
/// encoder samples, this many samples in a row.
const REST_DEGREES: f32 = 0.3;
const REST_SAMPLES: u8 = 5;
/// Number of times in a row a motion is taken up again after losing steps.
/// After that the shaft is taken to be blocked.
const MAX_RETRIES: u8 = 3;
/// Time without losing steps after which the retries start afresh.
const RETRY_WINDOW_S: f32 = 2.0;

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

/// Shaft measurements that homing and closed loop control rely on.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EncoderFeedback {
    /// Multi-turn shaft position, counted from the same zero as the
    /// commanded position.
    pub position_degrees: f32,
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

/// What closed loop control did in response to an encoder sample.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Correction {
    /// The shaft had lost this many steps; negative if it was ahead. The
    /// commanded position has been corrected and the motion taken up again.
    Recovered { steps: i32 },
    /// The shaft kept losing steps and is taken to be blocked. The motor
    /// has been stopped.
    Stalled,
}

/// The motor has been stopped after losing steps, and is waiting for the
/// shaft to come to rest so that it can tell how many.
struct Recovery {
    /// What to carry on with afterwards.
    resume: Goal,
    /// Whether to give up instead of carrying on.
    give_up: bool,
    /// Consecutive encoder samples without the shaft moving.
    samples_at_rest: u8,
}

struct ClosedLoop {
    enabled: bool,
    /// Commanded position when the previous encoder sample arrived. Reading
    /// the sensor starts its next measurement, so this is where the shaft
    /// was meant to be when the latest sample was measured.
    sampled_position: i32,
    recovery: Option<Recovery>,
    /// Recoveries since the shaft last followed for [`RETRY_WINDOW_S`].
    retries: u8,
    /// Encoder samples since steps were last lost.
    samples_in_step: u32,
    stalled: bool,
    /// Number of times lost steps have been corrected.
    corrections: u32,
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
    closed_loop: ClosedLoop,
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
            closed_loop: ClosedLoop {
                enabled: false,
                sampled_position: 0,
                recovery: None,
                retries: 0,
                samples_in_step: 0,
                stalled: false,
                corrections: 0,
            },
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
        if self.state == MotorState::Homing && state != MotorState::Homing {
            // Pushing against the end stop loses steps on purpose.
            self.adopt_shaft_position();
        }
        // A new command gets a fresh start.
        self.closed_loop.recovery = None;
        self.closed_loop.retries = 0;
        self.closed_loop.stalled = false;

        match state {
            // Cutting the power stops the motor whatever the profile says.
            MotorState::Off => self.profile.halt(),
            MotorState::Idle => self.profile.set_goal(Goal::Stop, self.now),
            MotorState::Position => self.profile.set_goal(Goal::stop_at(self.target), self.now),
            MotorState::Velocity => self
                .profile
                .set_goal(Goal::Velocity(self.target_velocity as f32), self.now),
            MotorState::Path => {
                if self.state != MotorState::Path {
                    self.segment = None;
                }
            }
            MotorState::Homing => {
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
        if self.closed_loop.recovery.is_some() {
            // Standing still on purpose, not because the mode is done.
            return;
        }
        match self.state {
            MotorState::Off | MotorState::Idle | MotorState::Velocity => {}
            MotorState::Position => {
                if self.profile.is_stopped() {
                    self.state = MotorState::Idle;
                }
            }
            MotorState::Path => self.supervise_path(),
            MotorState::Homing => self.supervise_homing(),
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
            self.state = MotorState::Idle;
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
                    self.state = MotorState::Idle;
                    // Pushing against the end stop loses steps on purpose.
                    self.adopt_shaft_position();
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

    /// Takes in a new encoder sample, and with closed loop control enabled
    /// acts on it. Call once for every sample, as soon as it has been read.
    pub fn set_encoder_feedback(&mut self, feedback: EncoderFeedback) -> Option<Correction> {
        let shaft_travel = fabsf(feedback.position_degrees - self.encoder.position_degrees);
        self.encoder = feedback;
        let position = self.profile.position();
        let sampled_position = core::mem::replace(&mut self.closed_loop.sampled_position, position);
        if !self.closed_loop.enabled || self.state == MotorState::Homing {
            return None;
        }

        if let Some(recovery) = &mut self.closed_loop.recovery {
            if shaft_travel < REST_DEGREES {
                recovery.samples_at_rest += 1;
            } else {
                recovery.samples_at_rest = 0;
            }
            return if recovery.samples_at_rest >= REST_SAMPLES {
                self.finish_recovery()
            } else {
                None
            };
        }

        // The sample may be up to one more sample period old than assumed.
        let sample_rate = feedback.update_rate_hz.max(1.0);
        let timing_allowance = fabsf(self.profile.velocity()) / sample_rate;
        let error = self.steps_ahead_of_shaft(sampled_position);
        if fabsf(error) > SLIP_DETECTION_STEPS + timing_allowance {
            self.start_recovery();
        } else {
            self.closed_loop.samples_in_step += 1;
            if self.closed_loop.samples_in_step as f32 >= RETRY_WINDOW_S * sample_rate {
                self.closed_loop.retries = 0;
            }
        }
        None
    }

    /// How many steps `position` is ahead of where the encoder last saw the
    /// shaft.
    fn steps_ahead_of_shaft(&self, position: i32) -> f32 {
        let shaft =
            f64::from(self.encoder.position_degrees) / 360.0 * f64::from(STEPS_PER_REVOLUTION);
        (f64::from(position) - shaft) as f32
    }

    /// Stops stepping until the shaft has come to rest.
    fn start_recovery(&mut self) {
        let resume = match self.state {
            // The shaft is free to turn; just keep track of it.
            MotorState::Off => Goal::Stop,
            // Holding position: go back to it.
            MotorState::Idle if self.profile.is_stopped() => Goal::stop_at(self.profile.position()),
            _ => self.profile.goal(),
        };
        let mut give_up = false;
        if self.state != MotorState::Off {
            self.closed_loop.retries += 1;
            give_up = self.closed_loop.retries > MAX_RETRIES;
        }
        // A rotor that is out of step cannot be braked, or picked up again
        // at speed.
        self.profile.halt();
        self.closed_loop.samples_in_step = 0;
        self.closed_loop.recovery = Some(Recovery {
            resume,
            give_up,
            samples_at_rest: 0,
        });
    }

    /// Corrects the commanded position now that the shaft is at rest, and
    /// carries on with what was interrupted.
    fn finish_recovery(&mut self) -> Option<Correction> {
        let recovery = self.closed_loop.recovery.take()?;
        let steps = self.adopt_shaft_position();
        if recovery.give_up {
            self.state = MotorState::Idle;
            self.segment = None;
            self.closed_loop.stalled = true;
            return Some(Correction::Stalled);
        }
        self.profile.set_goal(recovery.resume, self.now);
        self.supervise();
        // The shaft was merely slow to follow if there is nothing to correct.
        (steps != 0).then_some(Correction::Recovered { steps })
    }

    /// Corrects the commanded position by the whole electrical cycles the
    /// shaft has fallen behind, and returns that distance in steps. Only
    /// meaningful while neither the motor nor the shaft is moving much.
    fn adopt_shaft_position(&mut self) -> i32 {
        if !self.closed_loop.enabled {
            return 0;
        }
        let position = self.profile.position();
        let cycles = roundf(self.steps_ahead_of_shaft(position) / CYCLE_STEPS as f32) as i32;
        let lost = cycles * CYCLE_STEPS;
        self.profile.set_position(position.wrapping_sub(lost));
        self.closed_loop.sampled_position = self.profile.position();
        if lost != 0 {
            self.closed_loop.corrections = self.closed_loop.corrections.wrapping_add(1);
        }
        lost
    }

    /// Whether the encoder is used to notice and make up for lost steps.
    pub fn closed_loop(&self) -> bool {
        self.closed_loop.enabled
    }

    /// Switches closed loop control on or off. Switching it on takes the
    /// shaft to be where it should be, give or take the cycles it has
    /// already lost, so nothing moves. Only enable it with a working
    /// encoder.
    pub fn set_closed_loop(&mut self, enabled: bool) {
        if enabled == self.closed_loop.enabled {
            return;
        }
        self.closed_loop.enabled = enabled;
        self.closed_loop.retries = 0;
        self.closed_loop.stalled = false;
        if enabled {
            self.adopt_shaft_position();
        } else if let Some(recovery) = self.closed_loop.recovery.take() {
            if !recovery.give_up {
                self.profile.set_goal(recovery.resume, self.now);
            }
        }
    }

    /// Whether closed loop control gave up on a blocked shaft. Cleared by
    /// the next change of state.
    pub fn is_stalled(&self) -> bool {
        self.closed_loop.stalled
    }

    /// Number of times closed loop control has corrected lost steps.
    pub fn corrections(&self) -> u32 {
        self.closed_loop.corrections
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

    /// Speed the motor is being driven at right now, in steps per second;
    /// negative is counter-clockwise.
    pub fn velocity(&self) -> f32 {
        self.profile.velocity()
    }

    pub fn position_degrees(&self) -> f64 {
        steps_to_degrees(self.profile.position())
    }

    /// Redefines the current position. A moving motor is stopped dead.
    pub fn set_position_degrees(&mut self, degrees: f64) {
        self.profile.halt();
        self.target = degrees_to_steps(degrees);
        self.profile.set_position(self.target);
        self.closed_loop.sampled_position = self.target;
        self.closed_loop.recovery = None;
        self.closed_loop.retries = 0;
        self.closed_loop.stalled = false;
        self.segment = None;
        if self.state != MotorState::Off {
            self.state = MotorState::Idle;
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

    /// Queues a path segment: travel at `speed` steps per second until
    /// `steps` is reached. Start the path with [`MotorState::Path`].
    pub fn add_path_segment(
        &mut self,
        speed: u32,
        steps: i32,
        mode: PositionMode,
    ) -> Result<(), PathFull> {
        let segment = PathSegment { mode, speed, steps };
        self.path.push_back(segment).map_err(|_| PathFull)?;
        if self.state == MotorState::Path && self.path.len() == 1 {
            // The segment being travelled was going to stop at its end; it
            // may now be able to run on into this one.
            self.aim_at_segment();
        }
        Ok(())
    }

    /// Number of path segments waiting to be travelled.
    pub fn path_queued(&self) -> usize {
        self.path.len()
    }

    /// Number of further path segments that can be queued.
    pub fn path_free(&self) -> usize {
        PATH_CAPACITY - self.path.len()
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
        assert_eq!(motion.state(), MotorState::Idle);
        assert_eq!(motion.position_degrees(), 90.0);
        assert_eq!(motion.target_degrees(), 90.0);
    }

    #[test]
    fn a_move_to_the_current_position_goes_straight_to_idle() {
        let mut motion = Motion::new(TICK_HZ);
        motion.move_to_degrees(0.0);
        assert_eq!(motion.state(), MotorState::Idle);
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

        motion.set_state(MotorState::Idle);
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
        assert_eq!(motion.state(), MotorState::Idle);
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
            .add_path_segment(800, -300, PositionMode::Relative)
            .unwrap();
        motion.set_state(MotorState::Path);

        // Out to 200...
        let mut peak = 0;
        while motion.state() == MotorState::Path {
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
        assert_eq!(motion.state(), MotorState::Idle);
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
        motion.set_state(MotorState::Path);

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
        motion.set_state(MotorState::Path);
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
        motion.set_state(MotorState::Path);
        assert_eq!(run_for(&mut motion, &mut now, 1_000_000), 40);
        assert!(motion.take_path_finished());
    }

    #[test]
    fn an_empty_path_finishes_immediately() {
        let mut motion = Motion::new(TICK_HZ);
        motion.set_state(MotorState::Path);
        assert_eq!(motion.state(), MotorState::Idle);
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
            position_degrees: 0.0,
            velocity_deg_per_s,
            update_rate_hz: 100.0,
        };
        motion.set_home_direction(HomeDirection::Clockwise);
        motion.set_state(MotorState::Homing);
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
        assert_eq!(motion.state(), MotorState::Homing);
        // At most the one step that was already due is still taken.
        motion.set_encoder_feedback(moving(0.0));
        assert!(run_for(&mut motion, &mut now, 100_000) >= -1);

        assert!(motion.position_degrees() < position);
        assert_eq!(motion.state(), MotorState::Idle);
        assert!(motion.is_homed());
    }

    /// Encoder sample period of the rig, in ticks.
    const SAMPLE_TICKS: u32 = 10_000;

    /// A motor with a shaft and an encoder on it.
    struct Rig {
        motion: Motion,
        now: u32,
        /// Where the shaft is, in steps.
        shaft: i32,
        /// Whether the shaft is held so that the motor cannot turn it.
        blocked: bool,
        /// The measurement in progress: like the sensor's, it is read out
        /// one sample period after it was taken.
        measuring: i32,
        events: Vec<Correction>,
    }

    impl Rig {
        fn new() -> Self {
            let mut motion = Motion::new(TICK_HZ);
            motion.set_closed_loop(true);
            Self {
                motion,
                now: 0,
                shaft: 0,
                blocked: false,
                measuring: 0,
                events: Vec::new(),
            }
        }

        /// Runs the motor and the encoder, returning the steps issued.
        fn run(&mut self, duration: u32) -> i32 {
            let mut steps = 0;
            for _ in 0..duration / SAMPLE_TICKS {
                let taken = run_for(&mut self.motion, &mut self.now, SAMPLE_TICKS);
                steps += taken;
                if !self.blocked {
                    self.shaft += taken;
                }
                let feedback = EncoderFeedback {
                    position_degrees: steps_to_degrees(self.measuring) as f32,
                    velocity_deg_per_s: 0.0,
                    update_rate_hz: 100.0,
                };
                self.measuring = self.shaft;
                self.events.extend(self.motion.set_encoder_feedback(feedback));
            }
            steps
        }

        /// Lets go of a blocked shaft, which drops into the nearest place
        /// the rotor can rest.
        fn release(&mut self) {
            self.blocked = false;
            let behind = position_steps(&self.motion) - self.shaft;
            let cycles = (behind as f32 / CYCLE_STEPS as f32).round() as i32;
            self.shaft = position_steps(&self.motion) - cycles * CYCLE_STEPS;
        }
    }

    #[test]
    fn open_loop_does_not_notice_lost_steps() {
        let mut rig = Rig::new();
        rig.motion.set_closed_loop(false);
        rig.motion.move_to_degrees(90.0);
        rig.run(300_000);
        rig.shaft -= 2 * CYCLE_STEPS;
        rig.run(2_000_000);

        assert_eq!(rig.motion.position_degrees(), 90.0);
        assert_eq!(rig.shaft, 400 - 2 * CYCLE_STEPS);
        assert_eq!(rig.events, []);
    }

    #[test]
    fn closed_loop_leaves_a_shaft_that_follows_alone() {
        let mut rig = Rig::new();
        rig.motion.set_max_speed(8_000);
        rig.motion.set_acceleration(40_000);
        rig.motion.move_to_degrees(3_600.0);
        // The encoder is a few steps out and the shaft lags a little.
        rig.shaft = -10;
        rig.measuring = -10;

        // 1.6 s to accelerate and brake, 1.8 s at full speed.
        assert_eq!(rig.run(2_200_000), 16_000);
        assert_eq!(rig.motion.state(), MotorState::Idle);
        assert_eq!(rig.motion.position_degrees(), 3_600.0);
        assert_eq!(rig.events, []);
        assert_eq!(rig.motion.corrections(), 0);
    }

    #[test]
    fn a_move_makes_up_for_lost_steps() {
        let mut rig = Rig::new();
        rig.motion.move_to_degrees(360.0);
        rig.run(1_000_000);
        rig.shaft -= 3 * CYCLE_STEPS;
        rig.run(3_000_000);

        assert_eq!(
            rig.events,
            [Correction::Recovered {
                steps: 3 * CYCLE_STEPS
            }]
        );
        assert_eq!(rig.shaft, 1600);
        assert_eq!(rig.motion.position_degrees(), 360.0);
        assert_eq!(rig.motion.state(), MotorState::Idle);
        assert_eq!(rig.motion.corrections(), 1);
        assert!(!rig.motion.is_stalled());
    }

    #[test]
    fn a_stalled_move_starts_again_from_standstill() {
        let mut rig = Rig::new();
        rig.motion.move_to_degrees(360.0);
        rig.run(500_000);
        rig.blocked = true;
        rig.run(100_000);
        // Stopped, rather than stepping on against the stall.
        assert_eq!(rig.run(30_000), 0);
        assert_eq!(rig.motion.velocity(), 0.0);
        assert_eq!(rig.motion.state(), MotorState::Position);

        rig.release();
        rig.run(3_000_000);
        assert_eq!(rig.shaft, 1600);
        assert_eq!(rig.motion.state(), MotorState::Idle);
        assert!(!rig.motion.is_stalled());
    }

    #[test]
    fn holds_position_against_being_forced_away() {
        let mut rig = Rig::new();
        rig.motion.move_to_degrees(90.0);
        rig.run(2_000_000);
        assert_eq!(rig.motion.state(), MotorState::Idle);

        rig.shaft += 5 * CYCLE_STEPS;
        assert_eq!(rig.run(1_000_000), -5 * CYCLE_STEPS);
        assert_eq!(rig.shaft, 400);
        assert_eq!(rig.motion.position_degrees(), 90.0);
        assert_eq!(rig.motion.state(), MotorState::Idle);
        assert_eq!(
            rig.events,
            [Correction::Recovered {
                steps: -5 * CYCLE_STEPS
            }]
        );
    }

    #[test]
    fn differences_smaller_than_half_a_cycle_are_not_corrected() {
        let mut rig = Rig::new();
        rig.motion.set_state(MotorState::Idle);
        rig.shaft = CYCLE_STEPS / 2 - 2;
        assert_eq!(rig.run(1_000_000), 0);
        assert_eq!(rig.motion.position_degrees(), 0.0);
        assert_eq!(rig.events, []);
    }

    #[test]
    fn velocity_mode_picks_up_speed_again_after_a_stall() {
        let mut rig = Rig::new();
        rig.motion.set_target_velocity(800.0);
        rig.run(1_000_000);
        rig.blocked = true;
        rig.run(200_000);
        rig.release();
        rig.run(1_000_000);

        assert_eq!(rig.motion.state(), MotorState::Velocity);
        let steps = rig.run(1_000_000);
        assert!((799..=801).contains(&steps), "{steps} steps");
        // The commanded position has been brought back to the shaft.
        assert_eq!(position_steps(&rig.motion), rig.shaft);
        assert_eq!(rig.motion.corrections(), 1);
    }

    #[test]
    fn a_path_carries_on_to_its_end_after_lost_steps() {
        let mut rig = Rig::new();
        for _ in 0..2 {
            rig.motion
                .add_path_segment(400, 400, PositionMode::Relative)
                .unwrap();
        }
        rig.motion.set_state(MotorState::Path);
        rig.run(500_000);
        rig.shaft -= CYCLE_STEPS;
        rig.run(4_000_000);

        assert_eq!(rig.shaft, 800);
        assert_eq!(position_steps(&rig.motion), 800);
        assert!(rig.motion.take_path_finished());
        assert_eq!(rig.motion.state(), MotorState::Idle);
    }

    #[test]
    fn gives_up_on_a_blocked_shaft() {
        let mut rig = Rig::new();
        rig.motion.move_to_degrees(360.0);
        rig.blocked = true;
        rig.run(3_000_000);

        assert_eq!(rig.events.last(), Some(&Correction::Stalled));
        assert_eq!(rig.events.len(), 1 + MAX_RETRIES as usize);
        assert!(rig.motion.is_stalled());
        assert_eq!(rig.motion.state(), MotorState::Idle);
        // The commanded position is where the shaft is, to within the
        // half cycle a held rotor can be off by, not where the motor tried
        // to go; and nothing more is attempted.
        assert!(position_steps(&rig.motion).abs() <= CYCLE_STEPS / 2);
        assert_eq!(rig.run(1_000_000), 0);

        // The next command tries again.
        rig.release();
        rig.motion.move_to_degrees(360.0);
        assert!(!rig.motion.is_stalled());
        rig.run(3_000_000);
        assert_eq!(rig.shaft, 1600);
    }

    #[test]
    fn follows_the_shaft_while_switched_off() {
        let mut rig = Rig::new();
        rig.shaft = 10 * CYCLE_STEPS + 3;
        assert_eq!(rig.run(1_000_000), 0);
        assert_eq!(rig.motion.state(), MotorState::Off);
        assert_eq!(position_steps(&rig.motion), 10 * CYCLE_STEPS);

        // Turning it by hand any number of times is not a stall.
        for turn in 0..10 {
            rig.shaft = turn * CYCLE_STEPS;
            rig.run(200_000);
        }
        assert!(!rig.motion.is_stalled());

        // Switching on holds the shaft where it is.
        rig.motion.set_state(MotorState::Idle);
        assert_eq!(rig.run(1_000_000), 0);
        assert_eq!(position_steps(&rig.motion), 9 * CYCLE_STEPS);
    }

    #[test]
    fn enabling_closed_loop_does_not_move_the_motor() {
        let mut rig = Rig::new();
        rig.motion.set_closed_loop(false);
        rig.motion.set_state(MotorState::Idle);
        rig.shaft = -4 * CYCLE_STEPS;
        rig.run(100_000);

        rig.motion.set_closed_loop(true);
        assert!(rig.motion.closed_loop());
        assert_eq!(position_steps(&rig.motion), -4 * CYCLE_STEPS);
        assert_eq!(rig.run(1_000_000), 0);
        assert_eq!(rig.events, []);
    }

    #[test]
    fn homing_leaves_the_position_where_the_shaft_is() {
        let mut rig = Rig::new();
        let feedback = |shaft: i32, velocity_deg_per_s| EncoderFeedback {
            position_degrees: steps_to_degrees(shaft) as f32,
            velocity_deg_per_s,
            update_rate_hz: 100.0,
        };
        rig.motion.set_home_direction(HomeDirection::Clockwise);
        rig.motion.set_state(MotorState::Homing);

        // The shaft gets 20 steps towards the end stop and no further,
        // whatever the motor does.
        let (motion, now) = (&mut rig.motion, &mut rig.now);
        motion.set_encoder_feedback(feedback(-10, -22.5));
        run_for(motion, now, 500_000);
        motion.set_encoder_feedback(feedback(-20, 5.0));
        run_for(motion, now, 1_000_000);
        motion.set_encoder_feedback(feedback(30, 0.0));
        run_for(motion, now, 100_000);
        motion.set_encoder_feedback(feedback(0, -11.25));
        run_for(motion, now, 2_000_000);
        assert_eq!(motion.state(), MotorState::Homing);
        motion.set_encoder_feedback(feedback(-20, 0.0));
        run_for(motion, now, 100_000);

        assert!(motion.is_homed());
        assert_eq!(motion.state(), MotorState::Idle);
        // Within half a cycle of the shaft, and it stays there.
        assert!((position_steps(motion) + 20).abs() <= CYCLE_STEPS / 2);
        rig.shaft = -20;
        rig.measuring = -20;
        assert_eq!(rig.run(1_000_000), 0);
        assert_eq!(rig.events, []);
    }
}
