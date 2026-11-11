//! The motor: step generation in the step timer interrupt, plus the driver
//! settings that go with each mode.

use core::cell::RefCell;

use atsamd_hal::{
    ehal::{delay::DelayNs, digital::OutputPin},
    pac::{self, interrupt},
};
use axis_core::motion::{EncoderFeedback, Motion};
use axis_core::profile::Direction;
use axis_core::types::{MotorBrake, MotorState};
use cortex_m::interrupt::Mutex;
use log::info;

use crate::board::{MotorPins, CPU_HZ};
use crate::clock;
use crate::step_timer::{StepTimer, TICKS_PER_SECOND};
use crate::tmc2209::{self, Tmc2209};

const CYCLES_PER_MICRO: u32 = CPU_HZ / 1_000_000;
/// Width of a step pulse; the TMC2209 needs at least 100 ns.
const STEP_PULSE_CYCLES: u32 = CYCLES_PER_MICRO;
/// Time the direction input has to be stable for before a step, with
/// plenty of margin over the 20 ns the TMC2209 asks for.
const DIRECTION_SETUP_CYCLES: u32 = CYCLES_PER_MICRO / 4;
/// Time the driver needs after power-up before it accepts commands.
const DRIVER_STARTUP_MS: u32 = 20;

/// Everything the step timer interrupt works with.
struct StepGenerator {
    motion: Motion,
    pins: MotorPins,
    timer: StepTimer,
    /// Direction the direction pin is set to, once known.
    direction: Option<Direction>,
    /// Number of timer interrupts handled, for the self test.
    interrupts: u32,
}

static STEP_GENERATOR: Mutex<RefCell<Option<StepGenerator>>> = Mutex::new(RefCell::new(None));

impl StepGenerator {
    /// Takes a step if one is due and sets the alarm for the next event.
    /// Harmless to call at any time, any number of times.
    fn service(&mut self) {
        let tick = self.motion.on_timer(self.timer.now());
        if let Some(direction) = tick.step {
            self.step(direction);
        }
        self.timer.set_alarm(tick.wake_at);
    }

    fn step(&mut self, direction: Direction) {
        if self.direction != Some(direction) {
            // The direction input is wired so that low is clockwise.
            let _ = match direction {
                Direction::Clockwise => self.pins.direction.set_low(),
                Direction::CounterClockwise => self.pins.direction.set_high(),
            };
            self.direction = Some(direction);
            cortex_m::asm::delay(DIRECTION_SETUP_CYCLES);
        }
        let _ = self.pins.step.set_high();
        cortex_m::asm::delay(STEP_PULSE_CYCLES);
        let _ = self.pins.step.set_low();
    }

    fn update_enable_pin(&mut self) {
        let _ = if self.motion.outputs_enabled() {
            self.pins.enable.set_low()
        } else {
            self.pins.enable.set_high()
        };
    }
}

#[interrupt]
fn TC2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(generator) = STEP_GENERATOR.borrow(cs).borrow_mut().as_mut() {
            generator.timer.clear_alarm();
            generator.interrupts = generator.interrupts.wrapping_add(1);
            generator.service();
        }
    });
}

/// Runs `f` on the step generator with the step timer interrupt held off.
fn with_generator<R>(f: impl FnOnce(&mut StepGenerator) -> R) -> R {
    cortex_m::interrupt::free(|cs| {
        let mut generator = STEP_GENERATOR.borrow(cs).borrow_mut();
        f(generator.as_mut().expect("motor not started"))
    })
}

/// Handle to the motor for the main loop.
pub struct Motor {
    driver: Tmc2209,
    /// Whether the driver accepted its configuration at start-up.
    configured: Result<(), tmc2209::Error>,
    brake: MotorBrake,
}

/// Why the step timer failed its self test. The details are for the log.
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub enum StepTimerFault {
    /// The counter does not advance at its nominal rate.
    WrongRate { ticks: u32, expected: u32 },
    /// The alarm interrupt does not fire at the supervision rate.
    InterruptRate { interrupts: u32, expected: u32 },
}

impl Motor {
    /// Configures the driver and starts the step timer.
    ///
    /// # Panics
    ///
    /// Panics if called twice.
    pub fn start(pins: MotorPins, timer: StepTimer, mut driver: Tmc2209) -> Self {
        // The driver is powered from the motor supply, which may be absent.
        // Carry on regardless so that the rest of the board stays usable.
        while clock::millis() < DRIVER_STARTUP_MS {}
        let configured = driver.configure();

        let mut generator = StepGenerator {
            motion: Motion::new(TICKS_PER_SECOND),
            pins,
            timer,
            direction: None,
            interrupts: 0,
        };
        generator.update_enable_pin();
        generator.service();
        cortex_m::interrupt::free(|cs| {
            let previous = STEP_GENERATOR.borrow(cs).replace(Some(generator));
            assert!(previous.is_none(), "motor started twice");
        });
        // SAFETY: the handler only touches STEP_GENERATOR, inside a critical
        // section, and nothing relies on this interrupt staying masked.
        unsafe { pac::NVIC::unmask(interrupt::TC2) };

        Self {
            driver,
            configured,
            brake: MotorBrake::Normal,
        }
    }

    /// Checks that the driver took its configuration and is healthy.
    pub fn check_driver(&mut self) -> Result<(), tmc2209::Error> {
        self.configured?;
        self.driver.self_test()
    }

    /// Checks that the step timer counts at the right rate and that its
    /// interrupt runs. Only meaningful while the motor is stopped, when the
    /// interrupt fires once a millisecond.
    pub fn check_step_timer(&mut self) -> Result<(), StepTimerFault> {
        const WINDOW_MS: u32 = 20;
        const EXPECTED_TICKS: u32 = TICKS_PER_SECOND / 1_000 * WINDOW_MS;

        let sample = || with_generator(|generator| (generator.timer.now(), generator.interrupts));
        let (start_ticks, start_interrupts) = sample();
        clock::Delay.delay_ms(WINDOW_MS);
        let (end_ticks, end_interrupts) = sample();
        let ticks = end_ticks.wrapping_sub(start_ticks);
        let interrupts = end_interrupts.wrapping_sub(start_interrupts);
        info!(
            "Step timer: {} ticks and {} interrupts in {} ms (expected about {} and {})",
            ticks, interrupts, WINDOW_MS, EXPECTED_TICKS, WINDOW_MS
        );

        // Both clocks derive from the same oscillator, so they agree closely.
        if ticks.abs_diff(EXPECTED_TICKS) > EXPECTED_TICKS / 50 {
            Err(StepTimerFault::WrongRate {
                ticks,
                expected: EXPECTED_TICKS,
            })
        } else if !(WINDOW_MS / 2..=WINDOW_MS * 2).contains(&interrupts) {
            Err(StepTimerFault::InterruptRate {
                interrupts,
                expected: WINDOW_MS,
            })
        } else {
            Ok(())
        }
    }

    /// Reads or changes the motion state. Changes take effect immediately.
    pub fn motion<R>(&mut self, f: impl FnOnce(&mut Motion) -> R) -> R {
        with_generator(|generator| {
            // Bring the motion state up to the present, so that the command
            // is timed from now, and act on its outcome straight away.
            generator.service();
            let result = f(&mut generator.motion);
            generator.update_enable_pin();
            generator.service();
            result
        })
    }

    /// Returns `true` once after a queued path has run to completion.
    pub fn take_path_finished(&mut self) -> bool {
        with_generator(|generator| generator.motion.take_path_finished())
    }

    pub fn set_state(&mut self, state: MotorState) -> Result<(), tmc2209::Error> {
        // Moving needs the full run current; a path also drops the hold
        // current so the motor runs cool between segments.
        let driver_result = match state {
            MotorState::Position | MotorState::Velocity => self.driver.set_run_current(100),
            MotorState::Path => self.driver.set_currents(100, 0, 0),
            MotorState::Off | MotorState::Idle | MotorState::Homing => Ok(()),
        };
        self.motion(|motion| motion.set_state(state));
        driver_result
    }

    /// Starts a move to an absolute position in degrees.
    pub fn move_to(&mut self, degrees: f64) -> Result<(), tmc2209::Error> {
        let driver_result = self.driver.set_run_current(100);
        self.motion(|motion| motion.move_to_degrees(degrees));
        driver_result
    }

    /// Starts a move relative to the previous target, in degrees.
    pub fn move_by(&mut self, degrees: f64) -> Result<(), tmc2209::Error> {
        let driver_result = self.driver.set_run_current(100);
        self.motion(|motion| motion.move_by_degrees(degrees));
        driver_result
    }

    /// Accelerates to a constant velocity in steps per second.
    pub fn set_velocity(&mut self, steps_per_s: f64) -> Result<(), tmc2209::Error> {
        let driver_result = self.driver.set_run_current(100);
        self.motion(|motion| motion.set_target_velocity(steps_per_s));
        driver_result
    }

    pub fn brake(&self) -> MotorBrake {
        self.brake
    }

    pub fn set_brake(&mut self, brake: MotorBrake) -> Result<(), tmc2209::Error> {
        self.brake = brake;
        self.driver.set_brake(brake)
    }

    pub fn set_home_threshold(&mut self, threshold: u16) -> Result<(), tmc2209::Error> {
        self.motion(|motion| motion.set_home_threshold(threshold));
        self.driver.set_stallguard_threshold(threshold as u8)
    }

    pub fn set_encoder_feedback(&mut self, feedback: EncoderFeedback) {
        self.motion(|motion| motion.set_encoder_feedback(feedback));
    }
}
