//! The motor: step generation in the step timer interrupt, plus the driver
//! settings that go with each mode.

use core::cell::RefCell;

use atsamd_hal::{
    ehal::digital::OutputPin,
    fugit::ExtU32,
    pac::{self, interrupt},
    timer_traits::InterruptDrivenTimer,
};
use axis_core::motion::{EncoderFeedback, Motion};
use axis_core::protocol::{MotorBrake, MotorState};
use axis_core::stepper::Direction;
use cortex_m::interrupt::Mutex;
use log::info;

use crate::board::{MotorPins, StepTimer, CPU_HZ};
use crate::clock;
use crate::tmc2209::{self, Tmc2209};

/// Width of a step pulse; the TMC2209 needs at least 100 ns.
const STEP_PULSE_CYCLES: u32 = CPU_HZ / 1_000_000;
/// Time the driver needs after power-up before it accepts commands.
const DRIVER_STARTUP_MS: u32 = 20;

/// Everything the step timer interrupt works with.
struct StepGenerator {
    motion: Motion,
    /// Number of timer interrupts handled, for the self test.
    ticks: u32,
    pins: MotorPins,
    timer: StepTimer,
}

static STEP_GENERATOR: Mutex<RefCell<Option<StepGenerator>>> = Mutex::new(RefCell::new(None));

impl StepGenerator {
    fn on_timer(&mut self) {
        self.ticks = self.ticks.wrapping_add(1);
        let tick = self.motion.on_timer(clock::micros());
        if let Some(direction) = tick.step {
            self.step(direction);
        }
        self.sleep(tick.wake_in_us);
    }

    fn step(&mut self, direction: Direction) {
        // The direction input is wired so that low is clockwise.
        let _ = match direction {
            Direction::Clockwise => self.pins.direction.set_low(),
            Direction::CounterClockwise => self.pins.direction.set_high(),
        };
        let _ = self.pins.step.set_high();
        cortex_m::asm::delay(STEP_PULSE_CYCLES);
        let _ = self.pins.step.set_low();
    }

    /// Schedules the next timer interrupt.
    fn sleep(&mut self, duration_us: u32) {
        // Restarting the timer resets it, which also clears the pending
        // interrupt and disables the interrupt again.
        self.timer.start(duration_us.micros());
        self.timer.enable_interrupt();
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
            generator.on_timer();
        }
    });
}

/// Handle to the motor for the main loop.
pub struct Motor {
    driver: Tmc2209,
    /// Whether the driver accepted its configuration at start-up.
    configured: Result<(), tmc2209::Error>,
    brake: MotorBrake,
}

/// The step timer interrupt does not fire at the idle polling rate. The
/// details are for the log.
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub struct StepTimerFault {
    pub ticks: u32,
    pub window_ms: u32,
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
            motion: Motion::new(),
            ticks: 0,
            pins,
            timer,
        };
        generator.update_enable_pin();
        generator.sleep(1_000);
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

    /// Checks that the step timer interrupt runs. Only meaningful while the
    /// motor is stopped, when the interrupt polls once a millisecond.
    pub fn check_step_timer(&mut self) -> Result<(), StepTimerFault> {
        const WINDOW_MS: u32 = 20;

        let ticks = || {
            cortex_m::interrupt::free(|cs| {
                let generator = STEP_GENERATOR.borrow(cs).borrow();
                generator.as_ref().map_or(0, |generator| generator.ticks)
            })
        };
        let start_ticks = ticks();
        let start_ms = clock::millis();
        while clock::millis().wrapping_sub(start_ms) < WINDOW_MS {}
        let ticks = ticks().wrapping_sub(start_ticks);
        info!(
            "Step timer: {} interrupts in {} ms (expected about {})",
            ticks, WINDOW_MS, WINDOW_MS
        );

        if (WINDOW_MS / 2..=WINDOW_MS * 2).contains(&ticks) {
            Ok(())
        } else {
            Err(StepTimerFault {
                ticks,
                window_ms: WINDOW_MS,
            })
        }
    }

    /// Reads or changes the motion state, with the step timer held off.
    pub fn motion<R>(&mut self, f: impl FnOnce(&mut Motion) -> R) -> R {
        cortex_m::interrupt::free(|cs| {
            let mut generator = STEP_GENERATOR.borrow(cs).borrow_mut();
            let generator = generator.as_mut().expect("motor not started");
            let result = f(&mut generator.motion);
            generator.update_enable_pin();
            result
        })
    }

    pub fn set_state(&mut self, state: MotorState) -> Result<(), tmc2209::Error> {
        // Moving needs the full run current; a path also drops the hold
        // current so the motor runs cool between segments.
        let driver_result = match state {
            MotorState::Position | MotorState::Velocity => self.driver.set_run_current(100),
            MotorState::VelocityStep => self.driver.set_currents(100, 0, 0),
            MotorState::Off | MotorState::IdleOn | MotorState::Home => Ok(()),
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
