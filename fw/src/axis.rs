//! The device as a whole: everything a host command can act on.

use atsamd_hal::ehal::digital::StatefulOutputPin;
use axis_core::eeprom::{Eeprom, EEPROM_SIZE};
use axis_core::motion::Correction;
use axis_core::selftest::{Check, Report};
use axis_core::settings::{Settings, SettingsStore};
use axis_core::types::Rgb;
use core::fmt::Debug;
use log::{error, info, warn};

use crate::board::{I2cBus, StatusLed};
use crate::clock;
use crate::encoder::Encoder;
use crate::led::StatusPixel;
use crate::motor::Motor;

const HEARTBEAT_PERIOD_MS: u32 = 1_000;
/// The self test result is shown as one flash per check, green or red.
const SELF_TEST_FLASH_MS: u32 = 250;
const SELF_TEST_GAP_MS: u32 = 150;
const PASS_COLOR: Rgb = Rgb::new(0x00, 0x30, 0x00);
const FAIL_COLOR: Rgb = Rgb::new(0x60, 0x00, 0x00);
/// Shown when a queued path has finished.
const PATH_FINISHED_FLASH_MS: u32 = 500;
/// Shown when closed loop control gives up on a blocked shaft.
const STALLED_FLASH_MS: u32 = 1_000;

pub struct Axis {
    pub motor: Motor,
    pub pixel: StatusPixel,
    pub i2c: I2cBus,
    pub eeprom: Eeprom,
    pub settings: SettingsStore,
    pub mac_address: [u8; 6],
    pub serial_number: [u8; 16],
    /// Outcome of the power-on self test.
    pub self_test: Report,
    /// Whether the Ethernet link is up; `None` if the HAT is not in use.
    pub ethernet_link: Option<bool>,
    /// `None` if the sensor failed the self test.
    encoder: Option<Encoder>,
    status_led: StatusLed,
    last_heartbeat_ms: u32,
}

impl Axis {
    /// Brings up the devices on the I2C bus and runs the power-on self
    /// test.
    pub fn start(
        mut motor: Motor,
        mut pixel: StatusPixel,
        mut i2c: I2cBus,
        status_led: StatusLed,
    ) -> Self {
        let mut report = Report::new();
        info!("Self test starting");

        report.record(Check::StepTimer, passed(motor.check_step_timer()));
        report.record(Check::MotorDriver, passed(motor.check_driver()));

        let encoder = Encoder::start(&mut i2c);
        report.record(Check::Encoder, passed(encoder.as_ref().map(|_| ())));

        let mut eeprom = Eeprom::new();
        let settings = SettingsStore::load(&mut eeprom, &mut i2c);
        if let Ok(settings) = &settings {
            let settings = settings.settings();
            info!(
                "EEPROM: settings read; Ethernet address {:#010x} port {}, I2C address {:#04x}",
                settings.ethernet_address(),
                settings.ethernet_port(),
                settings.i2c_address()
            );
        }
        let mac_address = eeprom.mac_address(&mut i2c);
        // An erased or absent identity chip reads as all ones or all zeros.
        let mac_is_valid = mac_address.is_ok_and(|mac| mac != [0x00; 6] && mac != [0xFF; 6]);
        if let Ok(mac) = mac_address {
            info!(
                "EEPROM: MAC address {:02X?} ({})",
                mac,
                if mac_is_valid { "valid" } else { "blank" }
            );
        }
        // Informational only: nothing depends on the serial number.
        let serial_number = eeprom.serial_number(&mut i2c).unwrap_or_else(|error| {
            info!("EEPROM: serial number not read: {:?}", error);
            [0; 16]
        });
        info!("EEPROM: serial number {:02X?}", serial_number);
        report.record(
            Check::Eeprom,
            passed(settings.as_ref().map(|_| ())) && passed(mac_address) && mac_is_valid,
        );

        show_report(&report, &mut pixel);

        // Carry on with whatever works, so that the board can still be
        // talked to and diagnosed.
        let settings = settings
            .unwrap_or_else(|_| SettingsStore::unsaved(Settings::from_image([0xFF; EEPROM_SIZE])));
        let mac_address = mac_address.unwrap_or_default();
        let encoder = encoder.ok();
        if encoder.is_some() {
            motor.motion(|motion| motion.set_closed_loop(true));
            info!("Closed loop control enabled");
        } else {
            info!("Closed loop control disabled: no encoder");
        }

        Self {
            motor,
            pixel,
            i2c,
            eeprom,
            settings,
            mac_address,
            serial_number,
            self_test: report,
            ethernet_link: None,
            encoder,
            status_led,
            last_heartbeat_ms: 0,
        }
    }

    /// Whether the shaft encoder passed its self test and is in use.
    pub fn has_encoder(&self) -> bool {
        self.encoder.is_some()
    }

    /// Shaft position in degrees and velocity in degrees per second, if the
    /// encoder is in use.
    pub fn encoder_reading(&self) -> Option<(f32, f32)> {
        self.encoder
            .as_ref()
            .map(|encoder| (encoder.position_degrees(), encoder.velocity_deg_per_s()))
    }

    /// Redefines the motor's current position, and the encoder's with it so
    /// that the two keep the same zero. A moving motor is stopped dead.
    pub fn set_position(&mut self, degrees: f64) {
        let position = self.motor.motion(|motion| {
            motion.set_position_degrees(degrees);
            // Rounded to a whole step.
            motion.position_degrees()
        });
        if let Some(encoder) = &mut self.encoder {
            encoder.set_position_degrees(position as f32);
        }
    }

    /// Runs the background work. Call continuously from the main loop.
    pub fn poll(&mut self) {
        if let Some(encoder) = &mut self.encoder {
            if let Some(feedback) = encoder.poll(&mut self.i2c) {
                match self.motor.set_encoder_feedback(feedback) {
                    Some(Correction::Recovered { steps }) => {
                        warn!("Closed loop: made up for {} lost steps", steps);
                    }
                    Some(Correction::Stalled) => {
                        error!("Closed loop: the shaft is blocked; motor stopped");
                        self.pixel.flash(Rgb::RED, STALLED_FLASH_MS);
                    }
                    None => {}
                }
            }
        }

        if self.motor.take_path_finished() {
            self.pixel.flash(Rgb::PURPLE, PATH_FINISHED_FLASH_MS);
        }
        self.pixel.poll();

        if clock::every(HEARTBEAT_PERIOD_MS, &mut self.last_heartbeat_ms) {
            let _ = self.status_led.toggle();
        }
    }
}

/// Logs why a check failed and reduces its outcome to pass or fail.
fn passed<T>(result: Result<T, impl Debug>) -> bool {
    match result {
        Ok(_) => true,
        Err(fault) => {
            error!("Self test fault: {:?}", fault);
            false
        }
    }
}

/// Logs the self test report and queues it on the status pixel.
fn show_report(report: &Report, pixel: &mut StatusPixel) {
    for (check, passed) in report.results() {
        info!(
            "Self test {:?}: {}",
            check,
            if passed { "pass" } else { "FAIL" }
        );
        let color = if passed { PASS_COLOR } else { FAIL_COLOR };
        // The queue is empty at start-up, so these always fit.
        let _ = pixel.push(color, SELF_TEST_FLASH_MS);
        let _ = pixel.push(Rgb::BLACK, SELF_TEST_GAP_MS);
    }
    if report.all_passed() {
        info!("Self test passed");
    } else {
        let failed = report.results().filter(|&(_, passed)| !passed).count();
        error!(
            "Self test failed: {} of {} checks",
            failed,
            Check::ALL.len()
        );
    }
}
