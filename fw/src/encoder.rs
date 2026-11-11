//! The magnetic shaft encoder.

use atsamd_hal::{ehal::delay::DelayNs, sercom::i2c};
use axis_core::encoder::ShaftTracker;
use axis_core::motion::EncoderFeedback;
use axis_core::tlv493d::{
    self, Field, Tlv493d, MEASUREMENT_PERIOD_MS, MILLITESLA_PER_COUNT, MIN_PLANAR_STRENGTH,
};
use log::{debug, info};

use crate::board::I2cBus;
use crate::clock::{self, Delay};

/// Why the encoder failed its self test. The details are for the log.
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub enum Fault {
    Sensor(tlv493d::Error<i2c::Error>),
    /// The sensor works but does not see a usable field from the magnet.
    Magnet(Field),
}

pub struct Encoder {
    sensor: Tlv493d,
    tracker: ShaftTracker,
    last_sample_ms: u32,
}

impl Encoder {
    /// Sets up the sensor and checks that it sees the magnet.
    pub fn start(i2c: &mut I2cBus) -> Result<Self, Fault> {
        let mut sensor = Tlv493d::new(i2c, &mut Delay)
            .map_err(|error| Fault::Sensor(tlv493d::Error::Bus(error)))?;
        info!(
            "Encoder: TLV493D found at address {:#04x}",
            tlv493d::ADDRESS
        );
        // The first readout only triggers the first measurement.
        let _ = sensor.read(i2c);
        Delay.delay_ms(MEASUREMENT_PERIOD_MS);
        let field = sensor.read(i2c).map_err(Fault::Sensor)?;
        info!(
            "Encoder: field x={} y={} z={} counts, {} mT in the sensing plane (at least {} mT needed)",
            field.x,
            field.y,
            field.z,
            field.planar_strength() * MILLITESLA_PER_COUNT,
            MIN_PLANAR_STRENGTH * MILLITESLA_PER_COUNT,
        );
        info!("Encoder: shaft angle {} degrees", field.azimuth_degrees());
        if !field.is_usable() {
            return Err(Fault::Magnet(field));
        }

        let mut tracker = ShaftTracker::new(
            field.azimuth_degrees(),
            clock::micros(),
            1_000.0 / MEASUREMENT_PERIOD_MS as f32,
        );
        // The motor's commanded position starts at zero, so count the shaft
        // angle from here too. The two then only differ by how far the shaft
        // is from where it was told to go.
        tracker.set_position_degrees(0.0);

        Ok(Self {
            sensor,
            tracker,
            last_sample_ms: clock::millis(),
        })
    }

    /// Multi-turn shaft position in degrees, counted from the same zero as
    /// the motor's commanded position.
    pub fn position_degrees(&self) -> f32 {
        self.tracker.position_degrees()
    }

    /// Redefines the current shaft position.
    pub fn set_position_degrees(&mut self, degrees: f32) {
        self.tracker.set_position_degrees(degrees);
    }

    pub fn velocity_deg_per_s(&self) -> f32 {
        self.tracker.velocity_deg_per_s()
    }

    /// Samples the sensor when a new measurement is due. Returns fresh
    /// feedback for the motor whenever there is some.
    pub fn poll(&mut self, i2c: &mut I2cBus) -> Option<EncoderFeedback> {
        if !clock::every(MEASUREMENT_PERIOD_MS, &mut self.last_sample_ms) {
            return None;
        }
        match self.sensor.read(i2c) {
            Ok(field) => {
                self.tracker
                    .update(field.azimuth_degrees(), clock::micros());
                Some(self.tracker.feedback())
            }
            Err(error) => {
                debug!("Encoder read failed: {:?}", error);
                None
            }
        }
    }
}
