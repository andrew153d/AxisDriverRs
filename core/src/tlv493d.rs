//! Driver for the Infineon TLV493D-A1B6 3D magnetic sensor.
//!
//! The sensor sits under a diametrically magnetised magnet on the motor
//! shaft, so the direction of the field in the X/Y plane is the shaft angle.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use libm::{atan2f, sqrtf};

/// Bus address when SDA is high at power-up.
pub const ADDRESS: u8 = 0x5E;

/// Time the sensor needs after power-up before it can be addressed.
const STARTUP_DELAY_MS: u32 = 40;
/// Time between measurements in master controlled mode.
pub const MEASUREMENT_PERIOD_MS: u32 = 10;

/// Flux density of one sensor count.
pub const MILLITESLA_PER_COUNT: f32 = 0.098;
/// Weakest X/Y field that still gives a steady angle, in sensor counts
/// (about 5 mT).
pub const MIN_PLANAR_STRENGTH: f32 = 50.0;
/// Readings this close to the end of the 12-bit range are clipped.
const SATURATION_COUNTS: u16 = 2040;

/// Layout of the four write registers.
mod write {
    pub const MODE1: usize = 1;
    pub const RESERVED2: usize = 2;
    pub const MODE2: usize = 3;

    pub const MODE1_PARITY: u8 = 0x80;
    pub const MODE1_RESERVED: u8 = 0x18;
    pub const MODE1_FAST: u8 = 0x02;
    pub const MODE1_LOW_POWER: u8 = 0x01;

    pub const MODE2_TEMPERATURE_OFF: u8 = 0x80;
    pub const MODE2_LOW_POWER_PERIOD: u8 = 0x40;
    pub const MODE2_PARITY_TEST: u8 = 0x20;
    pub const MODE2_RESERVED: u8 = 0x1F;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error<E> {
    Bus(E),
    /// The sensor was read in the middle of a conversion.
    ConversionRunning,
}

/// One reading of the magnetic field, in 12-bit sensor counts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Field {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Field {
    /// Decodes the first six read registers.
    fn from_registers(regs: &[u8; 7]) -> Self {
        let axis = |high: u8, low_nibble: u8| {
            // Assemble the 12 bits at the top of an i16, then shift back down
            // to sign-extend.
            (i16::from_be_bytes([high, low_nibble << 4])) >> 4
        };
        Self {
            x: axis(regs[0], regs[4] >> 4),
            y: axis(regs[1], regs[4] & 0x0F),
            z: axis(regs[2], regs[5] & 0x0F),
        }
    }

    /// Strength of the field in the X/Y plane, in sensor counts.
    pub fn planar_strength(&self) -> f32 {
        let (x, y) = (f32::from(self.x), f32::from(self.y));
        sqrtf(x * x + y * y)
    }

    /// Whether the field is strong enough to measure the shaft angle from,
    /// without driving any axis into saturation. Fails when the magnet is
    /// missing or badly placed.
    pub fn is_usable(&self) -> bool {
        let saturated = [self.x, self.y, self.z]
            .iter()
            .any(|axis| axis.unsigned_abs() >= SATURATION_COUNTS);
        self.planar_strength() >= MIN_PLANAR_STRENGTH && !saturated
    }

    /// Direction of the field in the X/Y plane, in `-180.0..=180.0` degrees.
    pub fn azimuth_degrees(&self) -> f32 {
        atan2f(f32::from(self.y), f32::from(self.x)).to_degrees()
    }
}

pub struct Tlv493d;

impl Tlv493d {
    /// Resets the sensor and puts it into master controlled mode, in which
    /// every readout triggers the next measurement.
    pub fn new<I: I2c>(i2c: &mut I, delay: &mut impl DelayNs) -> Result<Self, I::Error> {
        delay.delay_ms(STARTUP_DELAY_MS);

        // Reset via the general call address; the data byte selects the bus
        // address. Sensors do not acknowledge the general call, so there is
        // no result to check.
        let _ = i2c.write(0x00, &[0xFF]);

        // The reserved write bits must be programmed with their factory
        // values, which can be read back from registers 7 to 9.
        let mut factory = [0; 10];
        i2c.read(ADDRESS, &mut factory)?;

        let mut config = [0u8; 4];
        config[write::MODE1] =
            (factory[7] & write::MODE1_RESERVED) | write::MODE1_FAST | write::MODE1_LOW_POWER;
        config[write::RESERVED2] = factory[8];
        config[write::MODE2] = (factory[9] & write::MODE2_RESERVED)
            | write::MODE2_TEMPERATURE_OFF
            | write::MODE2_LOW_POWER_PERIOD
            | write::MODE2_PARITY_TEST;
        set_parity(&mut config);
        i2c.write(ADDRESS, &config)?;

        Ok(Self)
    }

    /// Reads the latest measurement and starts the next one. Call no more
    /// often than every [`MEASUREMENT_PERIOD_MS`].
    pub fn read<I: I2c>(&mut self, i2c: &mut I) -> Result<Field, Error<I::Error>> {
        let mut regs = [0; 7];
        i2c.read(ADDRESS, &mut regs).map_err(Error::Bus)?;

        // The channel bits are zero only when a conversion has completed.
        if regs[3] & 0x03 != 0 {
            return Err(Error::ConversionRunning);
        }
        Ok(Field::from_registers(&regs))
    }
}

/// Sets the parity bit so that the whole configuration has odd parity.
fn set_parity(config: &mut [u8; 4]) {
    config[write::MODE1] &= !write::MODE1_PARITY;
    let ones: u32 = config.iter().map(|byte| byte.count_ones()).sum();
    if ones.is_multiple_of(2) {
        config[write::MODE1] |= write::MODE1_PARITY;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_signed_12_bit_axes() {
        //                 Bx    By    Bz    T/frm  Bx/By lo  Bz lo  T
        let regs = [0x12, 0xFF, 0x80, 0x00, 0x3F, 0x05, 0x00];
        assert_eq!(
            Field::from_registers(&regs),
            Field {
                x: 0x123,
                y: -1,
                z: -2048 + 5,
            }
        );
    }

    #[test]
    fn azimuth_follows_the_field_direction() {
        let field = |x, y| Field { x, y, z: 0 };
        assert_eq!(field(100, 0).azimuth_degrees(), 0.0);
        assert_eq!(field(0, 100).azimuth_degrees(), 90.0);
        assert_eq!(field(0, -100).azimuth_degrees(), -90.0);
        assert_eq!(field(-100, 0).azimuth_degrees(), 180.0);
    }

    #[test]
    fn judges_field_strength() {
        let field = |x, y, z| Field { x, y, z };
        assert!(field(300, -400, 20).is_usable());
        assert_eq!(field(300, -400, 20).planar_strength(), 500.0);
        // No magnet: just noise.
        assert!(!field(3, -2, 5).is_usable());
        // A strong field along Z only says nothing about the angle.
        assert!(!field(3, -2, 900).is_usable());
        assert!(!field(2047, 100, 0).is_usable());
        assert!(!field(300, 400, -2048).is_usable());
    }

    #[test]
    fn configuration_has_odd_parity() {
        for config in [
            [0, 0x03, 0x00, 0xE0],
            [0, 0x1B, 0x55, 0xFF],
            [0, 0x80, 0, 0],
        ] {
            let mut with_parity = config;
            set_parity(&mut with_parity);
            let ones: u32 = with_parity.iter().map(|byte| byte.count_ones()).sum();
            assert_eq!(ones % 2, 1);
            assert_eq!(with_parity[1] & 0x7F, config[1] & 0x7F);
        }
    }
}
