//! Configuration of the TMC2209 stepper driver over its single-wire UART.
//!
//! TX and RX share one wire, so every byte sent is also received back and
//! has to be discarded before a reply can be read.

use atsamd_hal::{ehal_nb::serial, nb};
use axis_core::protocol::MotorBrake;
use log::{debug, info};
use tmc2209::{
    data::{MicroStepResolution, StandstillMode},
    reg::{
        self, CHOPCONF, DRV_STATUS, GCONF, GSTAT, IFCNT, IHOLD_IRUN, IOIN, PWMCONF, SGTHRS,
        TCOOLTHRS, TPOWERDOWN,
    },
    ReadableRegister, Reader, WritableRegister,
};

use crate::board::TmcUart;
use crate::clock;

/// UART address selected by the MS1/MS2 pins.
const ADDRESS: u8 = 0;
const ECHO_TIMEOUT_US: u32 = 10_000;
const RESPONSE_TIMEOUT_US: u32 = 100_000;
/// Idle time to leave on the wire between datagrams. The driver keeps hold
/// of the wire for a moment after replying; sending into that corrupts the
/// next datagram.
const BUS_IDLE_US: u32 = 500;

/// Value of `IOIN.version` for the TMC2209.
const CHIP_VERSION: u8 = 0x21;
/// Number of registers [`Tmc2209::configure`] writes.
const CONFIGURATION_WRITES: u8 = 6;

/// Range of the 5-bit IRUN/IHOLD current settings used for 0 to 100 percent.
const CURRENT_SETTING_MIN: u32 = 10;
const CURRENT_SETTING_MAX: u32 = 31;
const HOLD_DELAY_MAX: u32 = 15;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Uart,
    /// The bytes sent did not appear on the shared wire.
    NoEcho,
    /// The driver did not answer; it may be unpowered.
    NoResponse,
    BadCrc,
    UnexpectedRegister,
    /// The driver did not count all the register writes it was sent.
    WritesLost,
    /// The chip answering is not a TMC2209.
    UnexpectedVersion(u8),
    /// The charge pump is undervoltage: the motor supply is missing or low.
    MotorSupplyLow,
    Overtemperature,
    /// A motor output is shorted to ground or to the supply.
    ShortCircuit,
    /// The driver shut down for a reason it does not report.
    Shutdown,
}

pub struct Tmc2209 {
    uart: TmcUart,
    /// When the last datagram, or the reply to it, ended.
    last_activity_us: u32,
    // Copies of the write-only registers that are modified at run time.
    driver_current: IHOLD_IRUN,
    pwm_config: PWMCONF,
}

impl Tmc2209 {
    pub fn new(uart: TmcUart) -> Self {
        Self {
            uart,
            last_activity_us: clock::micros(),
            driver_current: IHOLD_IRUN::default(),
            pwm_config: PWMCONF::default(),
        }
    }

    /// Programs the driver's registers and checks that it accepted them.
    pub fn configure(&mut self) -> Result<(), Error> {
        // The driver counts the writes it accepts.
        let writes_before = self.read::<IFCNT>()?.get();

        // Take all settings from registers rather than pins: UART control,
        // internal current reference, StealthChop.
        let mut global_config = GCONF::default();
        global_config.set_i_scale_analog(false);
        global_config.set_internal_rsense(true);
        global_config.set_en_spread_cycle(false);
        global_config.set_pdn_disable(true);
        global_config.set_mstep_reg_select(true);
        global_config.set_multistep_filt(true);

        let mut power_down = TPOWERDOWN::default();
        power_down.set(100);

        let mut cool_threshold = TCOOLTHRS::default();
        cool_threshold.set(0x000F_FFFF);

        let mut chopper_config = CHOPCONF::default();
        chopper_config.set_vsense(true);
        chopper_config.set_toff(5);
        chopper_config.set_tbl(2);
        chopper_config.set_hstrt(4);
        chopper_config.set_hend(0);
        chopper_config.set_mres(MicroStepResolution::from_microsteps(8));

        self.driver_current.set_irun(15);
        self.driver_current.set_ihold(10);
        self.driver_current.set_ihold_delay(10);

        self.pwm_config.set_pwm_autoscale(true);
        self.pwm_config.set_pwm_autograd(true);
        self.pwm_config.set_pwm_freq(3);

        self.write(global_config)?;
        self.write(power_down)?;
        self.write(cool_threshold)?;
        self.write(chopper_config)?;
        self.write(self.driver_current)?;
        self.write(self.pwm_config)?;

        let writes_after = self.read::<IFCNT>()?.get();
        let writes = writes_after.wrapping_sub(writes_before);
        info!(
            "TMC2209: {} of {} configuration writes accepted (IFCNT {} -> {})",
            writes, CONFIGURATION_WRITES, writes_before, writes_after
        );
        if writes != CONFIGURATION_WRITES {
            return Err(Error::WritesLost);
        }
        Ok(())
    }

    /// Checks that the chip is a TMC2209, is powered and reports no fault.
    pub fn self_test(&mut self) -> Result<(), Error> {
        let inputs: IOIN = self.read()?;
        let version = inputs.version();
        info!(
            "TMC2209: version {:#04x} (expected {:#04x})",
            version, CHIP_VERSION
        );
        info!(
            "TMC2209: pins ENN={} MS1={} MS2={} SPREAD={} DIAG={} STEP={} DIR={}",
            u8::from(inputs.enn()),
            u8::from(inputs.ms1()),
            u8::from(inputs.ms2()),
            u8::from(inputs.spread_en()),
            u8::from(inputs.diag()),
            u8::from(inputs.step()),
            u8::from(inputs.dir()),
        );
        if version != CHIP_VERSION {
            return Err(Error::UnexpectedVersion(version));
        }

        let status: GSTAT = self.read()?;
        info!(
            "TMC2209: status reset={} driver_error={} charge_pump_undervoltage={}",
            status.reset(),
            status.drv_err(),
            status.uv_cp()
        );
        let driver: DRV_STATUS = self.read()?;
        info!(
            "TMC2209: overtemperature warning={} shutdown={}, above 120C={} 143C={} 150C={} 157C={}",
            driver.otpw(),
            driver.ot(),
            driver.t120(),
            driver.t143(),
            driver.t150(),
            driver.t157()
        );
        info!(
            "TMC2209: short to ground A={} B={}, low-side short A={} B={}, open load A={} B={}",
            driver.s2ga(),
            driver.s2gb(),
            driver.s2vsa(),
            driver.s2vsb(),
            driver.ola(),
            driver.olb()
        );
        info!(
            "TMC2209: standstill={} stealthchop={} current scale={}/31",
            driver.stst(),
            driver.stealth(),
            driver.cs_actual()
        );

        if status.uv_cp() {
            return Err(Error::MotorSupplyLow);
        }
        if status.drv_err() {
            return Err(if driver.ot() {
                Error::Overtemperature
            } else if driver.s2ga() || driver.s2gb() || driver.s2vsa() || driver.s2vsb() {
                Error::ShortCircuit
            } else {
                Error::Shutdown
            });
        }
        Ok(())
    }

    /// Sets the motor current while moving, as a percentage of the maximum.
    pub fn set_run_current(&mut self, percent: u8) -> Result<(), Error> {
        self.driver_current.set_irun(current_setting(percent));
        self.write(self.driver_current)
    }

    /// Sets the run current, the standstill current and how slowly the
    /// current drops to the standstill value, all in percent.
    pub fn set_currents(
        &mut self,
        run_percent: u8,
        hold_percent: u8,
        hold_delay_percent: u8,
    ) -> Result<(), Error> {
        self.driver_current.set_irun(current_setting(run_percent));
        self.driver_current.set_ihold(current_setting(hold_percent));
        self.driver_current
            .set_ihold_delay(scale_percent(hold_delay_percent, 0, HOLD_DELAY_MAX));
        self.write(self.driver_current)
    }

    /// Sets what the driver does with the motor at standstill.
    pub fn set_brake(&mut self, brake: MotorBrake) -> Result<(), Error> {
        self.pwm_config.set_freewheel(match brake {
            MotorBrake::Normal => StandstillMode::Normal,
            MotorBrake::Freewheeling => StandstillMode::Freewheeling,
            MotorBrake::StrongBraking => StandstillMode::StrongBraking,
            MotorBrake::Braking => StandstillMode::Braking,
        });
        self.write(self.pwm_config)
    }

    pub fn set_stallguard_threshold(&mut self, threshold: u8) -> Result<(), Error> {
        let mut register = SGTHRS::default();
        register.set(threshold);
        self.write(register)
    }

    fn write<R: WritableRegister>(&mut self, register: R) -> Result<(), Error> {
        self.send(tmc2209::write_request(ADDRESS, register).bytes())
    }

    fn read<R: ReadableRegister>(&mut self) -> Result<R, Error> {
        self.send(tmc2209::read_request::<R>(ADDRESS).bytes())?;
        let result = self.receive();
        self.last_activity_us = clock::micros();
        result
    }

    /// Waits for the reply to a read request.
    fn receive<R: ReadableRegister>(&mut self) -> Result<R, Error> {
        let mut reader = Reader::default();
        let start = clock::micros();
        while clock::micros().wrapping_sub(start) < RESPONSE_TIMEOUT_US {
            let Some(byte) = self.read_byte()? else {
                continue;
            };
            if let (_, Some(response)) = reader.read_response(&[byte]) {
                if !response.crc_is_valid() {
                    return Err(Error::BadCrc);
                }
                return response
                    .register()
                    .map_err(|reg::UnknownAddress| Error::UnexpectedRegister);
            }
        }
        Err(Error::NoResponse)
    }

    /// Sends a datagram, once the wire has been idle for long enough.
    fn send(&mut self, datagram: &[u8]) -> Result<(), Error> {
        while clock::micros().wrapping_sub(self.last_activity_us) < BUS_IDLE_US {}
        self.uart.flush_rx_buffer();
        let result = self.transmit(datagram);
        self.last_activity_us = clock::micros();
        result
    }

    /// Transmits a datagram and consumes its echo.
    fn transmit(&mut self, datagram: &[u8]) -> Result<(), Error> {
        let start = clock::micros();
        let mut sent = 0;
        let mut echoed = 0;
        while echoed < datagram.len() {
            if sent < datagram.len() {
                match serial::Write::write(&mut self.uart, datagram[sent]) {
                    Ok(()) => sent += 1,
                    Err(nb::Error::WouldBlock) => {}
                    Err(nb::Error::Other(_)) => return Err(Error::Uart),
                }
            }
            if echoed < sent {
                match self.read_byte()? {
                    Some(byte) if byte == datagram[echoed] => echoed += 1,
                    Some(_) => return Err(Error::NoEcho),
                    None => {}
                }
            }
            if clock::micros().wrapping_sub(start) > ECHO_TIMEOUT_US {
                return Err(Error::NoEcho);
            }
        }
        Ok(())
    }

    fn read_byte(&mut self) -> Result<Option<u8>, Error> {
        match serial::Read::read(&mut self.uart) {
            Ok(byte) => Ok(Some(byte)),
            Err(nb::Error::WouldBlock) => Ok(None),
            Err(nb::Error::Other(error)) => {
                debug!("TMC2209 UART receive error: {:?}", error);
                Err(Error::Uart)
            }
        }
    }
}

/// Maps 0..=100 percent onto `min..=max`.
fn scale_percent(percent: u8, min: u32, max: u32) -> u8 {
    let percent = u32::from(percent.min(100));
    (min + percent * (max - min) / 100) as u8
}

fn current_setting(percent: u8) -> u8 {
    scale_percent(percent, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX)
}
