//! Execution of host requests.

use core::fmt::Debug;

use axis_core::protocol::{MotorState, ParseError, Reply, Request, Status, FIRMWARE_VERSION};
use log::warn;

use crate::axis::Axis;
use crate::clock::Delay;

/// The request was understood but could not be carried out.
struct Failed;

fn failed(error: impl Debug) -> Failed {
    warn!("Command failed: {:?}", error);
    Failed
}

impl Axis {
    /// Executes a request and builds the reply to send back: the requested
    /// value for "get" requests, an ACK for everything else.
    pub fn handle(&mut self, request: Request) -> Reply {
        let message_type = request.message_type();
        match self.execute(request) {
            Ok(Some(reply)) => reply,
            Ok(None) => Reply::ack(message_type, Status::Success),
            Err(Failed) => Reply::ack(message_type, Status::Error),
        }
    }

    /// Returns the value a "get" request asked for, if any.
    fn execute(&mut self, request: Request) -> Result<Option<Reply>, Failed> {
        use Request as R;

        let message_type = request.message_type();
        let value = |body: &[u8]| Some(Reply::new(message_type, body));

        Ok(match request {
            R::GetVersion => value(&FIRMWARE_VERSION),

            // Settings
            R::SetI2cAddress(address) => {
                self.settings.settings_mut().set_i2c_address(address);
                None
            }
            R::GetI2cAddress => value(&[self.settings.settings().i2c_address()]),
            R::SetEthernetAddress(address) => {
                self.settings.settings_mut().set_ethernet_address(address);
                None
            }
            R::GetEthernetAddress => {
                value(&self.settings.settings().ethernet_address().to_le_bytes())
            }
            R::SetEthernetPort(port) => {
                self.settings.settings_mut().set_ethernet_port(port as u16);
                None
            }
            R::GetEthernetPort => {
                value(&u32::from(self.settings.settings().ethernet_port()).to_le_bytes())
            }
            R::GetMacAddress => value(&self.mac_address),
            R::SaveConfiguration => {
                self.settings
                    .save(&mut self.eeprom, &mut self.i2c, &mut Delay)
                    .map_err(failed)?;
                None
            }

            // Status pixel
            R::SetLedColor(color) => {
                self.pixel.set_color(color);
                None
            }
            R::GetLedColor => {
                let color = self.pixel.color();
                value(&[color.r, color.g, color.b])
            }
            R::AddLedStep { duration_ms, color } => {
                self.pixel.push(color, duration_ms).map_err(failed)?;
                None
            }

            // Homing
            R::SetHomeDirection(direction) => {
                self.motor
                    .motion(|motion| motion.set_home_direction(direction));
                None
            }
            R::GetHomeDirection => {
                value(&[self.motor.motion(|motion| motion.home_direction()) as u8])
            }
            R::SetHomeThreshold(threshold) => {
                self.motor
                    .set_home_threshold(threshold as u16)
                    .map_err(failed)?;
                None
            }
            R::GetHomeThreshold => {
                let threshold = self.motor.motion(|motion| motion.home_threshold());
                value(&u32::from(threshold).to_le_bytes())
            }
            R::SetHomeSpeed(speed) => {
                self.motor.motion(|motion| motion.set_homing_speed(speed));
                None
            }
            R::GetHomeSpeed => value(
                &self
                    .motor
                    .motion(|motion| motion.homing_speed())
                    .to_le_bytes(),
            ),
            R::GetHomedState => value(&[self.motor.motion(|motion| motion.is_homed()) as u8]),
            R::Home => {
                // Homing watches the encoder for the shaft stalling; without
                // one it would push against the end stop forever.
                if !self.has_encoder() {
                    return Err(failed("homing needs the encoder"));
                }
                self.motor.set_state(MotorState::Home).map_err(failed)?;
                None
            }

            // Motion
            R::SetMotorState(state) => {
                if state == MotorState::Home && !self.has_encoder() {
                    return Err(failed("homing needs the encoder"));
                }
                self.motor.set_state(state).map_err(failed)?;
                None
            }
            R::GetMotorState => value(&[self.motor.motion(|motion| motion.state()) as u8]),
            R::SetMotorBrake(brake) => {
                self.motor.set_brake(brake).map_err(failed)?;
                None
            }
            R::GetMotorBrake => value(&[self.motor.brake() as u8]),
            R::SetMaxSpeed(speed) => {
                self.motor.motion(|motion| motion.set_max_speed(speed));
                None
            }
            R::GetMaxSpeed => value(&self.motor.motion(|motion| motion.max_speed()).to_le_bytes()),
            R::SetAcceleration(acceleration) => {
                self.motor
                    .motion(|motion| motion.set_acceleration(acceleration));
                None
            }
            R::GetAcceleration => value(
                &self
                    .motor
                    .motion(|motion| motion.acceleration())
                    .to_le_bytes(),
            ),
            R::SetCurrentPosition(degrees) => {
                self.motor
                    .motion(|motion| motion.set_position_degrees(degrees));
                None
            }
            R::GetCurrentPosition => value(
                &self
                    .motor
                    .motion(|motion| motion.position_degrees())
                    .to_le_bytes(),
            ),
            R::SetTargetPosition(degrees) => {
                self.motor.move_to(degrees).map_err(failed)?;
                None
            }
            R::GetTargetPosition => value(
                &self
                    .motor
                    .motion(|motion| motion.target_degrees())
                    .to_le_bytes(),
            ),
            R::SetRelativeTargetPosition(degrees) => {
                self.motor.move_by(degrees).map_err(failed)?;
                None
            }
            R::SetVelocity(velocity) => {
                self.motor
                    .motion(|motion| motion.set_target_velocity(velocity));
                None
            }
            R::GetVelocity => value(
                &self
                    .motor
                    .motion(|motion| motion.target_velocity())
                    .to_le_bytes(),
            ),
            R::SetVelocityAndSteps {
                velocity,
                steps,
                mode,
            } => {
                self.motor
                    .motion(|motion| motion.add_path_segment(velocity, steps, mode))
                    .map_err(failed)?;
                None
            }
            R::StartPath => {
                self.motor
                    .set_state(MotorState::VelocityStep)
                    .map_err(failed)?;
                None
            }
        })
    }
}

/// Builds the reply to a message that could not be decoded, if it deserves
/// one.
pub fn reject(error: ParseError) -> Option<Reply> {
    warn!("Message rejected: {:?}", error);
    error
        .rejected_type()
        .map(|message_type| Reply::ack_raw(message_type, Status::InvalidCommand))
}
