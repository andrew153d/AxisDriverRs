//! Execution of host requests.

use core::fmt::Debug;

use axis_core::framing::FrameError;
use axis_core::proto::{
    self, AddPath, Config, HomingSettings, Info, LedStep, Log, MotionSettings, Request,
    Request_::Command, Response, Response_::Result as Outcome, SelfTest, Status, FIRMWARE_VERSION,
};
use axis_core::selftest::Check;
use axis_core::types::{MotorState, PositionMode};
use log::warn;

use crate::axis::Axis;
use crate::clock::{self, Delay};
use crate::logger;

/// Why a request was not carried out.
enum Failure {
    /// It was understood but could not be done.
    Failed,
    /// It held a value that makes no sense.
    Invalid,
}

fn failed(error: impl Debug) -> Failure {
    warn!("Command failed: {:?}", error);
    Failure::Failed
}

fn invalid(error: impl Debug) -> Failure {
    warn!("Command invalid: {:?}", error);
    Failure::Invalid
}

/// Builds the response to a message that could not be decoded, if it
/// deserves one.
pub fn reject(error: FrameError) -> Option<Response> {
    warn!("Message rejected: {:?}", error);
    // A damaged frame may not even have been meant as one; an intact frame
    // with nonsense in it gets an answer.
    (error == FrameError::Malformed).then(|| Response {
        id: 0,
        status: Status::Invalid,
        result: None,
    })
}

impl Axis {
    /// Executes a request and builds the response to send back.
    pub fn handle(&mut self, request: &Request) -> Response {
        let Some(command) = &request.command else {
            warn!("Request {} holds no known command", request.id);
            return Response {
                id: request.id,
                status: Status::Invalid,
                result: None,
            };
        };
        let status = match self.execute(command) {
            Ok(()) => Status::Ok,
            Err(Failure::Failed) => Status::Error,
            Err(Failure::Invalid) => Status::Invalid,
        };
        Response {
            id: request.id,
            status,
            result: Some(self.outcome(command)),
        }
    }

    fn execute(&mut self, command: &Command) -> Result<(), Failure> {
        match command {
            // Requests for state are answered by `outcome`.
            Command::GetInfo(_)
            | Command::GetLog(_)
            | Command::GetConfig(_)
            | Command::GetLed(_)
            | Command::GetMotor(_) => Ok(()),

            Command::SetConfig(config) => self.apply_config(config),
            Command::SaveConfig(_) => self
                .settings
                .save(&mut self.eeprom, &mut self.i2c, &mut Delay)
                .map_err(failed),

            Command::SetLed(color) => {
                self.pixel.set_color((*color).into());
                Ok(())
            }
            Command::AddLedStep(LedStep {
                color, duration_ms, ..
            }) => self
                .pixel
                .push((*color).into(), *duration_ms)
                .map_err(failed),

            Command::SetMotorState(request) => {
                let state = MotorState::try_from(request.state).map_err(invalid)?;
                self.set_motor_state(state)
            }
            Command::MoveTo(request) => self.motor.move_to(request.degrees).map_err(failed),
            Command::MoveBy(request) => self.motor.move_by(request.degrees).map_err(failed),
            Command::SetVelocity(request) => {
                self.motor.set_velocity(request.steps_per_s).map_err(failed)
            }
            Command::SetPosition(request) => {
                self.set_position(request.degrees);
                Ok(())
            }
            Command::SetMotion(settings) => self.apply_motion_settings(settings),
            Command::SetHoming(settings) => self.apply_homing_settings(settings),
            Command::Home(_) => self.set_motor_state(MotorState::Homing),
            Command::AddPath(path) => self.add_path(path),
            Command::StartPath(_) => self.set_motor_state(MotorState::Path),
        }
    }

    /// The state to report back after `command`.
    fn outcome(&mut self, command: &Command) -> Outcome {
        match command {
            Command::GetInfo(_) => Outcome::Info(self.info()),
            Command::GetLog(_) => {
                let mut log = Log::default();
                let capacity = log.text.capacity();
                // Cannot fail: the buffer is empty and resized to capacity.
                let _ = log.text.resize(capacity, 0);
                let length = logger::read(&mut log.text);
                log.text.truncate(length);
                Outcome::Log(log)
            }
            Command::GetConfig(_) | Command::SetConfig(_) | Command::SaveConfig(_) => {
                Outcome::Config(self.config())
            }
            Command::GetLed(_) | Command::SetLed(_) | Command::AddLedStep(_) => {
                Outcome::Led(self.pixel.color().into())
            }
            Command::GetMotor(_)
            | Command::SetMotorState(_)
            | Command::MoveTo(_)
            | Command::MoveBy(_)
            | Command::SetVelocity(_)
            | Command::SetPosition(_)
            | Command::SetMotion(_)
            | Command::SetHoming(_)
            | Command::Home(_)
            | Command::AddPath(_)
            | Command::StartPath(_) => Outcome::Motor(self.motor_status()),
        }
    }

    fn info(&self) -> Info {
        let mut info = Info {
            uptime_ms: clock::millis(),
            ..Default::default()
        };
        // These cannot fail: the fields are sized for exactly this data.
        let _ = info.firmware_version.push_str(FIRMWARE_VERSION);
        let _ = info.mac_address.extend_from_slice(&self.mac_address);
        let _ = info.serial_number.extend_from_slice(&self.serial_number);
        info.set_self_test(SelfTest {
            step_timer: self.self_test.passed(Check::StepTimer),
            motor_driver: self.self_test.passed(Check::MotorDriver),
            encoder: self.self_test.passed(Check::Encoder),
            eeprom: self.self_test.passed(Check::Eeprom),
        });
        if let Some(link_up) = self.ethernet_link {
            info.set_ethernet(proto::Ethernet { link_up });
        }
        info
    }

    fn config(&self) -> Config {
        let settings = self.settings.settings();
        Config::default()
            .init_ethernet_address(settings.ethernet_address())
            .init_ethernet_port(settings.ethernet_port().into())
            .init_i2c_address(settings.i2c_address().into())
    }

    fn apply_config(&mut self, config: &Config) -> Result<(), Failure> {
        // Check everything before changing anything.
        let port = config
            .ethernet_port()
            .map(|&port| u16::try_from(port).map_err(invalid))
            .transpose()?;
        let i2c_address = config
            .i2c_address()
            .map(|&address| match u8::try_from(address) {
                Ok(address) if address < 0x80 => Ok(address),
                _ => Err(invalid("I2C addresses are 7 bits")),
            })
            .transpose()?;

        let settings = self.settings.settings_mut();
        if let Some(&address) = config.ethernet_address() {
            settings.set_ethernet_address(address);
        }
        if let Some(port) = port {
            settings.set_ethernet_port(port);
        }
        if let Some(address) = i2c_address {
            settings.set_i2c_address(address);
        }
        Ok(())
    }

    fn set_motor_state(&mut self, state: MotorState) -> Result<(), Failure> {
        // Homing watches the encoder for the shaft stalling; without one it
        // would push against the end stop forever.
        if state == MotorState::Homing && !self.has_encoder() {
            return Err(failed("homing needs the encoder"));
        }
        self.motor.set_state(state).map_err(failed)
    }

    fn apply_motion_settings(&mut self, settings: &MotionSettings) -> Result<(), Failure> {
        let brake = settings
            .brake()
            .map(|&brake| brake.try_into().map_err(invalid))
            .transpose()?;
        if settings.acceleration() == Some(&0) {
            return Err(invalid("acceleration must not be zero"));
        }
        if settings.closed_loop() == Some(&true) && !self.has_encoder() {
            return Err(failed("closed loop control needs the encoder"));
        }

        self.motor.motion(|motion| {
            if let Some(&max_speed) = settings.max_speed() {
                motion.set_max_speed(max_speed);
            }
            if let Some(&acceleration) = settings.acceleration() {
                motion.set_acceleration(acceleration);
            }
            if let Some(&closed_loop) = settings.closed_loop() {
                motion.set_closed_loop(closed_loop);
            }
        });
        match brake {
            Some(brake) => self.motor.set_brake(brake).map_err(failed),
            None => Ok(()),
        }
    }

    fn apply_homing_settings(&mut self, settings: &HomingSettings) -> Result<(), Failure> {
        let direction = settings
            .direction()
            .map(|&direction| direction.try_into().map_err(invalid))
            .transpose()?;
        let threshold = settings
            .threshold()
            .map(|&threshold| u8::try_from(threshold).map_err(invalid))
            .transpose()?;

        self.motor.motion(|motion| {
            if let Some(direction) = direction {
                motion.set_home_direction(direction);
            }
            if let Some(&speed) = settings.speed() {
                motion.set_homing_speed(speed);
            }
        });
        match threshold {
            Some(threshold) => self
                .motor
                .set_home_threshold(threshold.into())
                .map_err(failed),
            None => Ok(()),
        }
    }

    fn add_path(&mut self, path: &AddPath) -> Result<(), Failure> {
        self.motor.motion(|motion| {
            if motion.path_free() < path.segments.len() {
                return Err(failed("path queue full"));
            }
            for segment in &path.segments {
                let mode = if segment.relative {
                    PositionMode::Relative
                } else {
                    PositionMode::Absolute
                };
                motion
                    .add_path_segment(segment.speed, segment.steps, mode)
                    .map_err(failed)?;
            }
            Ok(())
        })
    }

    fn motor_status(&mut self) -> proto::Motor {
        let brake = self.motor.brake();
        // One consistent snapshot, taken with the step timer held off.
        let mut status = self.motor.motion(|motion| {
            let mut status = proto::Motor {
                state: motion.state().into(),
                position: motion.position_degrees(),
                target: motion.target_degrees(),
                velocity: motion.velocity(),
                target_velocity: motion.target_velocity(),
                homed: motion.is_homed(),
                path_queued: motion.path_queued() as u32,
                path_free: motion.path_free() as u32,
                stalled: motion.is_stalled(),
                corrections: motion.corrections(),
                ..Default::default()
            };
            status.set_motion(
                MotionSettings::default()
                    .init_max_speed(motion.max_speed())
                    .init_acceleration(motion.acceleration())
                    .init_brake(brake.into())
                    .init_closed_loop(motion.closed_loop()),
            );
            status.set_homing(
                HomingSettings::default()
                    .init_direction(motion.home_direction().into())
                    .init_speed(motion.homing_speed())
                    .init_threshold(motion.home_threshold().into()),
            );
            status
        });
        if let Some((position, velocity)) = self.encoder_reading() {
            status.set_encoder(proto::Encoder { position, velocity });
        }
        status
    }
}
