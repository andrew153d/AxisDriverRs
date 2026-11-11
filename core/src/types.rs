//! Value types shared by the motion logic and the host protocol.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorState {
    /// Driver outputs disabled.
    Off,
    /// Accelerating/decelerating towards a target position.
    Position,
    /// Running at a constant velocity.
    Velocity,
    /// Playing back the queued path.
    Path,
    /// Driver outputs enabled, holding position.
    Idle,
    /// Running the sensorless homing sequence.
    Homing,
}

/// Standstill behaviour of the driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorBrake {
    Normal,
    Freewheeling,
    /// Coils shorted through the low-side drivers.
    StrongBraking,
    /// Coils shorted through the high-side drivers.
    Braking,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HomeDirection {
    Clockwise,
    CounterClockwise,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PositionMode {
    Absolute,
    Relative,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const BLACK: Self = Self::new(0, 0, 0);
    pub const RED: Self = Self::new(255, 0, 0);
    pub const GREEN: Self = Self::new(0, 128, 0);
    pub const BLUE: Self = Self::new(0, 0, 255);
    pub const PURPLE: Self = Self::new(128, 0, 128);

    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}
