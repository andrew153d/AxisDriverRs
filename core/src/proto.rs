//! The host protocol: message types generated from `proto/axis.proto`, and
//! conversions between them and the firmware's own types.
//!
//! See the `.proto` file for what the messages mean, and [`crate::framing`]
//! for how they travel over a byte stream.

use crate::types::{self, Rgb};

mod generated {
    #![allow(clippy::all)]
    #![allow(nonstandard_style, unused, irrefutable_let_patterns)]
    include!(concat!(env!("OUT_DIR"), "/axis.rs"));
}

pub use generated::axis_::*;

/// Reported in [`Info::firmware_version`].
pub const FIRMWARE_VERSION: &str = "2.0.0";

/// A protocol enum held a value this firmware does not know.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UnknownValue(pub i32);

/// Implements the conversions between a protocol enum and its counterpart
/// among the firmware's types.
macro_rules! convert_enum {
    ($proto:ident <=> $domain:ident { $($proto_value:ident <=> $domain_value:ident,)+ }) => {
        impl From<types::$domain> for $proto {
            fn from(value: types::$domain) -> Self {
                match value {
                    $(types::$domain::$domain_value => Self::$proto_value,)+
                }
            }
        }

        impl TryFrom<$proto> for types::$domain {
            type Error = UnknownValue;

            fn try_from(value: $proto) -> Result<Self, UnknownValue> {
                $(if value == $proto::$proto_value {
                    return Ok(Self::$domain_value);
                })+
                Err(UnknownValue(value.0))
            }
        }
    };
}

convert_enum!(MotorState <=> MotorState {
    Off <=> Off,
    Position <=> Position,
    Velocity <=> Velocity,
    Path <=> Path,
    Idle <=> Idle,
    Homing <=> Homing,
});

convert_enum!(Brake <=> MotorBrake {
    Normal <=> Normal,
    Freewheel <=> Freewheeling,
    Strong <=> StrongBraking,
    Passive <=> Braking,
});

convert_enum!(Direction <=> HomeDirection {
    Clockwise <=> Clockwise,
    Counterclockwise <=> CounterClockwise,
});

impl From<Rgb> for Color {
    fn from(color: Rgb) -> Self {
        Self {
            red: color.r.into(),
            green: color.g.into(),
            blue: color.b.into(),
        }
    }
}

impl From<Color> for Rgb {
    /// Channels above 255 are clipped.
    fn from(color: Color) -> Self {
        let channel = |value: u32| value.min(255) as u8;
        Self::new(
            channel(color.red),
            channel(color.green),
            channel(color.blue),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{HomeDirection, MotorBrake};

    #[test]
    fn enums_round_trip() {
        for state in [
            types::MotorState::Off,
            types::MotorState::Position,
            types::MotorState::Velocity,
            types::MotorState::Path,
            types::MotorState::Idle,
            types::MotorState::Homing,
        ] {
            assert_eq!(MotorState::from(state).try_into(), Ok(state));
        }
        assert_eq!(MotorState::from(types::MotorState::Path), MotorState(3));
        assert_eq!(Brake::from(MotorBrake::Braking), Brake(3));
        assert_eq!(
            Direction::from(HomeDirection::CounterClockwise),
            Direction(1)
        );
    }

    #[test]
    fn rejects_unknown_enum_values() {
        assert_eq!(
            types::MotorState::try_from(MotorState(9)),
            Err(UnknownValue(9))
        );
        assert_eq!(MotorBrake::try_from(Brake(-1)), Err(UnknownValue(-1)));
    }

    #[test]
    fn colors_convert_and_clip() {
        let color = Color::from(Rgb::new(1, 2, 255));
        assert_eq!((color.red, color.green, color.blue), (1, 2, 255));
        let clipped = Color {
            red: 300,
            green: 0,
            blue: 7,
        };
        assert_eq!(Rgb::from(clipped), Rgb::new(255, 0, 7));
    }
}
