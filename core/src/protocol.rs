//! The Axis wire protocol (see `AxisProtocol.md` in the original project).
//!
//! Every message is a little-endian frame:
//!
//! | header                               | body              | footer        |
//! | ------------------------------------ | ----------------- | ------------- |
//! | `message_type: u16`, `body_size: u16` | `body_size` bytes | `checksum: u16` |
//!
//! The checksum is the wrapping sum of all header and body bytes.
//!
//! Hosts send [`Request`]s. "Set" requests and actions are answered with an
//! ACK frame; "get" requests are sent with an empty body and answered with a
//! frame of the same message type carrying the value.

use heapless::Vec;

pub const FIRMWARE_VERSION: [u8; 4] = [1, 0, 0, 0];

const HEADER_SIZE: usize = 4;
const FOOTER_SIZE: usize = 2;
/// Size of a frame with an empty body.
pub const FRAME_OVERHEAD: usize = HEADER_SIZE + FOOTER_SIZE;
/// Largest body the firmware ever sends.
const MAX_REPLY_BODY: usize = 8;

/// Declares a `#[repr]` enum together with a checked conversion from its
/// wire representation.
macro_rules! wire_enum {
    (
        $(#[$meta:meta])*
        $vis:vis enum $name:ident: $repr:ty {
            $($(#[$variant_meta:meta])* $variant:ident = $value:expr,)+
        }
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        #[repr($repr)]
        $vis enum $name {
            $($(#[$variant_meta])* $variant = $value,)+
        }

        impl $name {
            pub const fn from_wire(value: $repr) -> Option<Self> {
                match value {
                    $($value => Some(Self::$variant),)+
                    _ => None,
                }
            }
        }
    };
}

wire_enum! {
    pub enum MessageType: u16 {
        Ack = 0x0100,
        GetVersion = 0x0101,
        SetI2cAddress = 0x0102,
        GetI2cAddress = 0x0103,
        SetEthernetAddress = 0x0104,
        GetEthernetAddress = 0x0105,
        SetEthernetPort = 0x0106,
        GetEthernetPort = 0x0107,
        GetMacAddress = 0x0108,
        SaveConfiguration = 0x0109,
        SetLedColor = 0x0200,
        GetLedColor = 0x0201,
        AddLedStep = 0x0202,
        SetHomeDirection = 0x0300,
        GetHomeDirection = 0x0301,
        SetHomeThreshold = 0x0302,
        GetHomeThreshold = 0x0303,
        SetHomeSpeed = 0x0304,
        GetHomeSpeed = 0x0305,
        GetHomedState = 0x0306,
        SetMotorState = 0x0307,
        GetMotorState = 0x0308,
        SetMotorBrake = 0x0309,
        GetMotorBrake = 0x030A,
        SetMaxSpeed = 0x030B,
        GetMaxSpeed = 0x030C,
        SetAcceleration = 0x030D,
        GetAcceleration = 0x030E,
        SetCurrentPosition = 0x030F,
        GetCurrentPosition = 0x0310,
        SetTargetPosition = 0x0311,
        GetTargetPosition = 0x0312,
        SetVelocity = 0x0313,
        GetVelocity = 0x0314,
        Home = 0x0400,
        SetRelativeTargetPosition = 0x0401,
        SetVelocityAndSteps = 0x0402,
        StartPath = 0x0403,
    }
}

wire_enum! {
    pub enum Status: u8 {
        Success = 0,
        Error = 1,
        InvalidCommand = 2,
    }
}

wire_enum! {
    pub enum MotorState: u8 {
        /// Driver outputs disabled.
        Off = 0,
        /// Accelerating/decelerating towards a target position.
        Position = 1,
        /// Running at a constant velocity.
        Velocity = 2,
        /// Playing back the queued velocity/step path.
        VelocityStep = 3,
        /// Driver outputs enabled, holding position.
        IdleOn = 4,
        /// Running the sensorless homing sequence.
        Home = 5,
    }
}

wire_enum! {
    /// Standstill behaviour of the driver; the values are the TMC2209
    /// `PWMCONF.freewheel` settings.
    pub enum MotorBrake: u8 {
        Normal = 0,
        Freewheeling = 1,
        StrongBraking = 2,
        Braking = 3,
    }
}

wire_enum! {
    pub enum HomeDirection: u8 {
        Clockwise = 0,
        CounterClockwise = 1,
    }
}

wire_enum! {
    pub enum PositionMode: u8 {
        Absolute = 0,
        Relative = 1,
    }
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

/// A decoded message from the host.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Request {
    GetVersion,
    SetI2cAddress(u8),
    GetI2cAddress,
    SetEthernetAddress(u32),
    GetEthernetAddress,
    SetEthernetPort(u32),
    GetEthernetPort,
    GetMacAddress,
    SaveConfiguration,
    SetLedColor(Rgb),
    GetLedColor,
    AddLedStep {
        duration_ms: u32,
        color: Rgb,
    },
    SetHomeDirection(HomeDirection),
    GetHomeDirection,
    SetHomeThreshold(u32),
    GetHomeThreshold,
    SetHomeSpeed(u32),
    GetHomeSpeed,
    GetHomedState,
    Home,
    SetMotorState(MotorState),
    GetMotorState,
    SetMotorBrake(MotorBrake),
    GetMotorBrake,
    /// Steps per second.
    SetMaxSpeed(u32),
    GetMaxSpeed,
    /// Steps per second squared.
    SetAcceleration(u32),
    GetAcceleration,
    /// Degrees.
    SetCurrentPosition(f64),
    GetCurrentPosition,
    /// Degrees.
    SetTargetPosition(f64),
    GetTargetPosition,
    /// Degrees.
    SetRelativeTargetPosition(f64),
    /// Steps per second.
    SetVelocity(f64),
    GetVelocity,
    SetVelocityAndSteps {
        velocity: i32,
        steps: i32,
        mode: PositionMode,
    },
    StartPath,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParseError {
    /// The data is shorter than its header claims.
    Truncated,
    BadChecksum,
    UnknownType(u16),
    /// A known message whose body is too short or holds an invalid value.
    InvalidBody(MessageType),
}

impl ParseError {
    /// The message type to name in the `InvalidCommand` ACK, if the frame was
    /// intact enough to deserve an answer.
    pub fn rejected_type(self) -> Option<u16> {
        match self {
            Self::UnknownType(message_type) => Some(message_type),
            Self::InvalidBody(message_type) => Some(message_type as u16),
            Self::Truncated | Self::BadChecksum => None,
        }
    }
}

impl Request {
    /// Decodes one complete frame.
    pub fn parse(frame: &[u8]) -> Result<Self, ParseError> {
        let (raw_type, body) = split_frame(frame)?;
        let message_type =
            MessageType::from_wire(raw_type).ok_or(ParseError::UnknownType(raw_type))?;
        Self::from_body(message_type, body).ok_or(ParseError::InvalidBody(message_type))
    }

    fn from_body(message_type: MessageType, body: &[u8]) -> Option<Self> {
        use MessageType as T;

        let mut body = Reader(body);
        Some(match message_type {
            // ACKs only ever travel from the device to the host.
            T::Ack => return None,
            T::GetVersion => Self::GetVersion,
            T::SetI2cAddress => Self::SetI2cAddress(body.u8()?),
            T::GetI2cAddress => Self::GetI2cAddress,
            T::SetEthernetAddress => Self::SetEthernetAddress(body.u32()?),
            T::GetEthernetAddress => Self::GetEthernetAddress,
            T::SetEthernetPort => Self::SetEthernetPort(body.u32()?),
            T::GetEthernetPort => Self::GetEthernetPort,
            T::GetMacAddress => Self::GetMacAddress,
            T::SaveConfiguration => Self::SaveConfiguration,
            T::SetLedColor => Self::SetLedColor(body.rgb()?),
            T::GetLedColor => Self::GetLedColor,
            T::AddLedStep => Self::AddLedStep {
                duration_ms: body.u32()?,
                color: body.rgb()?,
            },
            T::SetHomeDirection => Self::SetHomeDirection(HomeDirection::from_wire(body.u8()?)?),
            T::GetHomeDirection => Self::GetHomeDirection,
            T::SetHomeThreshold => Self::SetHomeThreshold(body.u32()?),
            T::GetHomeThreshold => Self::GetHomeThreshold,
            T::SetHomeSpeed => Self::SetHomeSpeed(body.u32()?),
            T::GetHomeSpeed => Self::GetHomeSpeed,
            T::GetHomedState => Self::GetHomedState,
            T::Home => Self::Home,
            T::SetMotorState => Self::SetMotorState(MotorState::from_wire(body.u8()?)?),
            T::GetMotorState => Self::GetMotorState,
            T::SetMotorBrake => Self::SetMotorBrake(MotorBrake::from_wire(body.u8()?)?),
            T::GetMotorBrake => Self::GetMotorBrake,
            T::SetMaxSpeed => Self::SetMaxSpeed(body.u32()?),
            T::GetMaxSpeed => Self::GetMaxSpeed,
            T::SetAcceleration => Self::SetAcceleration(body.u32()?),
            T::GetAcceleration => Self::GetAcceleration,
            T::SetCurrentPosition => Self::SetCurrentPosition(body.f64()?),
            T::GetCurrentPosition => Self::GetCurrentPosition,
            T::SetTargetPosition => Self::SetTargetPosition(body.f64()?),
            T::GetTargetPosition => Self::GetTargetPosition,
            T::SetRelativeTargetPosition => Self::SetRelativeTargetPosition(body.f64()?),
            T::SetVelocity => Self::SetVelocity(body.f64()?),
            T::GetVelocity => Self::GetVelocity,
            T::SetVelocityAndSteps => Self::SetVelocityAndSteps {
                velocity: body.i32()?,
                steps: body.i32()?,
                mode: PositionMode::from_wire(body.u8()?)?,
            },
            T::StartPath => Self::StartPath,
        })
    }

    pub fn message_type(&self) -> MessageType {
        use MessageType as T;

        match self {
            Self::GetVersion => T::GetVersion,
            Self::SetI2cAddress(_) => T::SetI2cAddress,
            Self::GetI2cAddress => T::GetI2cAddress,
            Self::SetEthernetAddress(_) => T::SetEthernetAddress,
            Self::GetEthernetAddress => T::GetEthernetAddress,
            Self::SetEthernetPort(_) => T::SetEthernetPort,
            Self::GetEthernetPort => T::GetEthernetPort,
            Self::GetMacAddress => T::GetMacAddress,
            Self::SaveConfiguration => T::SaveConfiguration,
            Self::SetLedColor(_) => T::SetLedColor,
            Self::GetLedColor => T::GetLedColor,
            Self::AddLedStep { .. } => T::AddLedStep,
            Self::SetHomeDirection(_) => T::SetHomeDirection,
            Self::GetHomeDirection => T::GetHomeDirection,
            Self::SetHomeThreshold(_) => T::SetHomeThreshold,
            Self::GetHomeThreshold => T::GetHomeThreshold,
            Self::SetHomeSpeed(_) => T::SetHomeSpeed,
            Self::GetHomeSpeed => T::GetHomeSpeed,
            Self::GetHomedState => T::GetHomedState,
            Self::Home => T::Home,
            Self::SetMotorState(_) => T::SetMotorState,
            Self::GetMotorState => T::GetMotorState,
            Self::SetMotorBrake(_) => T::SetMotorBrake,
            Self::GetMotorBrake => T::GetMotorBrake,
            Self::SetMaxSpeed(_) => T::SetMaxSpeed,
            Self::GetMaxSpeed => T::GetMaxSpeed,
            Self::SetAcceleration(_) => T::SetAcceleration,
            Self::GetAcceleration => T::GetAcceleration,
            Self::SetCurrentPosition(_) => T::SetCurrentPosition,
            Self::GetCurrentPosition => T::GetCurrentPosition,
            Self::SetTargetPosition(_) => T::SetTargetPosition,
            Self::GetTargetPosition => T::GetTargetPosition,
            Self::SetRelativeTargetPosition(_) => T::SetRelativeTargetPosition,
            Self::SetVelocity(_) => T::SetVelocity,
            Self::GetVelocity => T::GetVelocity,
            Self::SetVelocityAndSteps { .. } => T::SetVelocityAndSteps,
            Self::StartPath => T::StartPath,
        }
    }
}

/// An encoded frame, ready to be sent to the host.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Reply {
    bytes: Vec<u8, { FRAME_OVERHEAD + MAX_REPLY_BODY }>,
}

impl Reply {
    /// Encodes a frame.
    ///
    /// # Panics
    ///
    /// Panics if `body` is longer than any reply the protocol defines.
    pub fn new(message_type: MessageType, body: &[u8]) -> Self {
        Self::encode(message_type as u16, body)
    }

    pub fn ack(request: MessageType, status: Status) -> Self {
        Self::ack_raw(request as u16, status)
    }

    /// Acknowledges a message type the firmware may not even know.
    pub fn ack_raw(request: u16, status: Status) -> Self {
        let [low, high] = request.to_le_bytes();
        Self::new(MessageType::Ack, &[low, high, status as u8])
    }

    fn encode(message_type: u16, body: &[u8]) -> Self {
        let mut bytes = Vec::new();
        let mut push = |part: &[u8]| {
            bytes.extend_from_slice(part).expect("reply body too long");
        };
        push(&message_type.to_le_bytes());
        push(&(body.len() as u16).to_le_bytes());
        push(body);
        let checksum = checksum(&bytes);
        bytes
            .extend_from_slice(&checksum.to_le_bytes())
            .expect("reply body too long");
        Self { bytes }
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }
}

/// Wrapping 16-bit sum of `bytes`.
pub fn checksum(bytes: &[u8]) -> u16 {
    bytes
        .iter()
        .fold(0, |sum, &byte| sum.wrapping_add(u16::from(byte)))
}

/// Total length of the frame starting at `data`, once its header has arrived.
fn frame_len(data: &[u8]) -> Option<usize> {
    let body_size = u16::from_le_bytes([*data.get(2)?, *data.get(3)?]);
    Some(FRAME_OVERHEAD + usize::from(body_size))
}

/// Validates a frame and splits it into its raw message type and body.
fn split_frame(frame: &[u8]) -> Result<(u16, &[u8]), ParseError> {
    let len = frame_len(frame).ok_or(ParseError::Truncated)?;
    let frame = frame.get(..len).ok_or(ParseError::Truncated)?;
    let (payload, footer) = frame.split_at(len - FOOTER_SIZE);
    if checksum(payload) != u16::from_le_bytes([footer[0], footer[1]]) {
        return Err(ParseError::BadChecksum);
    }
    let message_type = u16::from_le_bytes([payload[0], payload[1]]);
    Ok((message_type, &payload[HEADER_SIZE..]))
}

/// Cursor over a little-endian message body.
struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn array<const N: usize>(&mut self) -> Option<[u8; N]> {
        let (head, tail) = self.0.split_first_chunk()?;
        self.0 = tail;
        Some(*head)
    }

    fn u8(&mut self) -> Option<u8> {
        self.array().map(|[byte]| byte)
    }

    fn u32(&mut self) -> Option<u32> {
        self.array().map(u32::from_le_bytes)
    }

    fn i32(&mut self) -> Option<i32> {
        self.array().map(i32::from_le_bytes)
    }

    fn f64(&mut self) -> Option<f64> {
        self.array().map(f64::from_le_bytes)
    }

    fn rgb(&mut self) -> Option<Rgb> {
        self.array().map(|[r, g, b]| Rgb::new(r, g, b))
    }
}

/// Reassembles frames from a byte stream such as the USB serial port, where a
/// frame may arrive in pieces or several frames may arrive at once.
#[derive(Default)]
pub struct FrameBuffer<const N: usize = 256> {
    bytes: Vec<u8, N>,
}

impl<const N: usize> FrameBuffer<N> {
    pub const fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    /// Appends received bytes. If they do not fit the stream is out of sync,
    /// so everything buffered so far is dropped.
    pub fn extend(&mut self, bytes: &[u8]) {
        if self.bytes.extend_from_slice(bytes).is_err() {
            self.bytes.clear();
        }
    }

    /// Whether part of a frame is waiting for the rest to arrive.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Drops any partial frame, e.g. after the stream has gone quiet.
    pub fn clear(&mut self) {
        self.bytes.clear();
    }

    /// Takes the next complete frame out of the buffer and decodes it.
    ///
    /// Returns `None` until a whole frame has arrived.
    pub fn next_request(&mut self) -> Option<Result<Request, ParseError>> {
        let len = frame_len(&self.bytes)?;
        if len > N {
            // No valid frame is this long: the stream is out of sync.
            self.bytes.clear();
            return None;
        }
        if self.bytes.len() < len {
            return None;
        }

        let result = Request::parse(&self.bytes[..len]);
        if result == Err(ParseError::BadChecksum) {
            // The frame boundary cannot be trusted any more.
            self.bytes.clear();
        } else {
            let remaining = self.bytes.len() - len;
            self.bytes.copy_within(len.., 0);
            self.bytes.truncate(remaining);
        }
        Some(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a frame the way the host libraries do.
    fn frame(message_type: u16, body: &[u8]) -> std::vec::Vec<u8> {
        let mut frame = std::vec::Vec::new();
        frame.extend_from_slice(&message_type.to_le_bytes());
        frame.extend_from_slice(&(body.len() as u16).to_le_bytes());
        frame.extend_from_slice(body);
        let sum: u32 = frame.iter().map(|&byte| u32::from(byte)).sum();
        frame.extend_from_slice(&(sum as u16).to_le_bytes());
        frame
    }

    #[test]
    fn parses_get_request_with_empty_body() {
        assert_eq!(Request::parse(&frame(0x0101, &[])), Ok(Request::GetVersion));
    }

    #[test]
    fn parses_set_requests() {
        assert_eq!(
            Request::parse(&frame(0x0311, &90.5f64.to_le_bytes())),
            Ok(Request::SetTargetPosition(90.5))
        );
        assert_eq!(
            Request::parse(&frame(0x0202, &[0xE8, 0x03, 0, 0, 1, 2, 3])),
            Ok(Request::AddLedStep {
                duration_ms: 1000,
                color: Rgb::new(1, 2, 3)
            })
        );

        let mut body = std::vec::Vec::new();
        body.extend_from_slice(&(-400i32).to_le_bytes());
        body.extend_from_slice(&1600i32.to_le_bytes());
        body.push(1);
        assert_eq!(
            Request::parse(&frame(0x0402, &body)),
            Ok(Request::SetVelocityAndSteps {
                velocity: -400,
                steps: 1600,
                mode: PositionMode::Relative
            })
        );
    }

    #[test]
    fn ignores_the_body_of_actions() {
        assert_eq!(
            Request::parse(&frame(0x0400, &[1, 0, 0, 0])),
            Ok(Request::Home)
        );
        assert_eq!(Request::parse(&frame(0x0403, &[1])), Ok(Request::StartPath));
    }

    #[test]
    fn rejects_bad_frames() {
        assert_eq!(
            Request::parse(&frame(0x0999, &[])),
            Err(ParseError::UnknownType(0x0999))
        );
        assert_eq!(
            Request::parse(&frame(0x0307, &[])),
            Err(ParseError::InvalidBody(MessageType::SetMotorState))
        );
        assert_eq!(
            Request::parse(&frame(0x0307, &[9])),
            Err(ParseError::InvalidBody(MessageType::SetMotorState))
        );

        let mut corrupt = frame(0x0307, &[1]);
        corrupt[4] = 2;
        assert_eq!(Request::parse(&corrupt), Err(ParseError::BadChecksum));
        assert_eq!(Request::parse(&corrupt[..3]), Err(ParseError::Truncated));
        assert_eq!(Request::parse(&corrupt[..6]), Err(ParseError::Truncated));
    }

    #[test]
    fn message_type_round_trips() {
        let request = Request::parse(&frame(0x030B, &800u32.to_le_bytes())).unwrap();
        assert_eq!(request.message_type(), MessageType::SetMaxSpeed);
    }

    #[test]
    fn encodes_replies_like_the_original_firmware() {
        assert_eq!(
            Reply::ack(MessageType::SetMotorState, Status::Success).as_bytes(),
            &frame(0x0100, &[0x07, 0x03, 0x00])[..]
        );
        assert_eq!(
            Reply::new(MessageType::GetVersion, &FIRMWARE_VERSION).as_bytes(),
            &[0x01, 0x01, 0x04, 0x00, 1, 0, 0, 0, 0x07, 0x00]
        );
        assert_eq!(
            Reply::new(MessageType::GetCurrentPosition, &1.8f64.to_le_bytes()).as_bytes(),
            &frame(0x0310, &1.8f64.to_le_bytes())[..]
        );
    }

    #[test]
    fn frame_buffer_reassembles_split_and_joined_frames() {
        let mut stream = frame(0x0101, &[]);
        stream.extend(frame(0x030B, &800u32.to_le_bytes()));

        let mut buffer = FrameBuffer::<64>::new();
        buffer.extend(&stream[..9]);
        assert_eq!(buffer.next_request(), Some(Ok(Request::GetVersion)));
        assert_eq!(buffer.next_request(), None);
        assert!(!buffer.is_empty());

        buffer.extend(&stream[9..]);
        assert_eq!(buffer.next_request(), Some(Ok(Request::SetMaxSpeed(800))));
        assert_eq!(buffer.next_request(), None);
        assert!(buffer.is_empty());
    }

    #[test]
    fn frame_buffer_keeps_going_after_an_unknown_message() {
        let mut stream = frame(0x0999, &[1, 2]);
        stream.extend(frame(0x0101, &[]));

        let mut buffer = FrameBuffer::<64>::new();
        buffer.extend(&stream);
        assert_eq!(
            buffer.next_request(),
            Some(Err(ParseError::UnknownType(0x0999)))
        );
        assert_eq!(buffer.next_request(), Some(Ok(Request::GetVersion)));
    }

    #[test]
    fn frame_buffer_resyncs_after_garbage() {
        let mut buffer = FrameBuffer::<64>::new();
        buffer.extend(&[0xFF; 8]);
        assert_eq!(buffer.next_request(), None);
        assert!(buffer.is_empty());

        let mut corrupt = frame(0x0101, &[]);
        corrupt[5] ^= 0xFF;
        buffer.extend(&corrupt);
        buffer.extend(&frame(0x0101, &[]));
        assert_eq!(buffer.next_request(), Some(Err(ParseError::BadChecksum)));
        assert_eq!(buffer.next_request(), None);
    }
}
