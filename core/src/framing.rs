//! Framing of protocol messages on a byte stream.
//!
//! Protobuf messages do not say where they end, so each one travels in a
//! frame:
//!
//! | length                | payload         | crc                   |
//! | --------------------- | --------------- | --------------------- |
//! | `u16`, little-endian  | `length` bytes  | `u16`, little-endian  |
//!
//! The CRC is CRC-16/CCITT-FALSE over the length and the payload.

use heapless::Vec;
use micropb::{MessageDecode, MessageEncode, PbEncoder};

/// Largest encoded message that fits in a frame.
pub const MAX_PAYLOAD: usize = 508;
const LENGTH_SIZE: usize = 2;
const CRC_SIZE: usize = 2;
/// Largest frame.
pub const MAX_FRAME: usize = LENGTH_SIZE + MAX_PAYLOAD + CRC_SIZE;

// Every message the firmware sends must fit.
const _: () = match <crate::proto::Response as MessageEncode>::MAX_SIZE {
    Ok(size) => assert!(size <= MAX_PAYLOAD),
    Err(_) => panic!("Response has no maximum size"),
};

/// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF.
pub fn crc16(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0xFFFF, |crc, &byte| {
        (0..8).fold(crc ^ u16::from(byte) << 8, |crc, _| {
            if crc & 0x8000 != 0 {
                crc << 1 ^ 0x1021
            } else {
                crc << 1
            }
        })
    })
}

/// An encoded frame, ready to be sent.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Frame {
    bytes: Vec<u8, MAX_FRAME>,
}

impl Frame {
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }
}

/// The message does not fit in a frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TooLarge;

/// Encodes `message` into a frame.
pub fn encode(message: &impl MessageEncode) -> Result<Frame, TooLarge> {
    let length = message.compute_size();
    if length > MAX_PAYLOAD {
        return Err(TooLarge);
    }

    let mut bytes = Vec::new();
    let _ = bytes.extend_from_slice(&(length as u16).to_le_bytes());
    let mut encoder = PbEncoder::new(bytes);
    message.encode(&mut encoder).map_err(|_| TooLarge)?;
    let mut bytes = encoder.into_writer();
    let crc = crc16(&bytes);
    bytes
        .extend_from_slice(&crc.to_le_bytes())
        .map_err(|_| TooLarge)?;
    Ok(Frame { bytes })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrameError {
    /// The frame was damaged or the stream is out of step.
    BadCrc,
    /// The frame arrived intact but does not hold a valid message.
    Malformed,
}

/// Reassembles frames from a byte stream such as the USB serial port, where
/// a frame may arrive in pieces or several frames may arrive at once.
#[derive(Default)]
pub struct FrameBuffer {
    bytes: Vec<u8, MAX_FRAME>,
}

impl FrameBuffer {
    pub const fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    /// Appends received bytes. If they do not fit the stream is out of step,
    /// so everything buffered so far is dropped.
    pub fn extend(&mut self, bytes: &[u8]) {
        if self.bytes.extend_from_slice(bytes).is_err() {
            self.bytes.clear();
        }
    }

    /// Whether nothing is waiting for the rest of its frame to arrive.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Drops any partial frame, e.g. after the stream has gone quiet.
    pub fn clear(&mut self) {
        self.bytes.clear();
    }

    /// Takes the next complete frame out of the buffer and decodes the
    /// message in it. Returns `None` until a whole frame has arrived.
    pub fn next_message<M>(&mut self) -> Option<Result<M, FrameError>>
    where
        M: MessageDecode + Default,
    {
        let length = usize::from(u16::from_le_bytes([
            *self.bytes.first()?,
            *self.bytes.get(1)?,
        ]));
        if length > MAX_PAYLOAD {
            // No valid frame is this long: the stream is out of step.
            self.bytes.clear();
            return Some(Err(FrameError::BadCrc));
        }
        let frame_length = LENGTH_SIZE + length + CRC_SIZE;
        if self.bytes.len() < frame_length {
            return None;
        }

        let (covered, crc) = self.bytes[..frame_length].split_at(LENGTH_SIZE + length);
        if crc16(covered) != u16::from_le_bytes([crc[0], crc[1]]) {
            // The frame boundary cannot be trusted any more.
            self.bytes.clear();
            return Some(Err(FrameError::BadCrc));
        }

        let mut message = M::default();
        let decoded = message.decode_from_bytes(&covered[LENGTH_SIZE..]);
        let remaining = self.bytes.len() - frame_length;
        self.bytes.copy_within(frame_length.., 0);
        self.bytes.truncate(remaining);
        Some(match decoded {
            Ok(()) => Ok(message),
            Err(_) => Err(FrameError::Malformed),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{
        AddPath, GetInfo, MoveTo, PathSegment, Request, Request_::Command, Response, Status,
    };

    fn request(id: u32, command: Command) -> Request {
        Request {
            id,
            command: Some(command),
        }
    }

    #[test]
    fn crc_matches_the_standard_check_value() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
        assert_eq!(crc16(b""), 0xFFFF);
    }

    #[test]
    fn frames_have_length_payload_and_crc() {
        let frame = encode(&request(7, Command::GetInfo(GetInfo {}))).unwrap();
        // id = 7 (field 1, varint), get_info = {} (field 10, empty).
        let payload = [0x08, 0x07, 0x52, 0x00];
        let mut expected = std::vec![4, 0];
        expected.extend_from_slice(&payload);
        expected.extend_from_slice(&crc16(&expected).to_le_bytes());
        assert_eq!(frame.as_bytes(), &expected[..]);
    }

    #[test]
    fn messages_round_trip_through_frames() {
        let sent = request(42, Command::MoveTo(MoveTo { degrees: -90.5 }));
        let mut buffer = FrameBuffer::new();
        buffer.extend(encode(&sent).unwrap().as_bytes());
        assert_eq!(buffer.next_message::<Request>(), Some(Ok(sent)));
        assert_eq!(buffer.next_message::<Request>(), None);
        assert!(buffer.is_empty());
    }

    #[test]
    fn reassembles_split_and_joined_frames() {
        let first = request(1, Command::GetInfo(GetInfo {}));
        let second = request(2, Command::MoveTo(MoveTo { degrees: 1.0 }));
        let mut stream = std::vec::Vec::new();
        stream.extend_from_slice(encode(&first).unwrap().as_bytes());
        stream.extend_from_slice(encode(&second).unwrap().as_bytes());

        let mut buffer = FrameBuffer::new();
        buffer.extend(&stream[..5]);
        assert_eq!(buffer.next_message::<Request>(), None);
        assert!(!buffer.is_empty());
        buffer.extend(&stream[5..11]);
        assert_eq!(buffer.next_message::<Request>(), Some(Ok(first)));
        assert_eq!(buffer.next_message::<Request>(), None);
        buffer.extend(&stream[11..]);
        assert_eq!(buffer.next_message::<Request>(), Some(Ok(second)));
        assert!(buffer.is_empty());
    }

    #[test]
    fn a_damaged_frame_drops_the_buffer() {
        let mut bytes = std::vec::Vec::new();
        bytes.extend_from_slice(
            encode(&request(1, Command::GetInfo(GetInfo {})))
                .unwrap()
                .as_bytes(),
        );
        bytes[3] ^= 0xFF;
        let mut buffer = FrameBuffer::new();
        buffer.extend(&bytes);
        buffer.extend(
            encode(&request(2, Command::GetInfo(GetInfo {})))
                .unwrap()
                .as_bytes(),
        );
        assert_eq!(
            buffer.next_message::<Request>(),
            Some(Err(FrameError::BadCrc))
        );
        assert_eq!(buffer.next_message::<Request>(), None);
        assert!(buffer.is_empty());
    }

    #[test]
    fn an_impossible_length_drops_the_buffer() {
        let mut buffer = FrameBuffer::new();
        buffer.extend(&[0xFF; 16]);
        assert_eq!(
            buffer.next_message::<Request>(),
            Some(Err(FrameError::BadCrc))
        );
        assert!(buffer.is_empty());
    }

    #[test]
    fn an_intact_frame_with_a_bad_message_is_reported_and_skipped() {
        // Field 1 declared as a varint, then nothing.
        let mut bad = std::vec![1, 0, 0x08];
        bad.extend_from_slice(&crc16(&bad).to_le_bytes());
        let good = request(3, Command::GetInfo(GetInfo {}));

        let mut buffer = FrameBuffer::new();
        buffer.extend(&bad);
        buffer.extend(encode(&good).unwrap().as_bytes());
        assert_eq!(
            buffer.next_message::<Request>(),
            Some(Err(FrameError::Malformed))
        );
        assert_eq!(buffer.next_message::<Request>(), Some(Ok(good)));
    }

    #[test]
    fn too_many_path_segments_are_malformed() {
        // Encode by hand: 17 segments where the firmware holds 16.
        let mut payload = std::vec::Vec::new();
        let mut segments = std::vec::Vec::new();
        for _ in 0..17 {
            segments.extend_from_slice(&[0x0A, 0x02, 0x08, 0x01]);
        }
        payload.extend_from_slice(&[0xBA, 0x02, segments.len() as u8]);
        payload.extend_from_slice(&segments);
        let mut frame = std::vec::Vec::from((payload.len() as u16).to_le_bytes());
        frame.extend_from_slice(&payload);
        frame.extend_from_slice(&crc16(&frame).to_le_bytes());

        let mut buffer = FrameBuffer::new();
        buffer.extend(&frame);
        assert_eq!(
            buffer.next_message::<Request>(),
            Some(Err(FrameError::Malformed))
        );
    }

    #[test]
    fn a_full_path_request_fits_in_a_frame() {
        let mut add_path = AddPath::default();
        for _ in 0..16 {
            add_path
                .segments
                .push(PathSegment {
                    speed: u32::MAX,
                    steps: i32::MIN,
                    relative: true,
                })
                .unwrap();
        }
        let sent = request(u32::MAX, Command::AddPath(add_path));
        let mut buffer = FrameBuffer::new();
        buffer.extend(encode(&sent).unwrap().as_bytes());
        assert_eq!(buffer.next_message::<Request>(), Some(Ok(sent)));
    }

    #[test]
    fn responses_encode() {
        let response = Response {
            id: 5,
            status: Status::Error,
            result: None,
        };
        let frame = encode(&response).unwrap();
        assert_eq!(&frame.as_bytes()[..6], &[4, 0, 0x08, 0x05, 0x10, 0x01]);
    }
}
