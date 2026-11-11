//! Hardware-independent logic of the Axis Driver firmware.
//!
//! Everything in this crate is plain `no_std` Rust with no dependency on the
//! microcontroller, so it can be unit-tested on the host:
//!
//! ```sh
//! just test
//! ```

#![cfg_attr(not(test), no_std)]

pub mod eeprom;
pub mod encoder;
pub mod led;
pub mod motion;
pub mod profile;
pub mod protocol;
pub mod selftest;
pub mod settings;
pub mod tlv493d;
