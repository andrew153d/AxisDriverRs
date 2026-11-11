//! Driver for the AT24MAC402 EEPROM: 256 bytes of storage plus a factory
//! programmed MAC address and serial number.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

pub const EEPROM_SIZE: usize = 256;
/// The EEPROM is written a page at a time.
pub const PAGE_SIZE: usize = 16;

/// Bus address of the storage array.
const MEMORY_ADDRESS: u8 = 0x50;
/// Bus address of the read-only identity area.
const IDENTITY_ADDRESS: u8 = 0x58;
const SERIAL_NUMBER_REGISTER: u8 = 0x80;
const MAC_ADDRESS_REGISTER: u8 = 0x9A;

/// Time the chip takes to commit a page, during which it does not respond.
const WRITE_CYCLE_MS: u32 = 8;

#[derive(Default)]
pub struct Eeprom;

impl Eeprom {
    pub const fn new() -> Self {
        Self
    }

    /// Reads `buffer.len()` bytes starting at `offset`.
    pub fn read<I: I2c>(
        &mut self,
        i2c: &mut I,
        offset: u8,
        buffer: &mut [u8],
    ) -> Result<(), I::Error> {
        debug_assert!(usize::from(offset) + buffer.len() <= EEPROM_SIZE);
        i2c.write_read(MEMORY_ADDRESS, &[offset], buffer)
    }

    /// Writes up to one page of data and waits for it to be committed.
    /// `offset` must be the start of a page.
    pub fn write_page<I: I2c>(
        &mut self,
        i2c: &mut I,
        delay: &mut impl DelayNs,
        offset: u8,
        data: &[u8],
    ) -> Result<(), I::Error> {
        debug_assert!(usize::from(offset) % PAGE_SIZE == 0 && data.len() <= PAGE_SIZE);
        let mut message = [0; 1 + PAGE_SIZE];
        message[0] = offset;
        message[1..=data.len()].copy_from_slice(data);
        i2c.write(MEMORY_ADDRESS, &message[..=data.len()])?;
        delay.delay_ms(WRITE_CYCLE_MS);
        Ok(())
    }

    /// Reads the globally unique EUI-48 MAC address.
    pub fn mac_address<I: I2c>(&mut self, i2c: &mut I) -> Result<[u8; 6], I::Error> {
        let mut mac = [0; 6];
        i2c.write_read(IDENTITY_ADDRESS, &[MAC_ADDRESS_REGISTER], &mut mac)?;
        Ok(mac)
    }

    /// Reads the 128-bit serial number.
    pub fn serial_number<I: I2c>(&mut self, i2c: &mut I) -> Result<[u8; 16], I::Error> {
        let mut serial = [0; 16];
        i2c.write_read(IDENTITY_ADDRESS, &[SERIAL_NUMBER_REGISTER], &mut serial)?;
        Ok(serial)
    }
}

/// An in-memory EEPROM for tests.
#[cfg(test)]
pub(crate) mod fake {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::i2c::{ErrorType, Operation};

    pub const MAC: [u8; 6] = [0xFC, 0xC2, 0x3D, 0x01, 0x02, 0x03];

    pub struct FakeBus {
        pub memory: [u8; EEPROM_SIZE],
        pub page_writes: usize,
        /// Address latched by the last write, as in the real chip.
        pointer: usize,
    }

    pub struct NoDelay;

    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    impl FakeBus {
        pub fn new() -> Self {
            Self {
                memory: [0xFF; EEPROM_SIZE],
                page_writes: 0,
                pointer: 0,
            }
        }

        pub fn delay() -> NoDelay {
            NoDelay
        }
    }

    impl ErrorType for FakeBus {
        type Error = Infallible;
    }

    impl I2c for FakeBus {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Infallible> {
            for operation in operations {
                match operation {
                    Operation::Write(bytes) => {
                        self.pointer = usize::from(bytes[0]);
                        if bytes.len() > 1 {
                            assert_eq!(address, MEMORY_ADDRESS);
                            assert_eq!(self.pointer % PAGE_SIZE, 0);
                            let data = &bytes[1..];
                            self.memory[self.pointer..self.pointer + data.len()]
                                .copy_from_slice(data);
                            self.page_writes += 1;
                        }
                    }
                    Operation::Read(buffer) => match address {
                        MEMORY_ADDRESS => {
                            let end = self.pointer + buffer.len();
                            buffer.copy_from_slice(&self.memory[self.pointer..end]);
                        }
                        IDENTITY_ADDRESS => {
                            assert_eq!(self.pointer, usize::from(MAC_ADDRESS_REGISTER));
                            buffer.copy_from_slice(&MAC);
                        }
                        _ => panic!("unexpected address {address:#x}"),
                    },
                }
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::{FakeBus, MAC};
    use super::*;

    #[test]
    fn reads_back_written_pages() {
        let mut bus = FakeBus::new();
        let mut eeprom = Eeprom::new();
        eeprom
            .write_page(&mut bus, &mut FakeBus::delay(), 32, &[1, 2, 3])
            .unwrap();

        let mut buffer = [0; 4];
        eeprom.read(&mut bus, 31, &mut buffer).unwrap();
        assert_eq!(buffer, [0xFF, 1, 2, 3]);
    }

    #[test]
    fn reads_the_mac_address() {
        let mut bus = FakeBus::new();
        assert_eq!(Eeprom::new().mac_address(&mut bus), Ok(MAC));
    }
}
