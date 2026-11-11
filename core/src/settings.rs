//! Persistent configuration.
//!
//! The settings live in the board's 256 byte EEPROM as a packed,
//! little-endian image, in the layout defined by the original firmware:
//!
//! | offset | size | field                                   |
//! | ------ | ---- | --------------------------------------- |
//! | 0      | 2    | Ethernet UDP port                       |
//! | 2      | 4    | Ethernet IP address                     |
//! | 6      | 1    | I2C address                             |
//! | 7      | 1    | motor settings (placeholder)            |
//! | 8      | 166  | Home Assistant / MQTT settings          |
//! | 174    | 82   | unused                                  |
//!
//! Fields this firmware does not use are kept untouched.

use crate::eeprom::{Eeprom, EEPROM_SIZE, PAGE_SIZE};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

const ETHERNET_PORT: usize = 0;
const ETHERNET_ADDRESS: usize = 2;
const I2C_ADDRESS: usize = 6;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Settings {
    image: [u8; EEPROM_SIZE],
}

impl Settings {
    pub const fn from_image(image: [u8; EEPROM_SIZE]) -> Self {
        Self { image }
    }

    pub fn image(&self) -> &[u8; EEPROM_SIZE] {
        &self.image
    }

    pub fn ethernet_port(&self) -> u16 {
        u16::from_le_bytes(self.field(ETHERNET_PORT))
    }

    pub fn set_ethernet_port(&mut self, port: u16) {
        self.set_field(ETHERNET_PORT, port.to_le_bytes());
    }

    pub fn ethernet_address(&self) -> u32 {
        u32::from_le_bytes(self.field(ETHERNET_ADDRESS))
    }

    pub fn set_ethernet_address(&mut self, address: u32) {
        self.set_field(ETHERNET_ADDRESS, address.to_le_bytes());
    }

    pub fn i2c_address(&self) -> u8 {
        self.image[I2C_ADDRESS]
    }

    pub fn set_i2c_address(&mut self, address: u8) {
        self.image[I2C_ADDRESS] = address;
    }

    fn field<const N: usize>(&self, offset: usize) -> [u8; N] {
        let mut bytes = [0; N];
        bytes.copy_from_slice(&self.image[offset..offset + N]);
        bytes
    }

    fn set_field<const N: usize>(&mut self, offset: usize, bytes: [u8; N]) {
        self.image[offset..offset + N].copy_from_slice(&bytes);
    }
}

/// The working copy of the settings, together with what is known to be in
/// the EEPROM so that saving only rewrites the pages that changed.
pub struct SettingsStore {
    current: Settings,
    saved: Settings,
}

impl SettingsStore {
    /// Reads the settings from the EEPROM.
    pub fn load<I: I2c>(eeprom: &mut Eeprom, i2c: &mut I) -> Result<Self, I::Error> {
        let mut image = [0; EEPROM_SIZE];
        eeprom.read(i2c, 0, &mut image)?;
        let saved = Settings::from_image(image);
        Ok(Self {
            current: saved.clone(),
            saved,
        })
    }

    /// Starts from the given settings without touching the EEPROM, as if
    /// they had just been loaded.
    pub fn unsaved(settings: Settings) -> Self {
        Self {
            current: settings.clone(),
            saved: settings,
        }
    }

    pub fn settings(&self) -> &Settings {
        &self.current
    }

    pub fn settings_mut(&mut self) -> &mut Settings {
        &mut self.current
    }

    /// Writes the pages that changed since the last load or save.
    pub fn save<I: I2c>(
        &mut self,
        eeprom: &mut Eeprom,
        i2c: &mut I,
        delay: &mut impl DelayNs,
    ) -> Result<(), I::Error> {
        let pages = self.current.image.chunks_exact(PAGE_SIZE);
        let saved_pages = self.saved.image.chunks_exact_mut(PAGE_SIZE);
        for (index, (page, saved_page)) in pages.zip(saved_pages).enumerate() {
            if page != saved_page {
                eeprom.write_page(i2c, delay, (index * PAGE_SIZE) as u8, page)?;
                saved_page.copy_from_slice(page);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eeprom::fake::FakeBus;

    #[test]
    fn fields_use_the_original_packed_layout() {
        let mut image = [0; EEPROM_SIZE];
        image[..8].copy_from_slice(&[0xE0, 0x2E, 0x0A, 0x00, 0xA8, 0xC0, 0x42, 0x00]);
        let mut settings = Settings::from_image(image);

        assert_eq!(settings.ethernet_port(), 12000);
        assert_eq!(settings.ethernet_address(), 0xC0A8_000A);
        assert_eq!(settings.i2c_address(), 0x42);

        settings.set_ethernet_port(0x1234);
        settings.set_ethernet_address(0x0102_0304);
        settings.set_i2c_address(0x10);
        assert_eq!(
            settings.image()[..7],
            [0x34, 0x12, 0x04, 0x03, 0x02, 0x01, 0x10]
        );
    }

    #[test]
    fn loads_and_saves_only_changed_pages() {
        let mut bus = FakeBus::new();
        bus.memory[0] = 0xE0;
        bus.memory[1] = 0x2E;
        bus.memory[200] = 0x77;
        let mut eeprom = Eeprom::new();

        let mut store = SettingsStore::load(&mut eeprom, &mut bus).unwrap();
        assert_eq!(store.settings().ethernet_port(), 12000);

        store
            .save(&mut eeprom, &mut bus, &mut FakeBus::delay())
            .unwrap();
        assert_eq!(bus.page_writes, 0);

        store.settings_mut().set_i2c_address(0x33);
        store
            .save(&mut eeprom, &mut bus, &mut FakeBus::delay())
            .unwrap();
        assert_eq!(bus.page_writes, 1);
        assert_eq!(bus.memory[6], 0x33);
        assert_eq!(bus.memory[200], 0x77);

        store
            .save(&mut eeprom, &mut bus, &mut FakeBus::delay())
            .unwrap();
        assert_eq!(bus.page_writes, 1);
    }
}
