//! The Ethernet HAT: a W5500 on the auxiliary port, over which the host
//! protocol runs as UDP datagrams.
//!
//! Each datagram sent to the configured port holds one request, and the
//! response goes back to wherever it came from.

use core::fmt::{self, Write};

use atsamd_hal::ehal::delay::DelayNs;
use atsamd_hal::ehal::digital::{InputPin, OutputPin};
use atsamd_hal::ehal::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use atsamd_hal::gpio::{Pin, PullUpInput, PushPullOutput, PA03, PB01};
use atsamd_hal::sercom::spi;
use axis_core::framing::{self, FrameError, MAX_PAYLOAD};
use axis_core::proto::{Request, FIRMWARE_VERSION};
use axis_core::settings::Settings;
use axis_core::w5500::{self, Endpoint, Interface, SendProgress, W5500};
use heapless::String;
use log::{info, warn};

use crate::board::{AuxSpi, EthernetHat};
use crate::clock::{self, Delay};

/// How long the W5500 is given to answer after its reset is released.
const STARTUP_TIMEOUT_MS: u32 = 100;
/// The receive buffer is looked at this often even if the interrupt line
/// stays quiet, in case it is not connected.
const RECEIVE_CHECK_MS: u32 = 10;
const LINK_CHECK_MS: u32 = 500;
/// Longer than the W5500 takes to give up on a destination that does not
/// answer ARP.
const SEND_TIMEOUT_MS: u32 = 250;
/// Port to which the device announces itself.
const ANNOUNCE_PORT: u16 = 35461;

type Error = w5500::Error<spi::Error>;

/// Why the Ethernet HAT is not in use.
#[derive(Debug)]
pub enum Fault {
    /// No W5500 answered: no HAT is fitted.
    NotFitted,
    /// The address or the port in the settings is not usable.
    NotConfigured,
    /// The EEPROM gave no valid MAC address.
    NoMacAddress,
    /// The details are only for the log.
    Chip(#[allow(dead_code)] Error),
}

impl From<Error> for Fault {
    fn from(error: Error) -> Self {
        Self::Chip(error)
    }
}

pub struct Ethernet {
    chip: W5500<ChipSelected>,
    interrupt: Pin<PA03, PullUpInput>,
    address: [u8; 4],
    port: u16,
    link_up: bool,
    /// Whether datagrams may still be waiting in the receive buffer.
    receiving: bool,
    last_receive_check_ms: u32,
    last_link_check_ms: u32,
}

impl Ethernet {
    /// Brings up the HAT with the address and port from the settings.
    pub fn start(mut hat: EthernetHat, mac: [u8; 6], settings: &Settings) -> Result<Self, Fault> {
        let _ = hat.reset.set_high();
        let mut chip = W5500::new(ChipSelected {
            bus: hat.spi,
            chip_select: hat.chip_select,
        });
        let start = clock::millis();
        while !chip.is_present()? {
            if clock::millis().wrapping_sub(start) > STARTUP_TIMEOUT_MS {
                return Err(Fault::NotFitted);
            }
        }

        // First octet in the least significant byte.
        let address = settings.ethernet_address().to_le_bytes();
        let port = settings.ethernet_port();
        // A blank EEPROM reads as all ones.
        if address == [0; 4] || address == [0xFF; 4] || port == 0 {
            return Err(Fault::NotConfigured);
        }
        if mac == [0x00; 6] || mac == [0xFF; 6] {
            return Err(Fault::NoMacAddress);
        }

        chip.configure(&Interface::new(mac, address))?;
        chip.open_udp(port)?;
        info!("Ethernet: listening on {}:{}", Dotted(address), port);

        Ok(Self {
            chip,
            interrupt: hat.interrupt,
            address,
            port,
            link_up: false,
            receiving: false,
            last_receive_check_ms: 0,
            last_link_check_ms: 0,
        })
    }

    /// Whether the cable is plugged in and the other end is alive.
    pub fn link_up(&self) -> bool {
        self.link_up
    }

    /// Watches the link. Call continuously from the main loop.
    pub fn poll(&mut self) {
        if !clock::every(LINK_CHECK_MS, &mut self.last_link_check_ms) {
            return;
        }
        match self.chip.link_up() {
            Ok(link_up) if link_up != self.link_up => {
                self.link_up = link_up;
                info!("Ethernet: link {}", if link_up { "up" } else { "down" });
                if link_up {
                    self.announce();
                }
            }
            Ok(_) => {}
            Err(error) => warn!("Ethernet: link not read: {:?}", error),
        }
    }

    /// Takes the next message received, together with where to send the
    /// response.
    pub fn next_request(&mut self) -> Option<(Endpoint, Result<Request, FrameError>)> {
        if !self.receiving {
            let due = clock::every(RECEIVE_CHECK_MS, &mut self.last_receive_check_ms);
            if !due && self.interrupt.is_high().unwrap() {
                return None;
            }
            // Before emptying the buffer, so that the interrupt comes back
            // for a datagram that arrives meanwhile.
            if let Err(error) = self.chip.acknowledge_received() {
                warn!("Ethernet: interrupt not acknowledged: {:?}", error);
            }
            self.receiving = true;
        }

        let mut buffer = [0; MAX_PAYLOAD];
        match self.chip.receive(&mut buffer) {
            Ok(Some(received)) => {
                let message = match buffer.get(..received.length) {
                    Some(datagram) => framing::decode_datagram(datagram),
                    // Longer than any message.
                    None => Err(FrameError::Malformed),
                };
                Some((received.from, message))
            }
            Ok(None) => {
                self.receiving = false;
                None
            }
            Err(error) => {
                warn!("Ethernet: receive failed: {:?}", error);
                self.receiving = false;
                None
            }
        }
    }

    /// Sends a datagram and waits for it to be on its way. It is dropped if
    /// its destination cannot be found.
    pub fn send(&mut self, to: Endpoint, data: &[u8]) {
        match self.try_send(to, data) {
            Ok(SendProgress::Sent) => {}
            Ok(SendProgress::Unreachable) => {
                warn!("Ethernet: {} is unreachable", Dotted(to.address));
            }
            outcome => {
                // The socket is in an unknown state: start it afresh.
                warn!("Ethernet: send failed: {:?}", outcome);
                if let Err(error) = self.chip.open_udp(self.port) {
                    warn!("Ethernet: socket not reopened: {:?}", error);
                }
            }
        }
    }

    fn try_send(&mut self, to: Endpoint, data: &[u8]) -> Result<SendProgress, Error> {
        self.chip.send(to, data)?;
        let start = clock::millis();
        loop {
            let progress = self.chip.send_progress()?;
            let timed_out = clock::millis().wrapping_sub(start) > SEND_TIMEOUT_MS;
            if progress != SendProgress::Pending || timed_out {
                return Ok(progress);
            }
        }
    }

    /// Tells the local network that the device is there, and where.
    fn announce(&mut self) {
        let mut text = String::<64>::new();
        // Cannot fail: the longest announcement fits.
        let _ = write!(
            text,
            "Axis Driver {} at {}:{}",
            FIRMWARE_VERSION,
            Dotted(self.address),
            self.port
        );
        self.send(Endpoint::broadcast(ANNOUNCE_PORT), text.as_bytes());
    }
}

/// An IPv4 address in the usual notation.
struct Dotted([u8; 4]);

impl fmt::Display for Dotted {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [a, b, c, d] = self.0;
        write!(f, "{a}.{b}.{c}.{d}")
    }
}

/// The W5500 as the only device on the auxiliary SPI bus: selected for the
/// length of each transaction.
struct ChipSelected {
    bus: AuxSpi,
    chip_select: Pin<PB01, PushPullOutput>,
}

impl ErrorType for ChipSelected {
    type Error = spi::Error;
}

impl SpiDevice for ChipSelected {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), spi::Error> {
        let _ = self.chip_select.set_low();
        let transferred = operations
            .iter_mut()
            .try_for_each(|operation| match operation {
                Operation::Read(words) => self.bus.read(words),
                Operation::Write(words) => self.bus.write(words),
                Operation::Transfer(read, write) => self.bus.transfer(read, write),
                Operation::TransferInPlace(words) => self.bus.transfer_in_place(words),
                Operation::DelayNs(ns) => {
                    Delay.delay_ns(*ns);
                    Ok(())
                }
            });
        let flushed = self.bus.flush();
        let _ = self.chip_select.set_high();
        transferred.and(flushed)
    }
}
