//! Driver for the WIZnet W5500 Ethernet controller, as far as it takes to
//! exchange UDP datagrams.
//!
//! The chip runs the whole network stack itself, including ARP. Only its
//! first socket is used.

use embedded_hal::spi::{Operation, SpiDevice};

/// Bytes each buffer of a socket holds after a reset.
const BUFFER_SIZE: u16 = 2048;
/// What the chip puts in front of each received datagram: the sender's
/// address and port and the length of the data.
const RECEIVE_HEADER_SIZE: u16 = 8;
/// Largest datagram that can be sent or received.
pub const MAX_DATAGRAM: usize = (BUFFER_SIZE - RECEIVE_HEADER_SIZE) as usize;

/// Unanswered ARP requests are repeated at this interval, in units of 100 us.
const RETRY_TIME: u16 = 500;
/// Number of repeats before a datagram is given up as undeliverable.
const RETRY_COUNT: u8 = 2;

/// How often a register is read while waiting for the chip to act on it.
/// Commands take effect within microseconds.
const WAIT_ATTEMPTS: usize = 1_000;

/// The areas of the chip's memory, as selected in the control byte.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Block {
    Common = 0,
    Socket = 1,
    Transmit = 2,
    Receive = 3,
}

const CONTROL_WRITE: u8 = 0x04;

/// Registers shared by all sockets.
mod common {
    pub const MODE: u16 = 0x0000;
    pub const GATEWAY: u16 = 0x0001;
    pub const SUBNET: u16 = 0x0005;
    pub const MAC: u16 = 0x0009;
    pub const ADDRESS: u16 = 0x000F;
    pub const SOCKET_INTERRUPT_MASK: u16 = 0x0018;
    pub const RETRY_TIME: u16 = 0x0019;
    pub const RETRY_COUNT: u16 = 0x001B;
    pub const PHY: u16 = 0x002E;
    pub const VERSION: u16 = 0x0039;

    pub const MODE_RESET: u8 = 0x80;
    pub const PHY_LINK_UP: u8 = 0x01;
    /// What every W5500 holds in its version register.
    pub const VERSION_W5500: u8 = 0x04;
}

/// Registers of one socket.
mod socket {
    pub const MODE: u16 = 0x0000;
    pub const COMMAND: u16 = 0x0001;
    pub const INTERRUPT: u16 = 0x0002;
    pub const STATUS: u16 = 0x0003;
    pub const PORT: u16 = 0x0004;
    pub const DESTINATION_ADDRESS: u16 = 0x000C;
    pub const DESTINATION_PORT: u16 = 0x0010;
    pub const TRANSMIT_FREE: u16 = 0x0020;
    pub const TRANSMIT_WRITE: u16 = 0x0024;
    pub const RECEIVED: u16 = 0x0026;
    pub const RECEIVE_READ: u16 = 0x0028;
    pub const INTERRUPT_MASK: u16 = 0x002C;

    pub const MODE_UDP: u8 = 0x02;
    pub const STATUS_UDP: u8 = 0x22;

    pub const COMMAND_OPEN: u8 = 0x01;
    pub const COMMAND_CLOSE: u8 = 0x10;
    pub const COMMAND_SEND: u8 = 0x20;
    pub const COMMAND_RECEIVED: u8 = 0x40;

    pub const INTERRUPT_RECEIVED: u8 = 0x04;
    pub const INTERRUPT_TIMEOUT: u8 = 0x08;
    pub const INTERRUPT_SENT: u8 = 0x10;
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error<E> {
    Bus(E),
    /// The chip is not there, or does not behave like a W5500.
    Unresponsive,
    /// The datagram does not fit in the transmit buffer.
    NoRoom,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Self::Bus(error)
    }
}

/// The chip's identity on the network.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Interface {
    pub mac: [u8; 6],
    pub address: [u8; 4],
    pub subnet: [u8; 4],
    pub gateway: [u8; 4],
}

impl Interface {
    /// Assumes what the original firmware did: a network of 254 hosts whose
    /// gateway is host number 1.
    pub fn new(mac: [u8; 6], address: [u8; 4]) -> Self {
        Self {
            mac,
            address,
            subnet: [255, 255, 255, 0],
            gateway: [address[0], address[1], address[2], 1],
        }
    }
}

/// Where a datagram comes from or goes to.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Endpoint {
    pub address: [u8; 4],
    pub port: u16,
}

impl Endpoint {
    /// Reaches every host on the local network.
    pub const fn broadcast(port: u16) -> Self {
        Self {
            address: [255; 4],
            port,
        }
    }
}

/// A received datagram.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Received {
    pub from: Endpoint,
    /// Length of the datagram. If it is larger than the buffer it was
    /// received into, the buffer only holds the beginning.
    pub length: usize,
}

/// What became of the datagram last sent.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SendProgress {
    /// Still being sent, or waiting for the destination to answer ARP.
    Pending,
    Sent,
    /// Nobody answered for the destination's address; the datagram is lost.
    Unreachable,
}

pub struct W5500<S> {
    spi: S,
}

impl<S: SpiDevice> W5500<S> {
    pub fn new(spi: S) -> Self {
        Self { spi }
    }

    /// Whether a W5500 answers on the bus.
    pub fn is_present(&mut self) -> Result<bool, Error<S::Error>> {
        Ok(self.read_u8(Block::Common, common::VERSION)? == common::VERSION_W5500)
    }

    /// Resets the chip and gives it its addresses.
    pub fn configure(&mut self, interface: &Interface) -> Result<(), Error<S::Error>> {
        self.write_u8(Block::Common, common::MODE, common::MODE_RESET)?;
        self.wait_until(Block::Common, common::MODE, |mode| {
            mode & common::MODE_RESET == 0
        })?;

        self.write(Block::Common, common::GATEWAY, &interface.gateway)?;
        self.write(Block::Common, common::SUBNET, &interface.subnet)?;
        self.write(Block::Common, common::MAC, &interface.mac)?;
        self.write(Block::Common, common::ADDRESS, &interface.address)?;
        self.write_u16(Block::Common, common::RETRY_TIME, RETRY_TIME)?;
        self.write_u8(Block::Common, common::RETRY_COUNT, RETRY_COUNT)?;

        // A bus with nothing on it reads the same whatever is written.
        let mut mac = [0; 6];
        self.read(Block::Common, common::MAC, &mut mac)?;
        if mac != interface.mac || !self.is_present()? {
            return Err(Error::Unresponsive);
        }
        Ok(())
    }

    /// Whether the cable is plugged in and the other end is alive.
    pub fn link_up(&mut self) -> Result<bool, Error<S::Error>> {
        Ok(self.read_u8(Block::Common, common::PHY)? & common::PHY_LINK_UP != 0)
    }

    /// Starts listening for datagrams on `port`, dropping anything still
    /// waiting to be received or sent. The interrupt output is then low
    /// from when a datagram arrives until [`Self::acknowledge_received`].
    pub fn open_udp(&mut self, port: u16) -> Result<(), Error<S::Error>> {
        self.command(socket::COMMAND_CLOSE)?;
        self.write_u8(Block::Socket, socket::INTERRUPT, 0xFF)?;
        self.write_u8(Block::Socket, socket::MODE, socket::MODE_UDP)?;
        self.write_u16(Block::Socket, socket::PORT, port)?;
        self.command(socket::COMMAND_OPEN)?;
        self.wait_until(Block::Socket, socket::STATUS, |status| {
            status == socket::STATUS_UDP
        })?;

        self.write_u8(
            Block::Socket,
            socket::INTERRUPT_MASK,
            socket::INTERRUPT_RECEIVED,
        )?;
        self.write_u8(Block::Common, common::SOCKET_INTERRUPT_MASK, 0x01)?;
        Ok(())
    }

    /// Releases the interrupt output. Call before taking the datagrams with
    /// [`Self::receive`], so that one arriving in between is not missed.
    pub fn acknowledge_received(&mut self) -> Result<(), Error<S::Error>> {
        self.write_u8(Block::Socket, socket::INTERRUPT, socket::INTERRUPT_RECEIVED)?;
        Ok(())
    }

    /// Takes the oldest datagram waiting, if there is one, and stores as much
    /// of it as fits in `buffer`.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<Option<Received>, Error<S::Error>> {
        let waiting = self.read_changing_u16(socket::RECEIVED)?;
        if waiting < RECEIVE_HEADER_SIZE {
            return Ok(None);
        }
        let pointer = self.read_u16(Block::Socket, socket::RECEIVE_READ)?;

        let mut header = [0; RECEIVE_HEADER_SIZE as usize];
        self.read_buffer(pointer, &mut header)?;
        let [a, b, c, d, port_high, port_low, length_high, length_low] = header;
        let length = u16::from_be_bytes([length_high, length_low]);

        let received = if length <= waiting - RECEIVE_HEADER_SIZE {
            let stored = buffer.len().min(usize::from(length));
            self.read_buffer(
                pointer.wrapping_add(RECEIVE_HEADER_SIZE),
                &mut buffer[..stored],
            )?;
            self.finish_receiving(pointer.wrapping_add(RECEIVE_HEADER_SIZE + length))?;
            Some(Received {
                from: Endpoint {
                    address: [a, b, c, d],
                    port: u16::from_be_bytes([port_high, port_low]),
                },
                length: usize::from(length),
            })
        } else {
            // The header makes no sense, so there is no telling where the
            // next datagram starts: drop everything that is waiting.
            self.finish_receiving(pointer.wrapping_add(waiting))?;
            None
        };
        Ok(received)
    }

    /// Starts sending a datagram. The previous one must have left
    /// [`SendProgress::Pending`].
    pub fn send(&mut self, to: Endpoint, data: &[u8]) -> Result<(), Error<S::Error>> {
        let free = self.read_changing_u16(socket::TRANSMIT_FREE)?;
        if data.len() > usize::from(free) {
            return Err(Error::NoRoom);
        }
        let pointer = self.read_u16(Block::Socket, socket::TRANSMIT_WRITE)?;
        self.write_buffer(pointer, data)?;
        self.write_u16(
            Block::Socket,
            socket::TRANSMIT_WRITE,
            pointer.wrapping_add(data.len() as u16),
        )?;
        self.write(Block::Socket, socket::DESTINATION_ADDRESS, &to.address)?;
        self.write_u16(Block::Socket, socket::DESTINATION_PORT, to.port)?;
        self.command(socket::COMMAND_SEND)
    }

    /// Checks on the datagram last sent. An outcome is reported once.
    pub fn send_progress(&mut self) -> Result<SendProgress, Error<S::Error>> {
        let interrupt = self.read_u8(Block::Socket, socket::INTERRUPT)?;
        let (flag, progress) = if interrupt & socket::INTERRUPT_SENT != 0 {
            (socket::INTERRUPT_SENT, SendProgress::Sent)
        } else if interrupt & socket::INTERRUPT_TIMEOUT != 0 {
            (socket::INTERRUPT_TIMEOUT, SendProgress::Unreachable)
        } else {
            return Ok(SendProgress::Pending);
        };
        self.write_u8(Block::Socket, socket::INTERRUPT, flag)?;
        Ok(progress)
    }

    fn finish_receiving(&mut self, pointer: u16) -> Result<(), Error<S::Error>> {
        self.write_u16(Block::Socket, socket::RECEIVE_READ, pointer)?;
        self.command(socket::COMMAND_RECEIVED)
    }

    /// Gives the socket a command and waits for the chip to accept it.
    fn command(&mut self, command: u8) -> Result<(), Error<S::Error>> {
        self.write_u8(Block::Socket, socket::COMMAND, command)?;
        self.wait_until(Block::Socket, socket::COMMAND, |command| command == 0)
    }

    fn wait_until(
        &mut self,
        block: Block,
        address: u16,
        done: impl Fn(u8) -> bool,
    ) -> Result<(), Error<S::Error>> {
        for _ in 0..WAIT_ATTEMPTS {
            if done(self.read_u8(block, address)?) {
                return Ok(());
            }
        }
        Err(Error::Unresponsive)
    }

    /// Reads from the receive buffer, which is a ring addressed by a
    /// free-running pointer.
    fn read_buffer(&mut self, pointer: u16, data: &mut [u8]) -> Result<(), S::Error> {
        let (offset, first) = ring_span(pointer, data.len());
        let (head, tail) = data.split_at_mut(first);
        self.read(Block::Receive, offset, head)?;
        if !tail.is_empty() {
            self.read(Block::Receive, 0, tail)?;
        }
        Ok(())
    }

    /// Writes to the transmit buffer, a ring like the receive buffer.
    fn write_buffer(&mut self, pointer: u16, data: &[u8]) -> Result<(), S::Error> {
        let (offset, first) = ring_span(pointer, data.len());
        let (head, tail) = data.split_at(first);
        self.write(Block::Transmit, offset, head)?;
        if !tail.is_empty() {
            self.write(Block::Transmit, 0, tail)?;
        }
        Ok(())
    }

    /// Reads a 16-bit register that the chip may update between the two
    /// bytes being read: repeats until two readings agree.
    fn read_changing_u16(&mut self, address: u16) -> Result<u16, Error<S::Error>> {
        let mut previous = self.read_u16(Block::Socket, address)?;
        for _ in 0..WAIT_ATTEMPTS {
            let value = self.read_u16(Block::Socket, address)?;
            if value == previous {
                return Ok(value);
            }
            previous = value;
        }
        Err(Error::Unresponsive)
    }

    fn read_u8(&mut self, block: Block, address: u16) -> Result<u8, S::Error> {
        let mut value = [0];
        self.read(block, address, &mut value)?;
        Ok(value[0])
    }

    fn read_u16(&mut self, block: Block, address: u16) -> Result<u16, S::Error> {
        let mut value = [0; 2];
        self.read(block, address, &mut value)?;
        Ok(u16::from_be_bytes(value))
    }

    fn write_u8(&mut self, block: Block, address: u16, value: u8) -> Result<(), S::Error> {
        self.write(block, address, &[value])
    }

    fn write_u16(&mut self, block: Block, address: u16, value: u16) -> Result<(), S::Error> {
        self.write(block, address, &value.to_be_bytes())
    }

    fn read(&mut self, block: Block, address: u16, data: &mut [u8]) -> Result<(), S::Error> {
        let header = header(block, address, 0);
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Read(data)])
    }

    fn write(&mut self, block: Block, address: u16, data: &[u8]) -> Result<(), S::Error> {
        let header = header(block, address, CONTROL_WRITE);
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Write(data)])
    }
}

/// The bytes that start every transfer: the address, then the control byte
/// selecting the block of the first socket and the direction.
fn header(block: Block, address: u16, direction: u8) -> [u8; 3] {
    let [high, low] = address.to_be_bytes();
    [high, low, (block as u8) << 3 | direction]
}

/// Where in a buffer `pointer` points, and how many of `length` bytes fit
/// between there and the end of the buffer; the rest continue at its start.
fn ring_span(pointer: u16, length: usize) -> (u16, usize) {
    let offset = pointer % BUFFER_SIZE;
    (offset, length.min(usize::from(BUFFER_SIZE - offset)))
}

/// A simulated W5500 for tests.
#[cfg(test)]
pub(crate) mod fake {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::spi::ErrorType;
    use std::vec::Vec;

    const SIZE: usize = BUFFER_SIZE as usize;
    const TRANSMIT_READ: u16 = 0x0022;
    const RECEIVE_WRITE: u16 = 0x002A;

    pub struct FakeChip {
        /// Whether the chip is on the bus at all.
        pub present: bool,
        /// Whether destinations answer ARP.
        pub reachable: bool,
        pub link: bool,
        /// Datagrams sent, with their destinations.
        pub sent: Vec<(Endpoint, Vec<u8>)>,
        common: [u8; 0x40],
        socket: [u8; 0x30],
        transmit: [u8; SIZE],
        receive: [u8; SIZE],
    }

    impl FakeChip {
        pub fn new() -> Self {
            let mut chip = Self {
                present: true,
                reachable: true,
                link: true,
                sent: Vec::new(),
                common: [0; 0x40],
                socket: [0; 0x30],
                transmit: [0; SIZE],
                receive: [0; SIZE],
            };
            chip.reset();
            chip
        }

        pub fn absent() -> Self {
            Self {
                present: false,
                ..Self::new()
            }
        }

        /// Puts a datagram in the receive buffer, as if it had arrived from
        /// the network.
        pub fn deliver(&mut self, from: Endpoint, data: &[u8]) {
            assert_eq!(self.socket[socket::STATUS as usize], socket::STATUS_UDP);
            let mut packet = Vec::from(from.address);
            packet.extend_from_slice(&from.port.to_be_bytes());
            packet.extend_from_slice(&(data.len() as u16).to_be_bytes());
            packet.extend_from_slice(data);

            let waiting = self.socket_u16(socket::RECEIVED);
            assert!(usize::from(waiting) + packet.len() <= SIZE, "buffer full");
            let pointer = self.socket_u16(RECEIVE_WRITE);
            for (index, &byte) in packet.iter().enumerate() {
                self.receive[(usize::from(pointer) + index) % SIZE] = byte;
            }
            let length = packet.len() as u16;
            self.set_socket_u16(RECEIVE_WRITE, pointer.wrapping_add(length));
            self.set_socket_u16(socket::RECEIVED, waiting + length);
            self.socket[socket::INTERRUPT as usize] |= socket::INTERRUPT_RECEIVED;
        }

        /// Overwrites the start of the receive buffer and claims `waiting`
        /// bytes are in it.
        pub fn corrupt_receive_buffer(&mut self, bytes: &[u8], waiting: u16) {
            self.receive[..bytes.len()].copy_from_slice(bytes);
            self.set_socket_u16(RECEIVE_WRITE, waiting);
            self.set_socket_u16(socket::RECEIVED, waiting);
        }

        /// State of the interrupt output; low is active.
        pub fn interrupt_is_low(&self) -> bool {
            let socket_enabled = self.common[common::SOCKET_INTERRUPT_MASK as usize] & 0x01 != 0;
            let pending = self.socket[socket::INTERRUPT as usize]
                & self.socket[socket::INTERRUPT_MASK as usize];
            socket_enabled && pending != 0
        }

        pub fn common(&self, address: u16, length: usize) -> &[u8] {
            &self.common[usize::from(address)..][..length]
        }

        pub fn port(&self) -> u16 {
            self.socket_u16(socket::PORT)
        }

        fn reset(&mut self) {
            self.common = [0; 0x40];
            self.socket = [0; 0x30];
            self.common[common::VERSION as usize] = common::VERSION_W5500;
            self.set_socket_u16(socket::TRANSMIT_FREE, BUFFER_SIZE);
        }

        fn socket_u16(&self, address: u16) -> u16 {
            let address = usize::from(address);
            u16::from_be_bytes([self.socket[address], self.socket[address + 1]])
        }

        fn set_socket_u16(&mut self, address: u16, value: u16) {
            self.socket[usize::from(address)..][..2].copy_from_slice(&value.to_be_bytes());
        }

        fn read_byte(&self, block: u8, address: u16) -> u8 {
            let address = usize::from(address);
            match block {
                0 if address == usize::from(common::PHY) => u8::from(self.link),
                0 => self.common[address],
                1 => self.socket[address],
                3 => self.receive[address],
                _ => panic!("read from block {block}"),
            }
        }

        fn write_byte(&mut self, block: u8, address: u16, value: u8) {
            match (block, address) {
                (0, common::MODE) if value & common::MODE_RESET != 0 => self.reset(),
                (0, common::VERSION) => {}
                (0, _) => self.common[usize::from(address)] = value,
                (1, socket::COMMAND) => self.run(value),
                // Flags are cleared by writing ones.
                (1, socket::INTERRUPT) => self.socket[usize::from(address)] &= !value,
                (1, _) => self.socket[usize::from(address)] = value,
                (2, _) => self.transmit[usize::from(address)] = value,
                _ => panic!("write to block {block}"),
            }
        }

        fn run(&mut self, command: u8) {
            match command {
                socket::COMMAND_OPEN => {
                    assert_eq!(self.socket[socket::MODE as usize], socket::MODE_UDP);
                    self.socket[socket::STATUS as usize] = socket::STATUS_UDP;
                    for pointer in [
                        TRANSMIT_READ,
                        socket::TRANSMIT_WRITE,
                        socket::RECEIVED,
                        socket::RECEIVE_READ,
                        RECEIVE_WRITE,
                    ] {
                        self.set_socket_u16(pointer, 0);
                    }
                    self.set_socket_u16(socket::TRANSMIT_FREE, BUFFER_SIZE);
                }
                socket::COMMAND_CLOSE => self.socket[socket::STATUS as usize] = 0,
                socket::COMMAND_SEND => {
                    let start = self.socket_u16(TRANSMIT_READ);
                    let end = self.socket_u16(socket::TRANSMIT_WRITE);
                    let data = (0..end.wrapping_sub(start))
                        .map(|index| self.transmit[usize::from(start.wrapping_add(index)) % SIZE])
                        .collect();
                    self.set_socket_u16(TRANSMIT_READ, end);
                    if self.reachable {
                        let address = usize::from(socket::DESTINATION_ADDRESS);
                        let to = Endpoint {
                            address: self.socket[address..address + 4].try_into().unwrap(),
                            port: self.socket_u16(socket::DESTINATION_PORT),
                        };
                        self.sent.push((to, data));
                        self.socket[socket::INTERRUPT as usize] |= socket::INTERRUPT_SENT;
                    } else {
                        self.socket[socket::INTERRUPT as usize] |= socket::INTERRUPT_TIMEOUT;
                    }
                }
                socket::COMMAND_RECEIVED => {
                    let waiting = self
                        .socket_u16(RECEIVE_WRITE)
                        .wrapping_sub(self.socket_u16(socket::RECEIVE_READ));
                    self.set_socket_u16(socket::RECEIVED, waiting);
                }
                _ => panic!("unknown command {command:#x}"),
            }
        }
    }

    impl ErrorType for FakeChip {
        type Error = Infallible;
    }

    impl SpiDevice for FakeChip {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Infallible> {
            let [Operation::Write(header), data] = operations else {
                panic!("expected a header and one data phase");
            };
            let &[high, low, control] = &header[..] else {
                panic!("the header is three bytes");
            };
            let mut address = u16::from_be_bytes([high, low]);
            let block = control >> 3;
            let writing = control & CONTROL_WRITE != 0;
            // Variable length mode.
            assert_eq!(control & 0x03, 0);

            match data {
                Operation::Read(buffer) if !writing => {
                    for byte in buffer.iter_mut() {
                        // An undriven data line reads as zeros.
                        *byte = if self.present {
                            self.read_byte(block, address)
                        } else {
                            0
                        };
                        address += 1;
                    }
                }
                Operation::Write(bytes) if writing => {
                    for &byte in bytes.iter() {
                        if self.present {
                            self.write_byte(block, address, byte);
                        }
                        address += 1;
                    }
                }
                _ => panic!("data phase does not match the control byte"),
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::FakeChip;
    use super::*;

    const MAC: [u8; 6] = [0xFC, 0xC2, 0x3D, 0x01, 0x02, 0x03];
    const HOST: Endpoint = Endpoint {
        address: [192, 168, 1, 20],
        port: 50_000,
    };

    fn started() -> W5500<FakeChip> {
        let mut chip = W5500::new(FakeChip::new());
        chip.configure(&Interface::new(MAC, [192, 168, 1, 50]))
            .unwrap();
        chip.open_udp(12_000).unwrap();
        chip
    }

    #[test]
    fn configures_the_addresses() {
        let chip = started();
        assert_eq!(chip.spi.common(common::GATEWAY, 4), [192, 168, 1, 1]);
        assert_eq!(chip.spi.common(common::SUBNET, 4), [255, 255, 255, 0]);
        assert_eq!(chip.spi.common(common::MAC, 6), MAC);
        assert_eq!(chip.spi.common(common::ADDRESS, 4), [192, 168, 1, 50]);
        assert_eq!(chip.spi.port(), 12_000);
    }

    #[test]
    fn notices_a_missing_chip() {
        let mut chip = W5500::new(FakeChip::absent());
        assert_eq!(chip.is_present(), Ok(false));
        assert_eq!(
            chip.configure(&Interface::new(MAC, [192, 168, 1, 50])),
            Err(Error::Unresponsive)
        );
        assert_eq!(W5500::new(FakeChip::new()).is_present(), Ok(true));
    }

    #[test]
    fn reports_the_link() {
        let mut chip = started();
        assert_eq!(chip.link_up(), Ok(true));
        chip.spi.link = false;
        assert_eq!(chip.link_up(), Ok(false));
    }

    #[test]
    fn receives_datagrams_in_order() {
        let mut chip = started();
        let mut buffer = [0; 64];
        assert_eq!(chip.receive(&mut buffer), Ok(None));

        let other = Endpoint {
            address: [10, 0, 0, 7],
            port: 9,
        };
        chip.spi.deliver(HOST, b"first");
        chip.spi.deliver(other, b"");
        chip.spi.deliver(HOST, b"third");

        let received = chip.receive(&mut buffer).unwrap().unwrap();
        assert_eq!(received.from, HOST);
        assert_eq!(&buffer[..received.length], b"first");
        assert_eq!(
            chip.receive(&mut buffer),
            Ok(Some(Received {
                from: other,
                length: 0
            }))
        );
        let received = chip.receive(&mut buffer).unwrap().unwrap();
        assert_eq!(&buffer[..received.length], b"third");
        assert_eq!(chip.receive(&mut buffer), Ok(None));
    }

    #[test]
    fn the_interrupt_follows_received_datagrams() {
        let mut chip = started();
        assert!(!chip.spi.interrupt_is_low());
        chip.spi.deliver(HOST, b"x");
        assert!(chip.spi.interrupt_is_low());
        chip.acknowledge_received().unwrap();
        assert!(!chip.spi.interrupt_is_low());

        // Sending does not raise it.
        chip.send(HOST, b"y").unwrap();
        assert!(!chip.spi.interrupt_is_low());
    }

    #[test]
    fn an_oversized_datagram_is_cut_short_and_skipped() {
        let mut chip = started();
        chip.spi.deliver(HOST, b"0123456789");
        chip.spi.deliver(HOST, b"next");

        let mut buffer = [0; 4];
        let received = chip.receive(&mut buffer).unwrap().unwrap();
        assert_eq!(received.length, 10);
        assert_eq!(&buffer, b"0123");
        assert_eq!(chip.receive(&mut buffer).unwrap().unwrap().length, 4);
        assert_eq!(&buffer, b"next");
    }

    #[test]
    fn a_senseless_header_empties_the_buffer() {
        let mut chip = started();
        // Claims 4096 bytes of data where 12 bytes are waiting.
        chip.spi
            .corrupt_receive_buffer(&[1, 2, 3, 4, 0, 5, 0x10, 0x00], 12);
        let mut buffer = [0; 64];
        assert_eq!(chip.receive(&mut buffer), Ok(None));

        chip.spi.deliver(HOST, b"fine");
        let received = chip.receive(&mut buffer).unwrap().unwrap();
        assert_eq!(&buffer[..received.length], b"fine");
    }

    #[test]
    fn sends_datagrams() {
        let mut chip = started();
        assert_eq!(chip.send_progress(), Ok(SendProgress::Pending));
        chip.send(HOST, b"hello").unwrap();
        assert_eq!(chip.send_progress(), Ok(SendProgress::Sent));
        assert_eq!(chip.send_progress(), Ok(SendProgress::Pending));

        chip.send(Endpoint::broadcast(35_461), b"").unwrap();
        assert_eq!(
            chip.spi.sent,
            [
                (HOST, b"hello".to_vec()),
                (Endpoint::broadcast(35_461), std::vec![]),
            ]
        );
    }

    #[test]
    fn reports_an_unreachable_destination() {
        let mut chip = started();
        chip.spi.reachable = false;
        chip.send(HOST, b"hello").unwrap();
        assert_eq!(chip.send_progress(), Ok(SendProgress::Unreachable));
        assert_eq!(chip.send_progress(), Ok(SendProgress::Pending));

        chip.spi.reachable = true;
        chip.send(HOST, b"again").unwrap();
        assert_eq!(chip.send_progress(), Ok(SendProgress::Sent));
        assert_eq!(chip.spi.sent, [(HOST, b"again".to_vec())]);
    }

    #[test]
    fn refuses_a_datagram_larger_than_the_buffer() {
        let mut chip = started();
        assert_eq!(chip.send(HOST, &[0; 2049]), Err(Error::NoRoom));
        assert!(chip.spi.sent.is_empty());
    }

    #[test]
    fn data_wraps_around_the_ends_of_the_buffers() {
        let mut chip = started();
        let mut buffer = [0; 700];
        // Seven times 700 bytes is more than twice around either buffer.
        for round in 0..7u8 {
            let data: std::vec::Vec<u8> = (0..700u32)
                .map(|index| (index as u8).wrapping_mul(7).wrapping_add(round))
                .collect();

            chip.spi.deliver(HOST, &data);
            let received = chip.receive(&mut buffer).unwrap().unwrap();
            assert_eq!(received.length, data.len());
            assert_eq!(buffer[..], data[..]);

            chip.send(HOST, &data).unwrap();
            assert_eq!(chip.send_progress(), Ok(SendProgress::Sent));
            assert_eq!(chip.spi.sent.last().unwrap().1, data);
        }
    }

    #[test]
    fn reopening_drops_what_was_waiting() {
        let mut chip = started();
        chip.spi.deliver(HOST, b"stale");
        chip.open_udp(12_001).unwrap();
        assert_eq!(chip.spi.port(), 12_001);
        assert!(!chip.spi.interrupt_is_low());
        assert_eq!(chip.receive(&mut [0; 16]), Ok(None));
    }
}
