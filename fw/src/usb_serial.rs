//! The USB serial port, the board's main link to the host.

use atsamd_hal::usb::{
    usb_device::{
        bus::UsbBusAllocator,
        device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
        UsbError,
    },
    UsbBus,
};
use axis_core::protocol::{FrameBuffer, ParseError, Request};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::bootloader;
use crate::clock;

// Same identity as the original Arduino-based firmware.
const VID_PID: UsbVidPid = UsbVidPid(0x239A, 0x8031);
const MANUFACTURER: &str = "Grow Engn";
const PRODUCT: &str = "Grow Engn Axis Basic";

/// A partial frame is dropped if the rest does not arrive within this time.
const FRAME_TIMEOUT_MS: u32 = 100;
/// How long to wait for the host to take a reply before giving up on it.
const SEND_TIMEOUT_MS: u32 = 20;

pub struct UsbSerial {
    device: UsbDevice<'static, UsbBus>,
    port: SerialPort<'static, UsbBus>,
    frames: FrameBuffer,
    last_receive_ms: u32,
}

impl UsbSerial {
    pub fn new(bus: &'static UsbBusAllocator<UsbBus>) -> Self {
        // The class has to be created before the device.
        let port = SerialPort::new(bus);
        let device = UsbDeviceBuilder::new(bus, VID_PID)
            .strings(&[StringDescriptors::default()
                .manufacturer(MANUFACTURER)
                .product(PRODUCT)])
            .expect("too many USB string descriptors")
            .device_class(USB_CLASS_CDC)
            .build();

        Self {
            device,
            port,
            frames: FrameBuffer::new(),
            last_receive_ms: 0,
        }
    }

    /// Whether the host has asked for the bootloader, by setting the magic
    /// baud rate and closing the port.
    pub fn bootloader_requested(&self) -> bool {
        self.port.line_coding().data_rate() == bootloader::TOUCH_BAUD_RATE && !self.port.dtr()
    }

    /// Whether a host has opened the port.
    pub fn is_connected(&self) -> bool {
        self.port.dtr()
    }

    /// Services the USB bus and collects received bytes. Must be called
    /// every few milliseconds.
    pub fn poll(&mut self) {
        let now = clock::millis();
        self.device.poll(&mut [&mut self.port]);
        let mut buffer = [0; 64];
        while let Ok(count @ 1..) = self.port.read(&mut buffer) {
            self.frames.extend(&buffer[..count]);
            self.last_receive_ms = now;
        }
        if !self.frames.is_empty() && now.wrapping_sub(self.last_receive_ms) > FRAME_TIMEOUT_MS {
            self.frames.clear();
        }
    }

    /// Takes the next message received from the host.
    pub fn next_request(&mut self) -> Option<Result<Request, ParseError>> {
        self.frames.next_request()
    }

    /// Sends a frame to the host. It is dropped if no host is reading.
    pub fn send(&mut self, mut bytes: &[u8]) {
        let start = clock::millis();
        while !bytes.is_empty() && clock::millis().wrapping_sub(start) < SEND_TIMEOUT_MS {
            match self.port.write(bytes) {
                Ok(count) => bytes = &bytes[count..],
                Err(UsbError::WouldBlock) => {}
                Err(_) => return,
            }
            self.device.poll(&mut [&mut self.port]);
        }
    }
}
