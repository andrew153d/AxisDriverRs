#![allow(dead_code)]

use atsamd_hal::{ehal::digital::OutputPin, gpio::Pins};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Port {
    A,
    B,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Pin {
    pub port: Port,
    pub number: u8,
}

impl Pin {
    const fn new(port: Port, number: u8) -> Self {
        Self { port, number }
    }
}

pub const AUX3: Pin = Pin::new(Port::A, 2);
pub const AUX4: Pin = Pin::new(Port::A, 3);
pub const AUX5: Pin = Pin::new(Port::B, 4);
pub const AUX6: Pin = Pin::new(Port::B, 7);
pub const AUX1: Pin = Pin::new(Port::B, 30);
pub const AUX2: Pin = Pin::new(Port::B, 31);

pub const NEOPIX: Pin = Pin::new(Port::B, 5);
pub const STAT_LED: Pin = Pin::new(Port::B, 13);

pub const ENABLE: Pin = Pin::new(Port::B, 12);
pub const DIRECTION: Pin = Pin::new(Port::B, 8);
pub const STEP: Pin = Pin::new(Port::B, 9);
pub const MOTOR_M1: Pin = Pin::new(Port::A, 10);
pub const MOTOR_M0: Pin = Pin::new(Port::A, 11);
pub const INDEX: Pin = Pin::new(Port::A, 8);
pub const DIAGNOSTIC: Pin = Pin::new(Port::A, 9);
pub const SPREAD: Pin = Pin::new(Port::B, 11);
pub const UART_TX: Pin = Pin::new(Port::A, 4);
pub const UART_RX: Pin = Pin::new(Port::A, 7);

pub const SDA2: Pin = Pin::new(Port::A, 12);
pub const SCL2: Pin = Pin::new(Port::A, 13);
pub const SDA1: Pin = Pin::new(Port::A, 22);
pub const SCL1: Pin = Pin::new(Port::A, 23);
pub const SYNC: Pin = Pin::new(Port::A, 14);

pub const EEPROM_CS: Pin = Pin::new(Port::A, 21);
pub const ETHERNET_CS: Pin = Pin::new(Port::B, 1);
pub const MOSI: Pin = Pin::new(Port::B, 2);
pub const MISO: Pin = Pin::new(Port::B, 0);
pub const SCK: Pin = Pin::new(Port::B, 3);

pub const USB_VOLTAGE: Pin = Pin::new(Port::A, 6);
pub const BUS_VOLTAGE: Pin = Pin::new(Port::B, 6);
pub const USER_INPUT: Pin = Pin::new(Port::A, 15);

pub trait AxisDriverPins {
    fn board_outputs(self) -> (impl OutputPin, impl OutputPin);
}

impl AxisDriverPins for Pins {
    fn board_outputs(self) -> (impl OutputPin, impl OutputPin) {
        (
            self.pb13.into_push_pull_output(),
            self.pb05.into_push_pull_output(),
        )
    }
}
