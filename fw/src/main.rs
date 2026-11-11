//! Firmware for the Axis Driver, a SAMD51 based closed-loop stepper driver.
//!
//! The hardware-independent logic lives in the `axis-core` crate; this crate
//! connects it to the board:
//!
//! - [`board`]: pin assignments, clocks and peripherals
//! - [`motor`], [`step_timer`]: step generation in a timer interrupt, driver
//!   configuration
//! - [`encoder`], [`led`]: the other on-board devices
//! - [`usb_serial`], [`ethernet`]: the links to the host
//! - [`logger`]: log output over RTT and, on request, to the host
//! - [`bootloader`]: handing over to the USB bootloader for firmware uploads
//! - [`commands`]: execution of host requests

#![no_std]
#![no_main]

mod axis;
mod board;
mod bootloader;
mod clock;
mod commands;
mod encoder;
mod ethernet;
mod led;
mod logger;
mod motor;
mod step_timer;
mod tmc2209;
mod usb_serial;
mod ws2812;

use atsamd_hal::ehal::digital::{InputPin, OutputPin};
use atsamd_hal::usb::{usb_device::bus::UsbBusAllocator, UsbBus};
use axis_core::framing::{self, FrameError};
use axis_core::proto::{Request, Response, FIRMWARE_VERSION};
use axis_core::types::Rgb;
use cortex_m_rt::entry;
use log::{error, info};
use panic_halt as _;

use axis::Axis;
use board::{Board, StatusLed, UserButton};
use ethernet::Ethernet;
use led::StatusPixel;
use motor::Motor;
use tmc2209::Tmc2209;
use usb_serial::UsbSerial;
use ws2812::Ws2812;

const BOOT_COLOR: Rgb = Rgb::new(0x00, 0x11, 0x00);
const WAITING_FOR_HOST_COLOR: Rgb = Rgb::new(0x00, 0x00, 0xF0);
/// How long the user button is watched for after power-up.
const BOOT_BUTTON_WINDOW_MS: u32 = 1_500;
const WAITING_FOR_HOST_BLINK_MS: u32 = 80;
/// Shown once the Ethernet HAT is up, or in red if it could not be started.
const ETHERNET_FLASH_MS: u32 = 100;

#[entry]
fn main() -> ! {
    logger::init();
    let mut board = Board::take();
    info!("Axis Driver firmware {} starting", FIRMWARE_VERSION);

    let usb_bus = cortex_m::singleton!(: UsbBusAllocator<UsbBus> = board.usb).unwrap();
    let mut usb = UsbSerial::new(usb_bus);
    let mut pixel = StatusPixel::new(Ws2812::new(board.pixel));

    pixel.set_color(BOOT_COLOR);
    if button_pressed_at_boot(&mut board.user_button, &mut usb) {
        wait_for_host(&mut usb, &mut pixel, &mut board.status_led);
    }
    pixel.set_color(Rgb::BLACK);

    let motor = Motor::start(board.motor, board.step_timer, Tmc2209::new(board.tmc_uart));
    let mut axis = Axis::start(motor, pixel, board.i2c, board.status_led);
    let mut ethernet = start_ethernet(board.ethernet, &mut axis);
    info!("Axis Driver running");

    loop {
        usb.poll();
        if usb.bootloader_requested() {
            info!("Entering the bootloader");
            bootloader::enter();
        }
        while let Some(message) = usb.next_request() {
            match answer(&mut axis, message).as_ref().map(framing::encode) {
                Some(Ok(frame)) => {
                    axis.pixel.flash_activity(Rgb::BLUE);
                    usb.send(frame.as_bytes());
                }
                Some(Err(error)) => error!("Response not sent: {:?}", error),
                None => {}
            }
        }
        if let Some(ethernet) = &mut ethernet {
            ethernet.poll();
            axis.ethernet_link = Some(ethernet.link_up());
            while let Some((sender, message)) = ethernet.next_request() {
                let response = answer(&mut axis, message);
                match response.as_ref().map(framing::encode_datagram) {
                    Some(Ok(datagram)) => {
                        axis.pixel.flash_activity(Rgb::BLUE);
                        ethernet.send(sender, &datagram);
                    }
                    Some(Err(error)) => error!("Response not sent: {:?}", error),
                    None => {}
                }
            }
        }
        axis.poll();
    }
}

/// Carries out a request from the host and builds the response, if it gets
/// one.
fn answer(axis: &mut Axis, message: Result<Request, FrameError>) -> Option<Response> {
    axis.pixel.flash_activity(Rgb::GREEN);
    match message {
        Ok(request) => Some(axis.handle(&request)),
        Err(error) => commands::reject(error),
    }
}

/// Brings up the Ethernet HAT, if one is fitted, and shows the outcome on
/// the status pixel after the self test report.
fn start_ethernet(hat: board::EthernetHat, axis: &mut Axis) -> Option<Ethernet> {
    match Ethernet::start(hat, axis.mac_address, axis.settings.settings()) {
        Ok(ethernet) => {
            axis.pixel.flash(Rgb::PURPLE, ETHERNET_FLASH_MS);
            Some(ethernet)
        }
        Err(ethernet::Fault::NotFitted) => {
            info!("Ethernet: no HAT fitted");
            None
        }
        Err(fault) => {
            error!("Ethernet: HAT not started: {:?}", fault);
            axis.pixel.flash(Rgb::RED, ETHERNET_FLASH_MS);
            None
        }
    }
}

/// Watches the user button for the first moments after power-up.
fn button_pressed_at_boot(button: &mut UserButton, usb: &mut UsbSerial) -> bool {
    while clock::millis() < BOOT_BUTTON_WINDOW_MS {
        usb.poll();
        if button.is_low().unwrap() {
            return true;
        }
    }
    false
}

/// Blinks until a host opens the serial port, so that it does not miss the
/// start-up.
fn wait_for_host(usb: &mut UsbSerial, pixel: &mut StatusPixel, status_led: &mut StatusLed) {
    let mut last_blink_ms = 0;
    let mut lit = false;
    while !usb.is_connected() {
        usb.poll();
        if clock::every(WAITING_FOR_HOST_BLINK_MS, &mut last_blink_ms) {
            lit = !lit;
            pixel.set_color(if lit {
                WAITING_FOR_HOST_COLOR
            } else {
                Rgb::BLACK
            });
            let _ = status_led.set_state(lit.into());
        }
    }
}
