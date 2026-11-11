//! Firmware for the Axis Driver, a SAMD51 based closed-loop stepper driver.
//!
//! The hardware-independent logic lives in the `axis-core` crate; this crate
//! connects it to the board:
//!
//! - [`board`]: pin assignments, clocks and peripherals
//! - [`motor`]: step generation in a timer interrupt, driver configuration
//! - [`encoder`], [`led`]: the other on-board devices
//! - [`usb_serial`]: the link to the host
//! - [`commands`]: execution of host requests

#![no_std]
#![no_main]

mod axis;
mod board;
mod clock;
mod commands;
mod encoder;
mod led;
mod motor;
mod tmc2209;
mod usb_serial;
mod ws2812;

use atsamd_hal::ehal::digital::{InputPin, OutputPin};
use atsamd_hal::usb::{usb_device::bus::UsbBusAllocator, UsbBus};
use axis_core::protocol::{Rgb, FIRMWARE_VERSION};
use cortex_m_rt::entry;
use log::info;
use panic_halt as _;
use rtt_target::{rtt_init_log, ChannelMode};

use axis::Axis;
use board::{Board, StatusLed, UserButton};
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

#[entry]
fn main() -> ! {
    // Large enough to hold the whole self test report, so that none of it is
    // dropped while the debugger is still catching up.
    rtt_init_log!(log::LevelFilter::Debug, ChannelMode::NoBlockSkip, 8192);
    let mut board = Board::take();
    info!("Axis Driver firmware {:?} starting", FIRMWARE_VERSION);

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
    info!("Axis Driver running");

    loop {
        usb.poll();
        while let Some(message) = usb.next_request() {
            axis.pixel.flash_activity(Rgb::GREEN);
            let reply = match message {
                Ok(request) => Some(axis.handle(request)),
                Err(error) => commands::reject(error),
            };
            if let Some(reply) = reply {
                axis.pixel.flash_activity(Rgb::BLUE);
                usb.send(reply.as_bytes());
            }
        }
        axis.poll();
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
