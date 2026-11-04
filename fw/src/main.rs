#![no_std]
#![no_main]

use atsamd_hal::{ehal::digital::OutputPin, gpio::Pins, pac};
use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

const HALF_PERIOD_CYCLES: u32 = 50_000_000;

#[entry]
fn main() -> ! {
    let peripherals = pac::Peripherals::take().unwrap();
    let pins = Pins::new(peripherals.port);
    let mut led = pins.pb13.into_push_pull_output();

    rtt_init_print!();
    rprintln!("LED blinker started on PB13");

    loop {
        led.set_high().unwrap();
        rprintln!("LED high");
        cortex_m::asm::delay(HALF_PERIOD_CYCLES);
        led.set_low().unwrap();
        rprintln!("LED low");
        cortex_m::asm::delay(HALF_PERIOD_CYCLES);
    }
}
