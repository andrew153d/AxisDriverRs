#![no_std]
#![no_main]

mod pins;
mod ws2812;

use atsamd_hal::{ehal::digital::OutputPin, pac};
use atsamd_hal::{
    fugit::RateExtU32,
    sercom::{
        uart::{self, BaudMode, Oversampling},
        Sercom0,
    },
};
use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use tmc2209::{reg::IFCNT, Reader};

const HALF_PERIOD_CYCLES: u32 = 50_000_000;
const TMC2209_ADDRESS: u8 = 0;
const TMC2209_STARTUP_DELAY_CYCLES: u32 = 2_400_000;
const TMC2209_ECHO_TIMEOUT_CYCLES: u32 = 1_200_000;
const TMC2209_RESPONSE_TIMEOUT_CYCLES: u32 = 12_000_000;

#[entry]
fn main() -> ! {
    let mut peripherals = pac::Peripherals::take().unwrap();
    let mut core_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut _clocks = atsamd_hal::clock::GenericClockController::with_internal_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );
    let gclk0 = _clocks.gclk0();
    let sercom0_clock = _clocks.sercom0_core(&gclk0).unwrap();
    let pins = atsamd_hal::gpio::Pins::new(peripherals.port);
    let uart_pads = uart::Pads::<Sercom0>::default().rx(pins.pa07).tx(pins.pa04);
    let mut tmc_uart = uart::Config::new(
        &peripherals.mclk,
        peripherals.sercom0,
        uart_pads,
        sercom0_clock.freq(),
    )
    .baud(115_200u32.Hz(), BaudMode::Arithmetic(Oversampling::Bits16))
    .enable();
    let mut led = pins.pb13.into_push_pull_output();
    let _neopix = pins.pb05.into_push_pull_output();
    core_peripherals.DCB.enable_trace();
    core_peripherals.DWT.enable_cycle_counter();
    core_peripherals.DWT.set_cycle_count(0);

    rtt_init_print!();
    rprintln!(
        "LED blinker started on P{:?}{}",
        pins::STAT_LED.port,
        pins::STAT_LED.number
    );
    rprintln!("Waiting for TMC2209 power-on");
    cortex_m::asm::delay(TMC2209_STARTUP_DELAY_CYCLES);
    tmc_uart.flush_rx_buffer();
    probe_tmc2209(&mut tmc_uart);

    loop {
        led.set_high().unwrap();
        rprintln!("LED high");
        cortex_m::asm::delay(HALF_PERIOD_CYCLES);
        led.set_low().unwrap();
        rprintln!("LED low");
        cortex_m::asm::delay(HALF_PERIOD_CYCLES);
    }
}

fn probe_tmc2209<U>(uart: &mut U)
where
    U: atsamd_hal::ehal_nb::serial::Read<u8> + atsamd_hal::ehal_nb::serial::Write<u8>,
{
    let request = tmc2209::read_request::<IFCNT>(TMC2209_ADDRESS);
    rprintln!("TMC2209 IFCNT request sent");
    let echo_deadline =
        cortex_m::peripheral::DWT::cycle_count().wrapping_add(TMC2209_ECHO_TIMEOUT_CYCLES);

    let mut sent = 0;
    let mut echoed = 0;
    while echoed < request.bytes().len() {
        if sent < request.bytes().len() {
            match atsamd_hal::ehal_nb::serial::Write::write(uart, request.bytes()[sent]) {
                Ok(()) => sent += 1,
                Err(atsamd_hal::nb::Error::WouldBlock) => {}
                Err(atsamd_hal::nb::Error::Other(_)) => {
                    rprintln!("TMC2209 UART write failed");
                    return;
                }
            }
        }

        if echoed < sent {
            match uart.read() {
                Ok(byte) if byte == request.bytes()[echoed] => echoed += 1,
                Ok(byte) => {
                    rprintln!(
                        "TMC2209 echo mismatch: expected {:02X}, received {:02X}",
                        request.bytes()[echoed],
                        byte
                    );
                    return;
                }
                Err(atsamd_hal::nb::Error::Other(_)) => {
                    rprintln!("TMC2209 UART read failed while awaiting echo");
                    return;
                }
                Err(atsamd_hal::nb::Error::WouldBlock) => {}
            }
        }

        if (cortex_m::peripheral::DWT::cycle_count().wrapping_sub(echo_deadline) as i32) >= 0 {
            rprintln!("TMC2209 request echo timed out");
            return;
        }
    }

    loop {
        match atsamd_hal::ehal_nb::serial::Write::flush(uart) {
            Ok(()) => break,
            Err(atsamd_hal::nb::Error::WouldBlock) => {}
            Err(atsamd_hal::nb::Error::Other(_)) => {
                rprintln!("TMC2209 UART flush failed");
                return;
            }
        }
    }

    let mut reader = Reader::default();
    let deadline =
        cortex_m::peripheral::DWT::cycle_count().wrapping_add(TMC2209_RESPONSE_TIMEOUT_CYCLES);
    while (cortex_m::peripheral::DWT::cycle_count().wrapping_sub(deadline) as i32) < 0 {
        match uart.read() {
            Ok(byte) => {
                if let (_, Some(response)) = reader.read_response(&[byte]) {
                    if !response.crc_is_valid() {
                        rprintln!("TMC2209 response CRC failed");
                        return;
                    }
                    match response.register::<IFCNT>() {
                        Ok(ifcnt) => {
                            rprintln!("TMC2209 connected; IFCNT={}", ifcnt.get());
                            return;
                        }
                        Err(_) => {
                            rprintln!("TMC2209 response register mismatch");
                            return;
                        }
                    }
                }
            }
            Err(atsamd_hal::nb::Error::WouldBlock) => {}
            Err(atsamd_hal::nb::Error::Other(_)) => {
                rprintln!("TMC2209 UART read failed");
                return;
            }
        }
    }

    rprintln!("TMC2209 response timed out");
}
