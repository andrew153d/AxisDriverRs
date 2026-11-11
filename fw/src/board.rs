//! Pin assignments and peripheral setup of the Axis Driver board.

use atsamd_hal::{
    clock::{ClockGenId, ClockSource, GenericClockController},
    ehal::digital::OutputPin,
    fugit::RateExtU32,
    gpio::{
        FloatingInput, Pin, Pins, PullUpInput, PushPullOutput, PA03, PA04, PA07, PA09, PA10, PA11,
        PA15, PA22, PA23, PB00, PB01, PB02, PB03, PB04, PB05, PB07, PB08, PB09, PB11, PB12, PB13,
        PB30, PB31,
    },
    pac,
    sercom::{
        i2c, spi,
        uart::{self, BaudMode, Oversampling},
        Sercom0, Sercom3, Sercom5,
    },
    usb::{usb_device::bus::UsbBusAllocator, UsbBus},
};

use crate::step_timer::StepTimer;

pub const CPU_HZ: u32 = 120_000_000;

pub type StatusLed = Pin<PB13, PushPullOutput>;
pub type PixelPin = Pin<PB05, PushPullOutput>;
pub type UserButton = Pin<PA15, PullUpInput>;

/// Single-wire UART to the TMC2209.
pub type TmcUart = uart::Uart<uart::Config<uart::PadsFromIds<Sercom0, PA07, PA04>>, uart::Duplex>;
/// Internal I2C bus with the shaft encoder and the EEPROM.
pub type I2cBus = i2c::I2c<i2c::Config<i2c::PadsFromIds<Sercom3, PA22, PA23>>>;

/// SPI bus on the auxiliary port.
pub type AuxSpi = spi::Spi<spi::Config<spi::PadsFromIds<Sercom5, PB00, PB02, PB03>>, spi::Duplex>;

/// The W5500 runs up to 80 MHz; the original firmware used 14 MHz.
const ETHERNET_SPI_HZ: u32 = 12_000_000;

/// Connections to the Ethernet HAT, which plugs into the auxiliary port.
pub struct EthernetHat {
    pub spi: AuxSpi,
    /// Active low.
    pub chip_select: Pin<PB01, PushPullOutput>,
    /// Active low. Held from start-up until the HAT is brought up.
    pub reset: Pin<PB07, PushPullOutput>,
    /// Low while the W5500 has something to report; high without a HAT.
    pub interrupt: Pin<PA03, PullUpInput>,
    /// The W5500 reads its PHY mode from these as it leaves reset. All high
    /// selects auto-negotiation.
    #[allow(dead_code)]
    pub phy_mode: (
        Pin<PB31, PushPullOutput>,
        Pin<PB30, PushPullOutput>,
        Pin<PB04, PushPullOutput>,
    ),
}

/// Connections to the TMC2209 stepper driver, besides its UART.
pub struct MotorPins {
    pub step: Pin<PB09, PushPullOutput>,
    pub direction: Pin<PB08, PushPullOutput>,
    /// Active low.
    pub enable: Pin<PB12, PushPullOutput>,
    /// MS1 and MS2 select the driver's UART address.
    pub ms1: Pin<PA11, PushPullOutput>,
    pub ms2: Pin<PA10, PushPullOutput>,
    pub spread: Pin<PB11, PushPullOutput>,
    /// StallGuard output; not used yet.
    #[allow(dead_code)]
    pub diag: Pin<PA09, FloatingInput>,
}

pub struct Board {
    pub status_led: StatusLed,
    pub pixel: PixelPin,
    /// Low while the button is pressed.
    pub user_button: UserButton,
    pub motor: MotorPins,
    pub step_timer: StepTimer,
    pub tmc_uart: TmcUart,
    pub i2c: I2cBus,
    pub ethernet: EthernetHat,
    pub usb: UsbBusAllocator<UsbBus>,
}

impl Board {
    /// Sets up the clocks and peripherals. Can only be called once.
    pub fn take() -> Self {
        let mut peripherals = pac::Peripherals::take().unwrap();
        let mut core = cortex_m::Peripherals::take().unwrap();

        let mut clocks = GenericClockController::with_internal_32kosc(
            peripherals.gclk,
            &mut peripherals.mclk,
            &mut peripherals.osc32kctrl,
            &mut peripherals.oscctrl,
            &mut peripherals.nvmctrl,
        );
        let gclk0 = clocks.gclk0();
        crate::clock::start(core.SYST);

        // The WS2812 driver times its bits with the cycle counter.
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let pins = Pins::new(peripherals.port);

        let mut motor = MotorPins {
            step: pins.pb09.into_push_pull_output(),
            direction: pins.pb08.into_push_pull_output(),
            enable: pins.pb12.into_push_pull_output(),
            ms1: pins.pa11.into_push_pull_output(),
            ms2: pins.pa10.into_push_pull_output(),
            spread: pins.pb11.into_push_pull_output(),
            diag: pins.pa09.into_floating_input(),
        };
        // Outputs off, UART address 0, StealthChop.
        motor.enable.set_high().unwrap();
        motor.ms1.set_low().unwrap();
        motor.ms2.set_low().unwrap();
        motor.spread.set_low().unwrap();

        let step_timer = StepTimer::new(
            peripherals.tc2,
            peripherals.tc3,
            &mut peripherals.mclk,
            &clocks.tc2_tc3(&gclk0).unwrap(),
        );

        let tmc_uart = uart::Config::new(
            &peripherals.mclk,
            peripherals.sercom0,
            uart::Pads::default().rx(pins.pa07).tx(pins.pa04),
            clocks.sercom0_core(&gclk0).unwrap().freq(),
        )
        .baud(115_200.Hz(), BaudMode::Arithmetic(Oversampling::Bits16))
        .enable();

        let i2c = i2c::Config::new(
            &peripherals.mclk,
            peripherals.sercom3,
            i2c::Pads::new(pins.pa22, pins.pa23),
            clocks.sercom3_core(&gclk0).unwrap().freq(),
        )
        .baud(400.kHz())
        .enable();

        let mut ethernet = EthernetHat {
            spi: spi::Config::new(
                &peripherals.mclk,
                peripherals.sercom5,
                spi::Pads::default()
                    .data_in(pins.pb00)
                    .data_out(pins.pb02)
                    .sclk(pins.pb03),
                clocks.sercom5_core(&gclk0).unwrap().freq(),
            )
            .baud(ETHERNET_SPI_HZ.Hz())
            .spi_mode(spi::MODE_0)
            .enable(),
            chip_select: pins.pb01.into_push_pull_output(),
            reset: pins.pb07.into_push_pull_output(),
            interrupt: pins.pa03.into_pull_up_input(),
            phy_mode: (
                pins.pb31.into_push_pull_output(),
                pins.pb30.into_push_pull_output(),
                pins.pb04.into_push_pull_output(),
            ),
        };
        ethernet.chip_select.set_high().unwrap();
        ethernet.reset.set_low().unwrap();
        ethernet.phy_mode.0.set_high().unwrap();
        ethernet.phy_mode.1.set_high().unwrap();
        ethernet.phy_mode.2.set_high().unwrap();

        // USB runs from the 48 MHz DFLL.
        clocks.configure_gclk_divider_and_source(ClockGenId::Gclk2, 1, ClockSource::Dfll, false);
        let usb_gclk = clocks.get_gclk(ClockGenId::Gclk2).unwrap();
        let usb = UsbBusAllocator::new(UsbBus::new(
            &clocks.usb(&usb_gclk).unwrap(),
            &mut peripherals.mclk,
            pins.pa24,
            pins.pa25,
            peripherals.usb,
        ));

        Self {
            status_led: pins.pb13.into_push_pull_output(),
            pixel: pins.pb05.into_push_pull_output(),
            user_button: pins.pa15.into_pull_up_input(),
            motor,
            step_timer,
            tmc_uart,
            i2c,
            ethernet,
            usb,
        }
    }
}
