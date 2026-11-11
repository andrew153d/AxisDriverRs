# Hardware

KiCad 8 design files of the two boards.

| Path | Contents |
| ---- | -------- |
| [`main-board`](main-board) | The Axis Driver itself (`Axis_Basic`): microcontroller, stepper driver, shaft encoder, EEPROM, USB. |
| [`ethernet-hat`](ethernet-hat) | The Ethernet HAT: a W5500 that plugs into the main board's auxiliary port. |

Each holds the KiCad project (schematic and board), an interactive bill of
materials in `bom/ibom.html`, and the Gerber and drill files as last sent for
manufacture in `manufacturing.zip`.

## Main board

| Part | Role |
| ---- | ---- |
| ATSAMD51J19A | Microcontroller: 120 MHz Cortex-M4F, 512 KiB flash, 192 KiB RAM. Runs without a crystal. |
| TMC2209 | Stepper driver, configured over a single-wire UART. |
| TLV493D-A1B6 | Magnetic sensor facing a magnet on the motor shaft, used as the shaft encoder. I2C address `0x5E`. |
| AT24MAC402 | EEPROM: 256 bytes for the settings (I2C address `0x50`), and a factory programmed MAC address and serial number (`0x58`). |
| WS2812 | Status pixel. |

### Pin assignments

These are what the firmware uses; [`fw/src/board.rs`](../fw/src/board.rs) is
where they are set up.

| Pin | Function |
| --- | -------- |
| PB09 | TMC2209 step |
| PB08 | TMC2209 direction; low is clockwise |
| PB12 | TMC2209 enable, active low |
| PA11, PA10 | TMC2209 MS1 and MS2, which select its UART address; both low |
| PB11 | TMC2209 spread; low selects StealthChop |
| PA09 | TMC2209 diag (StallGuard output); not used by the firmware yet |
| PA04, PA07 | UART to the TMC2209, transmit and receive (SERCOM0), 115200 baud |
| PA22, PA23 | I2C bus with the encoder and the EEPROM, SDA and SCL (SERCOM3), 400 kHz |
| PB13 | Status LED |
| PB05 | Status pixel data |
| PA15 | User button; low while pressed |
| PA24, PA25 | USB D- and D+ |

Auxiliary port, as used by the Ethernet HAT:

| Pin | Function |
| --- | -------- |
| PB03 | SPI clock (SERCOM5) |
| PB02 | SPI data out, to the HAT |
| PB00 | SPI data in, from the HAT |
| PB01 | W5500 chip select, active low |
| PB07 | W5500 reset, active low |
| PA03 | W5500 interrupt, active low; pulled up, so it reads high without a HAT |
| PB31, PB30, PB04 | W5500 PHY mode, read as it leaves reset; all high selects auto-negotiation |

The step timer is TC2 and TC3 chained into one 32-bit counter.
