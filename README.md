# Axis Driver firmware

Rust firmware for the Axis Driver, a closed-loop stepper driver built around an
ATSAMD51J19A, a TMC2209 and a TLV493D magnetic shaft encoder. It is a port of
the original Arduino/PlatformIO firmware (the `AxisDriver` repository) and
speaks the same host protocol, documented in that repository's
`AxisProtocol.md`, so the existing Python library and web configurator keep
working.

## Layout

| Crate | Contents |
| ----- | -------- |
| [`core`](core) (`axis-core`) | Everything that does not touch the microcontroller: the wire protocol, step timing, the motor state machine, encoder filtering, settings, and drivers for the I2C devices. Unit-tested on the host. |
| [`fw`](fw) | The firmware image: board setup, the step timer interrupt, the TMC2209 and WS2812 drivers, the USB serial port and command execution. |

`fw/src/main.rs` lists the firmware modules; `core/src/protocol.rs` is the
place to start for the message format.

## Building and testing

```sh
cargo build --release -p fw   # firmware ELF for thumbv7em-none-eabihf
just test                     # unit tests of axis-core, on the host
```

## Flashing

Over SWD with an Atmel-ICE and [probe-rs](https://probe.rs):

```sh
just flash
```

`cargo run --release -p fw` flashes the same way and then shows the firmware's
log output, which is sent over RTT.

Over the SAM-BA serial bootloader:

```sh
PORT=/dev/ttyACM0 SAMBA_DEVICE=atsamd51j19a ./flash.sh
```

`fw/memory.x` places the image at the start of flash and is sized for a
SAMD51J19A (512 KiB flash, 192 KiB RAM). Update it and `SAMBA_DEVICE` for a
different part.

## Power-on self test

At start-up the firmware checks every peripheral it can verify on its own,
logs the outcome over RTT and shows it on the status pixel as one flash per
check, green for pass and red for fail, in this order:

1. **Step timer**: the step interrupt fires at its idle rate.
2. **Motor driver**: the TMC2209 answers on its UART, identifies as a
   TMC2209, accepted its configuration (it counts the writes), and reports no
   undervoltage of the motor supply, overtemperature or short circuit.
3. **Encoder**: the TLV493D answers and sees a usable field from the shaft
   magnet.
4. **EEPROM**: the settings and a valid MAC address can be read.

A failed check does not stop the firmware: it carries on with whatever works
so the board can still be reached and diagnosed. A failed encoder is not used
for homing.

Not covered, because they cannot be verified without a host or an operator:
the USB port, the status LED and pixel themselves, and the user button. The
supply voltage sense inputs are not read by this firmware yet. The motor is
not moved.

## Port status

Ported: the complete serial protocol over USB, position / velocity / path /
homing motion, TMC2209 configuration, the shaft encoder, the status LEDs and
settings storage in the EEPROM.

Not ported yet: the Ethernet HAT (UDP transport) and the Home Assistant MQTT
integration. The Ethernet address and port settings are stored and reported,
but nothing uses them.

Deliberate differences from the original firmware:

- Incoming messages are reassembled from the byte stream and their checksums
  are verified. Unknown or malformed messages are answered with an
  `INVALID_COMMAND` ACK; commands that fail (for example because the TMC2209
  does not respond) with an `ERROR` ACK.
- Positions are reported with full microstep resolution (0.225°) instead of
  being rounded down to whole degrees.
- "Get homed state" reports `1` once a homing sequence has completed.
- "Get home direction" returns the value that was set.
- The motor outputs stay disabled after power-up until the first motion
  command, rather than being enabled while the state reads `OFF`.
- The queues for LED steps (32) and path segments (1024) are bounded; a
  command that does not fit is answered with an `ERROR` ACK.
