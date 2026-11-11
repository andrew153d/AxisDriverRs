# Axis Driver firmware

Rust firmware for the Axis Driver, a stepper driver board built around an
ATSAMD51J19A, a TMC2209 and a TLV493D magnetic shaft encoder. It is a port of
the original Arduino/PlatformIO firmware (the `AxisDriver` repository) and
speaks the same host protocol, documented in that repository's
`AxisProtocol.md`, so the existing Python library and web configurator keep
working.

## Layout

| Crate | Contents |
| ----- | -------- |
| [`core`](core) (`axis-core`) | Everything that does not touch the microcontroller: the wire protocol, the speed profile, the motor state machine, encoder filtering, settings, and drivers for the I2C devices. Unit-tested on the host. |
| [`fw`](fw) | The firmware image: board setup, the step timer interrupt, the TMC2209 and WS2812 drivers, the USB serial port and command execution. |

`fw/src/main.rs` lists the firmware modules; `core/src/protocol.rs` is the
place to start for the message format.

## Building and testing

```sh
cargo build --release -p fw   # firmware ELF for thumbv7em-none-eabihf
just test                     # unit tests of axis-core, on the host
```

## Flashing

The board carries a UF2 bootloader in the first 16 KiB of flash, which starts
the firmware at `0x4000` (see `fw/memory.x`). Its image is kept in
[`bootloader/`](bootloader).

### Over USB

```sh
just flash-usb            # or: just flash-usb /dev/ttyACM1
```

This builds the firmware, asks the running firmware to reboot into the
bootloader by opening its serial port at 1200 baud, and uploads with
`bossac`. No button press or debugger is needed. `bossac` is taken from the
PlatformIO package directory (`~/.platformio/packages/tool-bossac`).

In bootloader mode the board also appears as a USB drive named `AXISBASIC`.
If the firmware is too broken to answer on USB, the bootloader can only be
reached by double-tapping reset, or the board recovered over SWD.

### Over SWD

With an Atmel-ICE and [probe-rs](https://probe.rs):

```sh
just flash
```

`cargo run --release -p fw` flashes the same way and then shows the firmware's
log output live over RTT.

A new board, or one whose bootloader has been overwritten, needs the
bootloader flashed once over SWD:

```sh
just flash-bootloader
```

`fw/memory.x` is sized for a SAMD51J19A (512 KiB flash, 192 KiB RAM).

## Host tools

[`tools/`](tools) holds small Python scripts (they need `pyserial`) built on
`tools/axis.py`, a minimal client for the protocol:

```sh
just move to 90          # go to 90 degrees
just move by -45         # move relative to the last target
just move spin 400 -t 3  # run at 400 steps/s for 3 seconds
just move demo           # a short back-and-forth sequence
just logs                # print the firmware's log
```

`move` switches the motor outputs off again when it is done, unless given
`--hold`, and prints anything the firmware logged meanwhile.

## Logs

The firmware logs through the `log` crate. Every message goes out over RTT,
for an attached debugger, and into an 8 KiB buffer that the host can read
back over USB:

```sh
just logs            # print what has been logged since the last read
just logs --follow   # keep watching
```

Reading empties the buffer. If it fills up before being read, the oldest
lines are dropped and the gap is marked in the output. Each line starts with
the time since boot in seconds.

This uses a message that the original protocol does not have:

| Type | Name | Request body | Reply |
| ---- | ---- | ------------ | ----- |
| `0x010A` | Get Log | empty | same type; body is up to 240 bytes of UTF-8 log text, empty when there is nothing left |

## Motion control

Steps are scheduled against a free-running 32-bit hardware timer (TC2 and
TC3 chained, 8.3 ns resolution) on an exact timeline: each step is due a
computed interval after the previous one was *due*, so interrupt latency
shifts individual pulses by microseconds but never accumulates into a speed
error. The speed profile (`core/src/profile.rs`) uses constant-acceleration
kinematics per step, so ramps are exact trapezoids and a position move ends
on its target without creeping.

Limitations: the step pulse is produced by the interrupt handler rather than
by a timer output, so pulses jitter by the interrupt latency (worst when the
status pixel is being written, about 40 us); the profile is trapezoidal, with
no jerk limiting; and control is open loop, with the encoder used only to
detect the stall during homing.

## Power-on self test

At start-up the firmware checks every peripheral it can verify on its own,
logs the outcome (see [Logs](#logs)) and shows it on the status pixel as one flash per
check, green for pass and red for fail, in this order:

1. **Step timer**: the timer counts at its nominal rate and its interrupt
   fires at the idle rate.
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

Ported: the complete serial protocol over USB (plus the Get Log message
described above), position / velocity / path /
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
- All motion is limited to the configured acceleration and maximum speed,
  not just position moves. A velocity command ramps to its speed, and a path
  ramps between segment speeds, carries speed across segments that continue
  in the same direction, and brakes to a stop at its end. A path whose
  speed changes are sharper than the acceleration setting allows will
  therefore lag its nominal timing; raise the acceleration to tighten it.
- `IDLE_ON` brakes a moving motor to a stop instead of freezing it mid-step;
  `OFF` and "set current position" still stop it dead.
- Path segments with zero speed or zero length are skipped instead of
  stalling the path.
- Homing is refused with an `ERROR` ACK if the encoder failed its self test.
- The queues for LED steps (32) and path segments (1024) are bounded; a
  command that does not fit is answered with an `ERROR` ACK.
