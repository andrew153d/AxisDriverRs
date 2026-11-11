# Axis Driver firmware

Rust firmware for the Axis Driver, a stepper driver board built around an
ATSAMD51J19A, a TMC2209 and a TLV493D magnetic shaft encoder, together with
the Python library for controlling it. It started as a port of the original
Arduino/PlatformIO firmware (the `AxisDriver` repository) but has its own,
Protocol Buffers based host protocol and is not compatible with that
project's host software.

## Layout

| Path | Contents |
| ---- | -------- |
| [`proto/axis.proto`](proto/axis.proto) | The host protocol: every request and response, with their meaning and units. The Rust and Python message types are both generated from it. |
| [`core`](core) (`axis-core`) | Everything that does not touch the microcontroller: message framing, the speed profile, the motor state machine, encoder filtering, settings, and drivers for the I2C devices and the W5500 Ethernet controller. Unit-tested on the host. |
| [`fw`](fw) | The firmware image: board setup, the step timer interrupt, the TMC2209 and WS2812 drivers, the USB serial port, the Ethernet HAT and command execution. |
| [`python`](python) | The `axis` Python package: the `Axis` class, a small command line tool and an example. |
| [`bootloader`](bootloader) | The board's USB bootloader image. |

`fw/src/main.rs` lists the firmware modules.

Building needs `protoc` on the `PATH`, as the Rust message types are
generated at build time.

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

## Protocol

The host sends `Request` messages and the device answers each with one
`Response` carrying the same `id`, a status (`OK`, `ERROR` or `INVALID`) and
the state the request concerns: for example, every motor command is answered
with the full motor status. [`proto/axis.proto`](proto/axis.proto) is the
reference.

On the USB serial port each message travels in a frame: a little-endian
`u16` payload length, the encoded message, and a little-endian
CRC-16/CCITT-FALSE of the length and the payload.

Over Ethernet each message is one UDP datagram holding just the encoded
message; see [Ethernet](#ethernet).

## Python

`python/` holds the `axis` package. Set it up once with

```sh
just python-setup     # creates .venv and installs the package into it
```

and control a board through the `Axis` class:

```python
from axis import Axis, PathSegment

with Axis("/dev/ttyACM0") as axis:
    print(axis.info().firmware_version, axis.state)

    axis.max_speed = 1600                 # steps per second
    axis.move_to(90, wait=True)           # degrees
    axis.move_by(-45, wait=True)

    axis.spin(400)                        # run at a constant velocity...
    axis.stop()                           # ...and brake to a stop

    axis.add_path([PathSegment(speed=600, steps=400),
                   PathSegment(speed=200, steps=-400)])
    axis.start_path(wait=True)

    print(axis.position, axis.encoder_position)
    axis.disable()                        # outputs off
```

A board with an Ethernet HAT is reached the same way by its address and UDP
port instead of a serial port: `Axis("192.168.1.50:12000")`.

Positions are in degrees; speeds are in steps per second, with 1600 steps to
a revolution. Requests the device refuses raise `axis.CommandError`. The
class also covers the status pixel, the stored configuration, homing and the
firmware log; see its docstrings in
[`python/axis/client.py`](python/axis/client.py).

There is also a control panel, which shows the motor's state live and has
controls for moving, jogging and spinning it, changing its limits, homing
and reading the firmware log:

```sh
just ui
```

The USB and Ethernet buttons next to Connect choose how the board is reached:
over a serial port, or over the network at an address given as `HOST:PORT`.
After connecting over USB to a board with an Ethernet HAT, the address field
is filled in with the one the board listens on. `just ui -p 192.168.1.50:12000`
opens the panel connected over Ethernet.

Its Configuration page shows the board's identity (firmware version, MAC
address, serial number) and edits the settings kept in the EEPROM: the
Ethernet address and port and the I2C address. "Save to EEPROM" sends them to
the board and stores them permanently.

Escape stops the motor, and closing the window switches its outputs off. It
needs Tk, which comes with most Python installations.

For a quick look at a board from the terminal there are

```sh
just info      # identity, self test result, configuration
just status    # state of the motor
just logs      # the firmware's log
just demo      # run the motor through a short sequence of moves
```

These talk to `/dev/ttyACM0` unless told otherwise: `just status -p /dev/ttyACM1`
picks another serial port and `just status -p 192.168.1.50:12000` goes over
Ethernet (`just demo 192.168.1.50:12000` for the demo).

After changing `proto/axis.proto`, run `just proto` to regenerate
`python/axis/axis_pb2.py`. It needs the Python `protobuf` package to be at
least as new as `protoc`, which is why the package lives in its own
environment.

## Ethernet

With the Ethernet HAT (a W5500 on the auxiliary port) fitted, the board takes
the same requests over UDP as over USB, and both can be used at once. It
listens on the address and port kept in the EEPROM; each datagram holds one
`Request` and the `Response` goes back to where it came from. To set a board
up, connect over USB and set the address and port on the control panel's
Configuration page (or with `Axis.configure` and `Axis.save_config`), then
restart the board: the settings are applied at start-up. After that

```sh
just info -p 192.168.1.50:12000
```

should answer, and the board answers pings.

- The network is assumed to be a `255.255.255.0` one with its gateway at host
  1, as in the original firmware. There is no DHCP.
- At start-up the status pixel flashes purple after the self test report when
  the HAT is up, and red when a HAT is fitted but could not be started, which
  the log explains: usually the address or port has not been set. Without a
  HAT nothing is shown.
- `just info` shows whether the link is up. Each time it comes up the board
  broadcasts a line of text such as `Axis Driver 2.0.0 at 192.168.1.50:12000`
  to UDP port 35461, which is a way to find boards on a network.
- Requests are neither authenticated nor encrypted: anyone who can reach the
  port can run the motor. A firmware upload still needs USB.
- UDP does not repeat lost datagrams and neither does the `Axis` class, since
  repeating a relative move would move twice; a lost message raises
  `TimeoutError`.

## Logs

The firmware logs through the `log` crate. Every message goes out over RTT,
for an attached debugger, and into an 8 KiB buffer that the host reads back
with the `GetLog` request:

```sh
just logs            # print what has been logged since the last read
just logs --follow   # keep watching
```

Reading empties the buffer. If it fills up before being read, the oldest
lines are dropped and the gap is marked in the output. Each line starts with
the time since boot in seconds.

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

## Compared to the original firmware

Carried over: position, velocity, path and homing motion, TMC2209
configuration, the shaft encoder, the status LEDs and settings storage in the
EEPROM, in the layout the original firmware used.

Carried over as well, with the new protocol: control over UDP through the
Ethernet HAT.

Not carried over: the Home Assistant MQTT integration, and debug output over
UDP (the log is read with `GetLog` instead, over either link). The I2C
address setting is stored and reported, but nothing uses it.

Different:

- The host protocol is new, as described above. Messages are checked on
  arrival: a request that cannot be decoded or holds an invalid value is
  answered with `INVALID`, and one that fails (for example because the
  TMC2209 does not respond) with `ERROR` and a line in the log.
- Positions are reported with full microstep resolution (0.225°) instead of
  being rounded down to whole degrees.
- The homed flag is set once a homing sequence has completed, and the homing
  direction reads back as it was set.
- The motor outputs stay disabled after power-up until the first motion
  command.
- All motion is limited to the configured acceleration and maximum speed,
  not just position moves. A velocity command ramps to its speed, and a path
  ramps between segment speeds, carries speed across segments that continue
  in the same direction, and brakes to a stop at its end. A path whose
  speed changes are sharper than the acceleration setting allows will
  therefore lag its nominal timing; raise the acceleration to tighten it.
- Setting the `IDLE` state brakes a moving motor to a stop instead of
  freezing it mid-step; `OFF` and redefining the position still stop it dead.
- Path segments with zero speed or zero length are skipped instead of
  stalling the path.
- Homing is refused if the encoder failed its self test.
- The queues for LED steps (32) and path segments (1024) are bounded;
  a request that does not fit fails.
