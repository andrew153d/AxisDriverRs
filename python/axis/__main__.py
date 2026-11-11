"""Command line access to a board.

    python -m axis info
    python -m axis status
    python -m axis logs [--follow]
    python -m axis ui

The board is reached over USB by default; "-p HOST:PORT" reaches it over
Ethernet instead, e.g. "python -m axis -p 192.168.1.50:12000 status".
"""

import argparse
import sys
import time

from . import DEFAULT_PORT, Axis, Brake, Direction, MotorState
from .client import ip_to_str


def show_info(axis: Axis) -> None:
    info = axis.info()
    config = axis.config()
    print(f"Firmware      {info.firmware_version}")
    print(f"MAC address   {info.mac_address.hex(':').upper()}")
    print(f"Serial number {info.serial_number.hex().upper()}")
    print(f"Uptime        {info.uptime_ms / 1000:.1f} s")
    print("Self test")
    for field in info.self_test.DESCRIPTOR.fields:
        passed = getattr(info.self_test, field.name)
        print(f"  {field.name:<12} {'pass' if passed else 'FAIL'}")
    if info.HasField("ethernet"):
        print(f"Ethernet      link {'up' if info.ethernet.link_up else 'down'}")
    else:
        print("Ethernet      not in use")
    print("Configuration")
    print(f"  Ethernet     {ip_to_str(config.ethernet_address)}:{config.ethernet_port}")
    print(f"  I2C address  {config.i2c_address:#04x}")


def show_status(axis: Axis) -> None:
    status = axis.status()
    print(f"State           {MotorState(status.state).name}")
    print(f"Position        {status.position:.3f} deg")
    print(f"Target          {status.target:.3f} deg")
    print(f"Velocity        {status.velocity:.1f} steps/s (target {status.target_velocity:g})")
    print(f"Max speed       {status.motion.max_speed} steps/s")
    print(f"Acceleration    {status.motion.acceleration} steps/s^2")
    print(f"Brake           {Brake(status.motion.brake).name}")
    print(
        f"Homing          {Direction(status.homing.direction).name} at "
        f"{status.homing.speed} steps/s, threshold {status.homing.threshold}, "
        f"{'homed' if status.homed else 'not homed'}"
    )
    print(f"Path            {status.path_queued} queued, room for {status.path_free}")
    if status.HasField("encoder"):
        encoder = status.encoder
        print(f"Encoder         {encoder.position:.1f} deg, {encoder.velocity:.1f} deg/s")
    else:
        print("Encoder         not available")
    print(
        f"Closed loop     {'on' if status.motion.closed_loop else 'off'}, "
        f"{status.corrections} corrections{', STALLED' if status.stalled else ''}"
    )


def show_logs(axis: Axis, follow: bool) -> None:
    while True:
        text = axis.read_log()
        sys.stdout.write(text)
        sys.stdout.flush()
        if not follow:
            return
        time.sleep(0.2)


def main() -> int:
    parser = argparse.ArgumentParser(prog="python -m axis", description=__doc__.splitlines()[0])
    parser.add_argument(
        "-p",
        "--port",
        default=DEFAULT_PORT,
        help="serial port, or HOST:PORT to use Ethernet (default: %(default)s)",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("info", help="identity, self test result and configuration")
    commands.add_parser("status", help="state of the motor")
    logs = commands.add_parser("logs", help="print the firmware's log")
    logs.add_argument("-f", "--follow", action="store_true", help="keep watching")
    commands.add_parser("ui", help="open the control panel")
    args = parser.parse_args()

    if args.command == "ui":
        # Imported here so that the other commands work without Tk installed.
        from . import ui

        ui.main(args.port)
        return 0

    try:
        with Axis(args.port) as axis:
            if args.command == "info":
                show_info(axis)
            elif args.command == "status":
                show_status(axis)
            else:
                show_logs(axis, args.follow)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
