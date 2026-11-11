#!/usr/bin/env python3
"""Moves the motor through a short sequence using the Axis class.

    python examples/demo.py [PORT]

The motor turns a quarter of a revolution each way, spins briefly in both
directions and plays a small path. The outputs are switched off at the end,
also on Ctrl-C or an error.
"""

import sys
import time

from axis import DEFAULT_PORT, Axis, MotorState, PathSegment


def show(status) -> None:
    state = MotorState(status.state).name
    sys.stdout.write(
        f"\r  {state:<9} {status.position:9.2f} deg {status.velocity:8.1f} steps/s "
    )
    sys.stdout.flush()


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    with Axis(port) as axis:
        print(f"Connected to firmware {axis.firmware_version} on {port}")
        axis.read_log()  # Only show log lines from this run.
        try:
            axis.set_position(0)

            for degrees in (90, -90, 0):
                print(f"Moving to {degrees} deg")
                axis.move_to(degrees)
                axis.wait_until_idle(on_poll=show)
                print()

            for speed in (400, -400):
                print(f"Spinning at {speed} steps/s")
                axis.spin(speed)
                end = time.monotonic() + 2
                while time.monotonic() < end:
                    show(axis.status())
                    time.sleep(0.05)
                axis.stop(wait=False)
                axis.wait_until_idle(on_poll=show)
                print()

            print("Playing a path: out fast, on slowly, and back")
            axis.add_path(
                [
                    PathSegment(speed=600, steps=400),
                    PathSegment(speed=200, steps=200),
                    PathSegment(speed=600, steps=-600),
                ]
            )
            axis.start_path()
            axis.wait_until_idle(on_poll=show)
            print()
        except KeyboardInterrupt:
            print("\nInterrupted")
        finally:
            axis.disable()
            print("Motor outputs off")
            log = axis.read_log()
            if log:
                print("Firmware log:")
                print(log, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
