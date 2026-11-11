#!/usr/bin/env python3
"""Move the motor.

    tools/move.py to 90              # go to 90 degrees
    tools/move.py by -45             # move 45 degrees back from the last target
    tools/move.py spin 400 -t 3      # run at 400 steps/s for 3 seconds
    tools/move.py demo               # a short back-and-forth sequence
    tools/move.py off                # switch the motor outputs off

One revolution is 360 degrees or 1600 steps. The outputs are switched off
again afterwards unless --hold is given. Ctrl-C stops the motor.
"""

import argparse
import sys
import time

from axis import DEFAULT_PORT, STEPS_PER_REVOLUTION, Axis, CommandError, MotorState


def show(state: MotorState, position: float) -> None:
    sys.stdout.write(f"\r  {state.name:<10} {position:10.2f} deg ")
    sys.stdout.flush()


def move_to(axis: Axis, degrees: float) -> None:
    print(f"Moving to {degrees:g} deg")
    axis.move_to(degrees)
    axis.wait_until_stopped(on_poll=show)
    print()


def move_by(axis: Axis, degrees: float) -> None:
    print(f"Moving by {degrees:g} deg")
    axis.move_by(degrees)
    axis.wait_until_stopped(on_poll=show)
    print()


def spin(axis: Axis, steps_per_s: float, seconds: float) -> None:
    rpm = steps_per_s / STEPS_PER_REVOLUTION * 60
    print(f"Spinning at {steps_per_s:g} steps/s ({rpm:.1f} rpm) for {seconds:g} s")
    axis.set_velocity(steps_per_s)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        show(axis.motor_state(), axis.position())
        time.sleep(0.05)
    # Brake to a stop rather than cutting the power at speed.
    axis.set_motor_state(MotorState.IDLE_ON)
    axis.wait_until_stopped(on_poll=show)
    print()


def demo(axis: Axis) -> None:
    axis.set_position(0)
    move_to(axis, 90)
    move_to(axis, -90)
    move_to(axis, 0)
    spin(axis, 400, 2)
    spin(axis, -400, 2)
    move_to(axis, 0)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("-p", "--port", default=DEFAULT_PORT)
    parser.add_argument("--max-speed", type=int, help="steps per second")
    parser.add_argument("--acceleration", type=int, help="steps per second squared")
    parser.add_argument(
        "--hold", action="store_true", help="leave the motor energised afterwards"
    )
    actions = parser.add_subparsers(dest="action", required=True)
    actions.add_parser("to", help="move to an absolute position").add_argument(
        "degrees", type=float
    )
    actions.add_parser("by", help="move relative to the last target").add_argument(
        "degrees", type=float
    )
    spin_parser = actions.add_parser("spin", help="run at a constant velocity")
    spin_parser.add_argument("steps_per_s", type=float)
    spin_parser.add_argument("-t", "--seconds", type=float, default=2.0)
    actions.add_parser("demo", help="a short back-and-forth sequence")
    actions.add_parser("off", help="switch the motor outputs off")
    args = parser.parse_args()

    with Axis(args.port) as axis:
        print(f"Connected to firmware {axis.version()} on {args.port}")
        axis.read_log()  # Only show log lines from this run.
        try:
            if args.max_speed is not None:
                axis.set_max_speed(args.max_speed)
            if args.acceleration is not None:
                axis.set_acceleration(args.acceleration)
            print(
                f"Limits: {axis.max_speed()} steps/s, {axis.acceleration()} steps/s^2; "
                f"position {axis.position():.2f} deg"
            )

            if args.action == "to":
                move_to(axis, args.degrees)
            elif args.action == "by":
                move_by(axis, args.degrees)
            elif args.action == "spin":
                spin(axis, args.steps_per_s, args.seconds)
            elif args.action == "demo":
                demo(axis)
            status = 0
        except KeyboardInterrupt:
            print("\nInterrupted")
            status = 130
        except (CommandError, TimeoutError) as error:
            print(f"\nFailed: {error}")
            status = 1
        finally:
            if not args.hold or args.action == "off" or status != 0:
                axis.set_motor_state(MotorState.OFF)
                print("Motor outputs off")
            log = axis.read_log()
            if log:
                print("Firmware log:")
                print(log, end="")
        return status


if __name__ == "__main__":
    sys.exit(main())
