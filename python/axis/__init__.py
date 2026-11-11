"""Host library for the Axis Driver stepper motor controller.

    from axis import Axis

    with Axis("/dev/ttyACM0") as axis:
        axis.move_to(90, wait=True)
        print(axis.position)

See :class:`Axis`. The messages exchanged with the device are defined in
``proto/axis.proto``.
"""

from .client import (
    DEFAULT_PORT,
    STEPS_PER_REVOLUTION,
    Axis,
    AxisError,
    Brake,
    CommandError,
    Direction,
    MotorState,
    PathSegment,
    Status,
)

__all__ = [
    "DEFAULT_PORT",
    "STEPS_PER_REVOLUTION",
    "Axis",
    "AxisError",
    "Brake",
    "CommandError",
    "Direction",
    "MotorState",
    "PathSegment",
    "Status",
]
