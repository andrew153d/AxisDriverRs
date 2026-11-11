"""Minimal host-side client for the Axis Driver protocol.

Frames are little-endian: message type (u16), body size (u16), body, and a
checksum (u16) that is the sum of all preceding bytes. "Get" requests have an
empty body and are answered with the value; everything else is answered with
an ACK. Requires pyserial.
"""

import struct
import time
from enum import IntEnum

import serial

DEFAULT_PORT = "/dev/ttyACM0"
STEPS_PER_REVOLUTION = 1600

ACK = 0x0100
GET_VERSION = 0x0101
GET_LOG = 0x010A
SET_MOTOR_STATE = 0x0307
GET_MOTOR_STATE = 0x0308
SET_MAX_SPEED = 0x030B
GET_MAX_SPEED = 0x030C
SET_ACCELERATION = 0x030D
GET_ACCELERATION = 0x030E
SET_CURRENT_POSITION = 0x030F
GET_CURRENT_POSITION = 0x0310
SET_TARGET_POSITION = 0x0311
GET_TARGET_POSITION = 0x0312
SET_VELOCITY = 0x0313
SET_RELATIVE_TARGET_POSITION = 0x0401


class MotorState(IntEnum):
    OFF = 0
    POSITION = 1
    VELOCITY = 2
    VELOCITY_STEP = 3
    IDLE_ON = 4
    HOME = 5


class Status(IntEnum):
    SUCCESS = 0
    ERROR = 1
    INVALID_COMMAND = 2


class CommandError(Exception):
    """The device answered a command with something other than success."""


def frame(message_type: int, body: bytes = b"") -> bytes:
    payload = struct.pack("<HH", message_type, len(body)) + body
    return payload + struct.pack("<H", sum(payload) & 0xFFFF)


class Axis:
    def __init__(self, port: str = DEFAULT_PORT, timeout: float = 1.0):
        self.serial = serial.Serial(port, 115200, timeout=timeout)
        self.serial.reset_input_buffer()

    def close(self) -> None:
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _exchange(self, message_type: int, body: bytes = b"") -> tuple[int, bytes]:
        """Sends a frame and returns the type and body of the reply."""
        self.serial.write(frame(message_type, body))
        header = self.serial.read(4)
        if len(header) < 4:
            raise TimeoutError("no reply from the device")
        reply_type, body_size = struct.unpack("<HH", header)
        rest = self.serial.read(body_size + 2)
        if len(rest) < body_size + 2:
            raise TimeoutError("reply cut short")
        reply_body = rest[:body_size]
        (checksum,) = struct.unpack("<H", rest[body_size:])
        if checksum != sum(header + reply_body) & 0xFFFF:
            raise ValueError("reply has a bad checksum")
        return reply_type, reply_body

    def get(self, message_type: int) -> bytes:
        """Sends a "get" request and returns the body of its reply."""
        reply_type, body = self._exchange(message_type)
        if reply_type != message_type:
            raise CommandError(f"unexpected reply {reply_type:#06x} to {message_type:#06x}")
        return body

    def command(self, message_type: int, body: bytes = b"") -> None:
        """Sends a command and checks that it was acknowledged as a success."""
        reply_type, reply = self._exchange(message_type, body)
        if reply_type != ACK or len(reply) != 3:
            raise CommandError(f"unexpected reply {reply_type:#06x} to {message_type:#06x}")
        acked_type, status = struct.unpack("<HB", reply)
        if acked_type != message_type or status != Status.SUCCESS:
            raise CommandError(
                f"command {message_type:#06x} answered with {Status(status).name}; "
                "the firmware log says why"
            )

    # Convenience wrappers.

    def version(self) -> str:
        return ".".join(str(part) for part in self.get(GET_VERSION))

    def motor_state(self) -> MotorState:
        return MotorState(self.get(GET_MOTOR_STATE)[0])

    def set_motor_state(self, state: MotorState) -> None:
        self.command(SET_MOTOR_STATE, struct.pack("<B", state))

    def position(self) -> float:
        """Current position in degrees."""
        return struct.unpack("<d", self.get(GET_CURRENT_POSITION))[0]

    def set_position(self, degrees: float) -> None:
        """Redefines the current position without moving."""
        self.command(SET_CURRENT_POSITION, struct.pack("<d", degrees))

    def max_speed(self) -> int:
        return struct.unpack("<I", self.get(GET_MAX_SPEED))[0]

    def set_max_speed(self, steps_per_s: int) -> None:
        self.command(SET_MAX_SPEED, struct.pack("<I", steps_per_s))

    def acceleration(self) -> int:
        return struct.unpack("<I", self.get(GET_ACCELERATION))[0]

    def set_acceleration(self, steps_per_s2: int) -> None:
        self.command(SET_ACCELERATION, struct.pack("<I", steps_per_s2))

    def move_to(self, degrees: float) -> None:
        self.command(SET_TARGET_POSITION, struct.pack("<d", degrees))

    def move_by(self, degrees: float) -> None:
        self.command(SET_RELATIVE_TARGET_POSITION, struct.pack("<d", degrees))

    def set_velocity(self, steps_per_s: float) -> None:
        self.command(SET_VELOCITY, struct.pack("<d", steps_per_s))

    def read_log(self) -> str:
        """Returns, and removes, everything in the firmware's log buffer."""
        text = b""
        while chunk := self.get(GET_LOG):
            text += chunk
        return text.decode("utf-8", errors="replace")

    def wait_until_stopped(self, timeout: float = 60.0, on_poll=None) -> None:
        """Waits for a position move, or braking, to finish."""
        deadline = time.monotonic() + timeout
        previous = None
        while time.monotonic() < deadline:
            state, position = self.motor_state(), self.position()
            if on_poll:
                on_poll(state, position)
            if state in (MotorState.IDLE_ON, MotorState.OFF) and position == previous:
                return
            previous = position
            time.sleep(0.05)
        raise TimeoutError("the motor did not stop in time")
