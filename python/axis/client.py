"""The :class:`Axis` class: a connection to one Axis Driver board."""

from __future__ import annotations

import binascii
import ipaddress
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Iterable, Optional

import serial

from . import axis_pb2 as pb

DEFAULT_PORT = "/dev/ttyACM0"
#: Steps per shaft revolution: a 200 step motor driven at 8 microsteps.
STEPS_PER_REVOLUTION = 1600

_MAX_PAYLOAD = 508
#: Most path segments the device accepts in one request.
_PATH_CHUNK = 16


def _enum(wrapper, prefix: str) -> type[IntEnum]:
    """Builds a Python enum from a protocol enum, dropping its name prefix."""
    members = {name.removeprefix(prefix): value for name, value in wrapper.items()}
    return IntEnum(wrapper.DESCRIPTOR.name, members)


#: Result of a request: ``OK``, ``ERROR`` or ``INVALID``.
Status = _enum(pb.Status, "STATUS_")
#: What the motor is doing: ``OFF``, ``POSITION``, ``VELOCITY``, ``PATH``,
#: ``IDLE`` or ``HOMING``.
MotorState = _enum(pb.MotorState, "MOTOR_STATE_")
#: What the driver does at standstill: ``NORMAL``, ``FREEWHEEL``, ``STRONG``
#: or ``PASSIVE``.
Brake = _enum(pb.Brake, "BRAKE_")
#: ``CLOCKWISE`` or ``COUNTERCLOCKWISE``.
Direction = _enum(pb.Direction, "DIRECTION_")


class AxisError(Exception):
    """Something went wrong talking to the device."""


class CommandError(AxisError):
    """The device refused a request.

    ``status`` is :attr:`Status.ERROR` if the request could not be carried
    out, in which case the firmware log (:meth:`Axis.read_log`) says why, or
    :attr:`Status.INVALID` if the device could not make sense of it.
    """

    def __init__(self, command: str, status: Status):
        hint = "; see the firmware log" if status == Status.ERROR else ""
        super().__init__(f"{command} failed with status {status.name}{hint}")
        self.command = command
        self.status = status


@dataclass(frozen=True)
class PathSegment:
    """One leg of a path: travel at ``speed`` until ``steps`` is reached.

    ``steps`` is relative to the end of the previous segment unless
    ``relative`` is false, in which case it is an absolute position.
    """

    speed: int
    steps: int
    relative: bool = True


class Axis:
    """A connection to an Axis Driver board over its USB serial port.

    Positions are in degrees of shaft rotation. Speeds are in steps per
    second and accelerations in steps per second squared, where one
    revolution is :data:`STEPS_PER_REVOLUTION` steps.

    Methods that start a motion return immediately unless called with
    ``wait=True``; :meth:`wait_until_idle` waits for it to finish. Requests
    the device refuses raise :class:`CommandError`.

    Can be used as a context manager, which closes the port on exit. The
    motor is left in whatever state it is in: call :meth:`disable` to switch
    it off.
    """

    def __init__(self, port: str = DEFAULT_PORT, timeout: float = 1.0):
        self._serial = serial.Serial(port, 115200, timeout=timeout)
        self._serial.reset_input_buffer()
        self._next_id = 1

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> Axis:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- Transport ---------------------------------------------------------

    def _request(self, **command) -> pb.Response:
        """Sends one command and returns the response to it.

        ``command`` names one field of the ``Request.command`` oneof and gives
        its message, e.g. ``_request(move_to=pb.MoveTo(degrees=90))``.
        """
        (name,) = command
        request = pb.Request(id=self._next_id, **command)
        self._next_id = self._next_id % 0xFFFF_FFFF + 1

        payload = request.SerializeToString()
        if len(payload) > _MAX_PAYLOAD:
            raise ValueError(f"{name} request is too large")
        frame = struct.pack("<H", len(payload)) + payload
        self._serial.write(frame + struct.pack("<H", _crc16(frame)))

        response = self._read_response()
        if response.id != request.id:
            # An answer to something else: the stream is out of step.
            self._serial.reset_input_buffer()
            raise AxisError(f"{name}: response does not match the request")
        status = Status(response.status)
        if status != Status.OK:
            raise CommandError(name, status)
        return response

    def _read_response(self) -> pb.Response:
        header = self._serial.read(2)
        if len(header) < 2:
            raise TimeoutError("no response from the device")
        (length,) = struct.unpack("<H", header)
        if length > _MAX_PAYLOAD:
            self._serial.reset_input_buffer()
            raise AxisError("response has an impossible length")
        rest = self._serial.read(length + 2)
        if len(rest) < length + 2:
            raise TimeoutError("response cut short")
        payload = rest[:length]
        (crc,) = struct.unpack("<H", rest[length:])
        if crc != _crc16(header + payload):
            self._serial.reset_input_buffer()
            raise AxisError("response has a bad CRC")
        return pb.Response.FromString(payload)

    # -- Device ------------------------------------------------------------

    def info(self) -> pb.Info:
        """Firmware version, MAC address, serial number, uptime and the
        outcome of the power-on self test."""
        return self._request(get_info=pb.GetInfo()).info

    @property
    def firmware_version(self) -> str:
        return self.info().firmware_version

    def read_log(self) -> str:
        """Returns, and removes, everything in the firmware's log buffer."""
        text = b""
        while chunk := self._request(get_log=pb.GetLog()).log.text:
            text += chunk
        return text.decode("utf-8", errors="replace")

    def config(self) -> pb.Config:
        """The settings kept in non-volatile memory."""
        return self._request(get_config=pb.GetConfig()).config

    def configure(
        self,
        *,
        ethernet_address: Optional[str] = None,
        ethernet_port: Optional[int] = None,
        i2c_address: Optional[int] = None,
    ) -> pb.Config:
        """Changes the given settings and returns them all.

        ``ethernet_address`` is a dotted IPv4 address. The changes are lost
        at power-off unless followed by :meth:`save_config`.
        """
        config = pb.Config()
        if ethernet_address is not None:
            config.ethernet_address = ip_to_int(ethernet_address)
        if ethernet_port is not None:
            config.ethernet_port = ethernet_port
        if i2c_address is not None:
            config.i2c_address = i2c_address
        return self._request(set_config=config).config

    def save_config(self) -> pb.Config:
        """Writes the settings to non-volatile memory."""
        return self._request(save_config=pb.SaveConfig()).config

    # -- Status pixel ------------------------------------------------------

    @property
    def led(self) -> tuple[int, int, int]:
        """Color of the status pixel as ``(red, green, blue)``, 0 to 255."""
        return _rgb(self._request(get_led=pb.GetLed()).led)

    def set_led(self, red: int, green: int, blue: int) -> None:
        """Changes the color of the status pixel immediately."""
        self._request(set_led=pb.Color(red=red, green=green, blue=blue))

    def add_led_step(self, red: int, green: int, blue: int, duration_ms: int) -> None:
        """Queues a color to be shown for a time, after those already queued."""
        color = pb.Color(red=red, green=green, blue=blue)
        self._request(add_led_step=pb.LedStep(color=color, duration_ms=duration_ms))

    # -- Motor: status -----------------------------------------------------

    def status(self) -> pb.Motor:
        """Everything about the motor in one snapshot: state, position,
        targets, limits, homing settings, path queue and encoder."""
        return self._request(get_motor=pb.GetMotor()).motor

    @property
    def state(self) -> MotorState:
        return MotorState(self.status().state)

    @property
    def position(self) -> float:
        """Commanded position in degrees."""
        return self.status().position

    @property
    def target(self) -> float:
        """Target of the last position move, in degrees."""
        return self.status().target

    @property
    def velocity(self) -> float:
        """Speed the motor is being driven at right now, in steps per second;
        negative is counter-clockwise."""
        return self.status().velocity

    @property
    def is_moving(self) -> bool:
        return _is_moving(self.status())

    @property
    def is_homed(self) -> bool:
        """Whether the last homing sequence ran to completion."""
        return self.status().homed

    @property
    def encoder_position(self) -> Optional[float]:
        """Shaft position measured by the encoder, in degrees, counted from the
        same zero as :attr:`position`; ``None`` if the encoder failed the
        self test."""
        status = self.status()
        return status.encoder.position if status.HasField("encoder") else None

    # -- Motor: motion -----------------------------------------------------

    def enable(self) -> pb.Motor:
        """Switches the motor outputs on, holding position. If the motor is
        moving it brakes to a stop; see :meth:`stop`."""
        return self._set_state(MotorState.IDLE)

    def disable(self) -> pb.Motor:
        """Switches the motor outputs off. A moving motor is not braked: it
        coasts, and its position is no longer known."""
        return self._set_state(MotorState.OFF)

    def stop(self, wait: bool = True, timeout: float = 60.0) -> pb.Motor:
        """Brakes to a stop at the configured acceleration and holds."""
        return self._started(self._set_state(MotorState.IDLE), wait, timeout)

    def move_to(self, degrees: float, wait: bool = False, timeout: float = 60.0) -> pb.Motor:
        """Starts a move to an absolute position."""
        motor = self._request(move_to=pb.MoveTo(degrees=degrees)).motor
        return self._started(motor, wait, timeout)

    def move_by(self, degrees: float, wait: bool = False, timeout: float = 60.0) -> pb.Motor:
        """Starts a move relative to the previous target (not to the current
        position), so that repeated relative moves do not accumulate errors."""
        motor = self._request(move_by=pb.MoveBy(degrees=degrees)).motor
        return self._started(motor, wait, timeout)

    def spin(self, steps_per_s: float) -> pb.Motor:
        """Accelerates to a constant velocity and keeps running until told
        otherwise. Negative is counter-clockwise. The speed is limited to
        :attr:`max_speed`."""
        return self._request(set_velocity=pb.SetVelocity(steps_per_s=steps_per_s)).motor

    def set_position(self, degrees: float) -> pb.Motor:
        """Redefines the current position without moving. A moving motor is
        stopped dead."""
        return self._request(set_position=pb.SetPosition(degrees=degrees)).motor

    def wait_until_idle(
        self,
        timeout: float = 60.0,
        poll_interval: float = 0.05,
        on_poll: Optional[Callable[[pb.Motor], None]] = None,
    ) -> pb.Motor:
        """Waits until the motor has finished what it was doing and stopped.

        ``on_poll`` is called with each status snapshot taken while waiting.
        Raises :class:`TimeoutError` if the motor is still busy after
        ``timeout`` seconds; a motor that is spinning never becomes idle.
        """
        deadline = time.monotonic() + timeout
        while True:
            status = self.status()
            if on_poll:
                on_poll(status)
            if not _is_moving(status):
                return status
            if time.monotonic() >= deadline:
                raise TimeoutError("the motor did not become idle in time")
            time.sleep(poll_interval)

    # -- Motor: settings ---------------------------------------------------

    def set_motion(
        self,
        *,
        max_speed: Optional[int] = None,
        acceleration: Optional[int] = None,
        brake: Optional[Brake] = None,
        closed_loop: Optional[bool] = None,
    ) -> pb.Motor:
        """Changes the limits that apply to all motion, the standstill
        behaviour of the driver, and whether closed loop control is on; see
        :attr:`closed_loop`. Only the given settings are changed."""
        settings = pb.MotionSettings()
        if closed_loop is not None:
            settings.closed_loop = closed_loop
        if max_speed is not None:
            settings.max_speed = max_speed
        if acceleration is not None:
            settings.acceleration = acceleration
        if brake is not None:
            settings.brake = int(brake)
        return self._request(set_motion=settings).motor

    @property
    def closed_loop(self) -> bool:
        """Whether the encoder is used to notice lost steps and make up for
        them: the motor stops, the commanded position is corrected to where
        the shaft is, and the motion is taken up again. On at power-up if
        the encoder works; switching it on without one is refused."""
        return self.status().motion.closed_loop

    @closed_loop.setter
    def closed_loop(self, enabled: bool) -> None:
        self.set_motion(closed_loop=enabled)

    @property
    def stalled(self) -> bool:
        """Whether closed loop control gave up on a blocked shaft and stopped
        the motor. Cleared by the next motion command."""
        return self.status().stalled

    @property
    def max_speed(self) -> int:
        """Speed limit in steps per second."""
        return self.status().motion.max_speed

    @max_speed.setter
    def max_speed(self, steps_per_s: int) -> None:
        self.set_motion(max_speed=steps_per_s)

    @property
    def acceleration(self) -> int:
        """Acceleration limit in steps per second squared."""
        return self.status().motion.acceleration

    @acceleration.setter
    def acceleration(self, steps_per_s2: int) -> None:
        self.set_motion(acceleration=steps_per_s2)

    @property
    def brake(self) -> Brake:
        return Brake(self.status().motion.brake)

    @brake.setter
    def brake(self, brake: Brake) -> None:
        self.set_motion(brake=brake)

    # -- Motor: homing -----------------------------------------------------

    def set_homing(
        self,
        *,
        direction: Optional[Direction] = None,
        speed: Optional[int] = None,
        threshold: Optional[int] = None,
    ) -> pb.Motor:
        """Changes how homing is done: the direction in which the end stop
        lies, the approach speed, and the driver's StallGuard threshold."""
        settings = pb.HomingSettings()
        if direction is not None:
            settings.direction = int(direction)
        if speed is not None:
            settings.speed = speed
        if threshold is not None:
            settings.threshold = threshold
        return self._request(set_homing=settings).motor

    def home(self, wait: bool = False, timeout: float = 120.0) -> pb.Motor:
        """Starts the sensorless homing sequence. Refused if the encoder
        failed the self test."""
        return self._started(self._request(home=pb.Home()).motor, wait, timeout)

    # -- Motor: paths ------------------------------------------------------

    def add_path(self, segments: Iterable[PathSegment]) -> pb.Motor:
        """Queues path segments, to be played back by :meth:`start_path` or,
        if a path is already running, as it reaches them.

        Consecutive segments in the same direction are travelled without
        stopping in between.
        """
        segments = list(segments)
        motor = self.status()
        if len(segments) > motor.path_free:
            raise ValueError(
                f"{len(segments)} segments do not fit: room for {motor.path_free}"
            )
        for start in range(0, len(segments), _PATH_CHUNK):
            chunk = [
                pb.PathSegment(speed=s.speed, steps=s.steps, relative=s.relative)
                for s in segments[start : start + _PATH_CHUNK]
            ]
            motor = self._request(add_path=pb.AddPath(segments=chunk)).motor
        return motor

    def start_path(self, wait: bool = False, timeout: float = 60.0) -> pb.Motor:
        """Starts playing back the queued path."""
        motor = self._request(start_path=pb.StartPath()).motor
        return self._started(motor, wait, timeout)

    # -- Helpers -----------------------------------------------------------

    def _set_state(self, state: MotorState) -> pb.Motor:
        request = pb.SetMotorState(state=int(state))
        return self._request(set_motor_state=request).motor

    def _started(self, motor: pb.Motor, wait: bool, timeout: float) -> pb.Motor:
        return self.wait_until_idle(timeout) if wait else motor


def _is_moving(status: pb.Motor) -> bool:
    busy = status.state not in (MotorState.OFF, MotorState.IDLE)
    return busy or status.velocity != 0


def _rgb(color: pb.Color) -> tuple[int, int, int]:
    return color.red, color.green, color.blue


def _crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    return binascii.crc_hqx(data, 0xFFFF)


def ip_to_int(address: str) -> int:
    """Packs a dotted IPv4 address the way the device stores it: first octet
    in the least significant byte."""
    return int.from_bytes(ipaddress.IPv4Address(address).packed, "little")


def ip_to_str(address: int) -> str:
    """The reverse of :func:`ip_to_int`."""
    return str(ipaddress.IPv4Address(address.to_bytes(4, "little")))
