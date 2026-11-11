from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    STATUS_OK: _ClassVar[Status]
    STATUS_ERROR: _ClassVar[Status]
    STATUS_INVALID: _ClassVar[Status]

class MotorState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    MOTOR_STATE_OFF: _ClassVar[MotorState]
    MOTOR_STATE_POSITION: _ClassVar[MotorState]
    MOTOR_STATE_VELOCITY: _ClassVar[MotorState]
    MOTOR_STATE_PATH: _ClassVar[MotorState]
    MOTOR_STATE_IDLE: _ClassVar[MotorState]
    MOTOR_STATE_HOMING: _ClassVar[MotorState]

class Brake(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    BRAKE_NORMAL: _ClassVar[Brake]
    BRAKE_FREEWHEEL: _ClassVar[Brake]
    BRAKE_STRONG: _ClassVar[Brake]
    BRAKE_PASSIVE: _ClassVar[Brake]

class Direction(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    DIRECTION_CLOCKWISE: _ClassVar[Direction]
    DIRECTION_COUNTERCLOCKWISE: _ClassVar[Direction]
STATUS_OK: Status
STATUS_ERROR: Status
STATUS_INVALID: Status
MOTOR_STATE_OFF: MotorState
MOTOR_STATE_POSITION: MotorState
MOTOR_STATE_VELOCITY: MotorState
MOTOR_STATE_PATH: MotorState
MOTOR_STATE_IDLE: MotorState
MOTOR_STATE_HOMING: MotorState
BRAKE_NORMAL: Brake
BRAKE_FREEWHEEL: Brake
BRAKE_STRONG: Brake
BRAKE_PASSIVE: Brake
DIRECTION_CLOCKWISE: Direction
DIRECTION_COUNTERCLOCKWISE: Direction

class Request(_message.Message):
    __slots__ = ("id", "get_info", "get_log", "get_config", "set_config", "save_config", "get_led", "set_led", "add_led_step", "get_motor", "set_motor_state", "move_to", "move_by", "set_velocity", "set_position", "set_motion", "set_homing", "home", "add_path", "start_path")
    ID_FIELD_NUMBER: _ClassVar[int]
    GET_INFO_FIELD_NUMBER: _ClassVar[int]
    GET_LOG_FIELD_NUMBER: _ClassVar[int]
    GET_CONFIG_FIELD_NUMBER: _ClassVar[int]
    SET_CONFIG_FIELD_NUMBER: _ClassVar[int]
    SAVE_CONFIG_FIELD_NUMBER: _ClassVar[int]
    GET_LED_FIELD_NUMBER: _ClassVar[int]
    SET_LED_FIELD_NUMBER: _ClassVar[int]
    ADD_LED_STEP_FIELD_NUMBER: _ClassVar[int]
    GET_MOTOR_FIELD_NUMBER: _ClassVar[int]
    SET_MOTOR_STATE_FIELD_NUMBER: _ClassVar[int]
    MOVE_TO_FIELD_NUMBER: _ClassVar[int]
    MOVE_BY_FIELD_NUMBER: _ClassVar[int]
    SET_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    SET_POSITION_FIELD_NUMBER: _ClassVar[int]
    SET_MOTION_FIELD_NUMBER: _ClassVar[int]
    SET_HOMING_FIELD_NUMBER: _ClassVar[int]
    HOME_FIELD_NUMBER: _ClassVar[int]
    ADD_PATH_FIELD_NUMBER: _ClassVar[int]
    START_PATH_FIELD_NUMBER: _ClassVar[int]
    id: int
    get_info: GetInfo
    get_log: GetLog
    get_config: GetConfig
    set_config: Config
    save_config: SaveConfig
    get_led: GetLed
    set_led: Color
    add_led_step: LedStep
    get_motor: GetMotor
    set_motor_state: SetMotorState
    move_to: MoveTo
    move_by: MoveBy
    set_velocity: SetVelocity
    set_position: SetPosition
    set_motion: MotionSettings
    set_homing: HomingSettings
    home: Home
    add_path: AddPath
    start_path: StartPath
    def __init__(self, id: _Optional[int] = ..., get_info: _Optional[_Union[GetInfo, _Mapping]] = ..., get_log: _Optional[_Union[GetLog, _Mapping]] = ..., get_config: _Optional[_Union[GetConfig, _Mapping]] = ..., set_config: _Optional[_Union[Config, _Mapping]] = ..., save_config: _Optional[_Union[SaveConfig, _Mapping]] = ..., get_led: _Optional[_Union[GetLed, _Mapping]] = ..., set_led: _Optional[_Union[Color, _Mapping]] = ..., add_led_step: _Optional[_Union[LedStep, _Mapping]] = ..., get_motor: _Optional[_Union[GetMotor, _Mapping]] = ..., set_motor_state: _Optional[_Union[SetMotorState, _Mapping]] = ..., move_to: _Optional[_Union[MoveTo, _Mapping]] = ..., move_by: _Optional[_Union[MoveBy, _Mapping]] = ..., set_velocity: _Optional[_Union[SetVelocity, _Mapping]] = ..., set_position: _Optional[_Union[SetPosition, _Mapping]] = ..., set_motion: _Optional[_Union[MotionSettings, _Mapping]] = ..., set_homing: _Optional[_Union[HomingSettings, _Mapping]] = ..., home: _Optional[_Union[Home, _Mapping]] = ..., add_path: _Optional[_Union[AddPath, _Mapping]] = ..., start_path: _Optional[_Union[StartPath, _Mapping]] = ...) -> None: ...

class GetInfo(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetLog(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetConfig(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SaveConfig(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetLed(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class GetMotor(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SetMotorState(_message.Message):
    __slots__ = ("state",)
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: MotorState
    def __init__(self, state: _Optional[_Union[MotorState, str]] = ...) -> None: ...

class MoveTo(_message.Message):
    __slots__ = ("degrees",)
    DEGREES_FIELD_NUMBER: _ClassVar[int]
    degrees: float
    def __init__(self, degrees: _Optional[float] = ...) -> None: ...

class MoveBy(_message.Message):
    __slots__ = ("degrees",)
    DEGREES_FIELD_NUMBER: _ClassVar[int]
    degrees: float
    def __init__(self, degrees: _Optional[float] = ...) -> None: ...

class SetVelocity(_message.Message):
    __slots__ = ("steps_per_s",)
    STEPS_PER_S_FIELD_NUMBER: _ClassVar[int]
    steps_per_s: float
    def __init__(self, steps_per_s: _Optional[float] = ...) -> None: ...

class SetPosition(_message.Message):
    __slots__ = ("degrees",)
    DEGREES_FIELD_NUMBER: _ClassVar[int]
    degrees: float
    def __init__(self, degrees: _Optional[float] = ...) -> None: ...

class Home(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class AddPath(_message.Message):
    __slots__ = ("segments",)
    SEGMENTS_FIELD_NUMBER: _ClassVar[int]
    segments: _containers.RepeatedCompositeFieldContainer[PathSegment]
    def __init__(self, segments: _Optional[_Iterable[_Union[PathSegment, _Mapping]]] = ...) -> None: ...

class StartPath(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class Response(_message.Message):
    __slots__ = ("id", "status", "info", "log", "config", "led", "motor")
    ID_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    INFO_FIELD_NUMBER: _ClassVar[int]
    LOG_FIELD_NUMBER: _ClassVar[int]
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    LED_FIELD_NUMBER: _ClassVar[int]
    MOTOR_FIELD_NUMBER: _ClassVar[int]
    id: int
    status: Status
    info: Info
    log: Log
    config: Config
    led: Color
    motor: Motor
    def __init__(self, id: _Optional[int] = ..., status: _Optional[_Union[Status, str]] = ..., info: _Optional[_Union[Info, _Mapping]] = ..., log: _Optional[_Union[Log, _Mapping]] = ..., config: _Optional[_Union[Config, _Mapping]] = ..., led: _Optional[_Union[Color, _Mapping]] = ..., motor: _Optional[_Union[Motor, _Mapping]] = ...) -> None: ...

class Info(_message.Message):
    __slots__ = ("firmware_version", "mac_address", "serial_number", "uptime_ms", "self_test", "ethernet")
    FIRMWARE_VERSION_FIELD_NUMBER: _ClassVar[int]
    MAC_ADDRESS_FIELD_NUMBER: _ClassVar[int]
    SERIAL_NUMBER_FIELD_NUMBER: _ClassVar[int]
    UPTIME_MS_FIELD_NUMBER: _ClassVar[int]
    SELF_TEST_FIELD_NUMBER: _ClassVar[int]
    ETHERNET_FIELD_NUMBER: _ClassVar[int]
    firmware_version: str
    mac_address: bytes
    serial_number: bytes
    uptime_ms: int
    self_test: SelfTest
    ethernet: Ethernet
    def __init__(self, firmware_version: _Optional[str] = ..., mac_address: _Optional[bytes] = ..., serial_number: _Optional[bytes] = ..., uptime_ms: _Optional[int] = ..., self_test: _Optional[_Union[SelfTest, _Mapping]] = ..., ethernet: _Optional[_Union[Ethernet, _Mapping]] = ...) -> None: ...

class Ethernet(_message.Message):
    __slots__ = ("link_up",)
    LINK_UP_FIELD_NUMBER: _ClassVar[int]
    link_up: bool
    def __init__(self, link_up: bool = ...) -> None: ...

class SelfTest(_message.Message):
    __slots__ = ("step_timer", "motor_driver", "encoder", "eeprom")
    STEP_TIMER_FIELD_NUMBER: _ClassVar[int]
    MOTOR_DRIVER_FIELD_NUMBER: _ClassVar[int]
    ENCODER_FIELD_NUMBER: _ClassVar[int]
    EEPROM_FIELD_NUMBER: _ClassVar[int]
    step_timer: bool
    motor_driver: bool
    encoder: bool
    eeprom: bool
    def __init__(self, step_timer: bool = ..., motor_driver: bool = ..., encoder: bool = ..., eeprom: bool = ...) -> None: ...

class Log(_message.Message):
    __slots__ = ("text",)
    TEXT_FIELD_NUMBER: _ClassVar[int]
    text: bytes
    def __init__(self, text: _Optional[bytes] = ...) -> None: ...

class Config(_message.Message):
    __slots__ = ("ethernet_address", "ethernet_port", "i2c_address")
    ETHERNET_ADDRESS_FIELD_NUMBER: _ClassVar[int]
    ETHERNET_PORT_FIELD_NUMBER: _ClassVar[int]
    I2C_ADDRESS_FIELD_NUMBER: _ClassVar[int]
    ethernet_address: int
    ethernet_port: int
    i2c_address: int
    def __init__(self, ethernet_address: _Optional[int] = ..., ethernet_port: _Optional[int] = ..., i2c_address: _Optional[int] = ...) -> None: ...

class Color(_message.Message):
    __slots__ = ("red", "green", "blue")
    RED_FIELD_NUMBER: _ClassVar[int]
    GREEN_FIELD_NUMBER: _ClassVar[int]
    BLUE_FIELD_NUMBER: _ClassVar[int]
    red: int
    green: int
    blue: int
    def __init__(self, red: _Optional[int] = ..., green: _Optional[int] = ..., blue: _Optional[int] = ...) -> None: ...

class LedStep(_message.Message):
    __slots__ = ("color", "duration_ms")
    COLOR_FIELD_NUMBER: _ClassVar[int]
    DURATION_MS_FIELD_NUMBER: _ClassVar[int]
    color: Color
    duration_ms: int
    def __init__(self, color: _Optional[_Union[Color, _Mapping]] = ..., duration_ms: _Optional[int] = ...) -> None: ...

class Motor(_message.Message):
    __slots__ = ("state", "position", "target", "velocity", "target_velocity", "motion", "homing", "homed", "path_queued", "path_free", "encoder")
    STATE_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    TARGET_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    TARGET_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    MOTION_FIELD_NUMBER: _ClassVar[int]
    HOMING_FIELD_NUMBER: _ClassVar[int]
    HOMED_FIELD_NUMBER: _ClassVar[int]
    PATH_QUEUED_FIELD_NUMBER: _ClassVar[int]
    PATH_FREE_FIELD_NUMBER: _ClassVar[int]
    ENCODER_FIELD_NUMBER: _ClassVar[int]
    state: MotorState
    position: float
    target: float
    velocity: float
    target_velocity: float
    motion: MotionSettings
    homing: HomingSettings
    homed: bool
    path_queued: int
    path_free: int
    encoder: Encoder
    def __init__(self, state: _Optional[_Union[MotorState, str]] = ..., position: _Optional[float] = ..., target: _Optional[float] = ..., velocity: _Optional[float] = ..., target_velocity: _Optional[float] = ..., motion: _Optional[_Union[MotionSettings, _Mapping]] = ..., homing: _Optional[_Union[HomingSettings, _Mapping]] = ..., homed: bool = ..., path_queued: _Optional[int] = ..., path_free: _Optional[int] = ..., encoder: _Optional[_Union[Encoder, _Mapping]] = ...) -> None: ...

class Encoder(_message.Message):
    __slots__ = ("position", "velocity")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    position: float
    velocity: float
    def __init__(self, position: _Optional[float] = ..., velocity: _Optional[float] = ...) -> None: ...

class MotionSettings(_message.Message):
    __slots__ = ("max_speed", "acceleration", "brake")
    MAX_SPEED_FIELD_NUMBER: _ClassVar[int]
    ACCELERATION_FIELD_NUMBER: _ClassVar[int]
    BRAKE_FIELD_NUMBER: _ClassVar[int]
    max_speed: int
    acceleration: int
    brake: Brake
    def __init__(self, max_speed: _Optional[int] = ..., acceleration: _Optional[int] = ..., brake: _Optional[_Union[Brake, str]] = ...) -> None: ...

class HomingSettings(_message.Message):
    __slots__ = ("direction", "speed", "threshold")
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    SPEED_FIELD_NUMBER: _ClassVar[int]
    THRESHOLD_FIELD_NUMBER: _ClassVar[int]
    direction: Direction
    speed: int
    threshold: int
    def __init__(self, direction: _Optional[_Union[Direction, str]] = ..., speed: _Optional[int] = ..., threshold: _Optional[int] = ...) -> None: ...

class PathSegment(_message.Message):
    __slots__ = ("speed", "steps", "relative")
    SPEED_FIELD_NUMBER: _ClassVar[int]
    STEPS_FIELD_NUMBER: _ClassVar[int]
    RELATIVE_FIELD_NUMBER: _ClassVar[int]
    speed: int
    steps: int
    relative: bool
    def __init__(self, speed: _Optional[int] = ..., steps: _Optional[int] = ..., relative: bool = ...) -> None: ...
