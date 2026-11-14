// The Axis Driver host protocol: the messages of proto/axis.proto, their
// Protocol Buffers encoding, and the frames they travel in on the USB serial
// port.
//
// The messages are described by hand in SCHEMA and ENUMS rather than
// generated, so that the site needs no build step. tests/protocol.test.mjs
// checks them against proto/axis.proto.
//
// A message is a plain object keyed by the field names of the .proto file. As
// in proto3, a field that is a message or is declared `optional` is left out
// when it is `undefined`, and decodes to `undefined` when absent; any other
// field decodes to its default (0, false, "", no bytes) when absent.

/** Largest encoded message that fits in a frame. */
export const MAX_PAYLOAD = 508;

/** A message or frame that cannot be decoded. */
export class ProtocolError extends Error {}

export const ENUMS = {
  Status: { STATUS_OK: 0, STATUS_ERROR: 1, STATUS_INVALID: 2 },
  MotorState: {
    MOTOR_STATE_OFF: 0,
    MOTOR_STATE_POSITION: 1,
    MOTOR_STATE_VELOCITY: 2,
    MOTOR_STATE_PATH: 3,
    MOTOR_STATE_IDLE: 4,
    MOTOR_STATE_HOMING: 5,
  },
  Brake: { BRAKE_NORMAL: 0, BRAKE_FREEWHEEL: 1, BRAKE_STRONG: 2, BRAKE_PASSIVE: 3 },
  Direction: { DIRECTION_CLOCKWISE: 0, DIRECTION_COUNTERCLOCKWISE: 1 },
};

/** An enum with the common prefix of its value names dropped. */
function unprefixed(values, prefix) {
  return Object.freeze(
    Object.fromEntries(Object.entries(values).map(([name, value]) => [name.slice(prefix.length), value])),
  );
}

/** Result of a request: OK, ERROR or INVALID. */
export const Status = unprefixed(ENUMS.Status, "STATUS_");
/** What the motor is doing: OFF, POSITION, VELOCITY, PATH, IDLE or HOMING. */
export const MotorState = unprefixed(ENUMS.MotorState, "MOTOR_STATE_");
export const Brake = unprefixed(ENUMS.Brake, "BRAKE_");
export const Direction = unprefixed(ENUMS.Direction, "DIRECTION_");

const field = (number, type, label) => ({ number, type, label });
const optional = (number, type) => field(number, type, "optional");
const repeated = (number, type) => field(number, type, "repeated");

/** Every message: its fields by name, with their number, type and label. */
export const SCHEMA = {
  Request: {
    id: field(1, "uint32"),
    get_info: field(10, "GetInfo"),
    get_log: field(11, "GetLog"),
    get_config: field(12, "GetConfig"),
    set_config: field(13, "Config"),
    save_config: field(14, "SaveConfig"),
    get_led: field(20, "GetLed"),
    set_led: field(21, "Color"),
    add_led_step: field(22, "LedStep"),
    get_motor: field(30, "GetMotor"),
    set_motor_state: field(31, "SetMotorState"),
    move_to: field(32, "MoveTo"),
    move_by: field(33, "MoveBy"),
    set_velocity: field(34, "SetVelocity"),
    set_position: field(35, "SetPosition"),
    set_motion: field(36, "MotionSettings"),
    set_homing: field(37, "HomingSettings"),
    home: field(38, "Home"),
    add_path: field(39, "AddPath"),
    start_path: field(40, "StartPath"),
  },
  GetInfo: {},
  GetLog: {},
  GetConfig: {},
  SaveConfig: {},
  GetLed: {},
  GetMotor: {},
  SetMotorState: { state: field(1, "MotorState") },
  MoveTo: { degrees: field(1, "double") },
  MoveBy: { degrees: field(1, "double") },
  SetVelocity: { steps_per_s: field(1, "double") },
  SetPosition: { degrees: field(1, "double") },
  Home: {},
  AddPath: { segments: repeated(1, "PathSegment") },
  StartPath: {},
  Response: {
    id: field(1, "uint32"),
    status: field(2, "Status"),
    info: field(10, "Info"),
    log: field(11, "Log"),
    config: field(12, "Config"),
    led: field(20, "Color"),
    motor: field(30, "Motor"),
  },
  Info: {
    firmware_version: field(1, "string"),
    mac_address: field(2, "bytes"),
    serial_number: field(3, "bytes"),
    uptime_ms: field(4, "uint32"),
    self_test: field(5, "SelfTest"),
    ethernet: field(6, "Ethernet"),
  },
  Ethernet: { link_up: field(1, "bool") },
  SelfTest: {
    step_timer: field(1, "bool"),
    motor_driver: field(2, "bool"),
    encoder: field(3, "bool"),
    eeprom: field(4, "bool"),
  },
  Log: { text: field(1, "bytes") },
  Config: {
    ethernet_address: optional(1, "fixed32"),
    ethernet_port: optional(2, "uint32"),
    i2c_address: optional(3, "uint32"),
  },
  Color: { red: field(1, "uint32"), green: field(2, "uint32"), blue: field(3, "uint32") },
  LedStep: { color: field(1, "Color"), duration_ms: field(2, "uint32") },
  Motor: {
    state: field(1, "MotorState"),
    position: field(2, "double"),
    target: field(3, "double"),
    velocity: field(4, "float"),
    target_velocity: field(5, "double"),
    motion: field(6, "MotionSettings"),
    homing: field(7, "HomingSettings"),
    homed: field(8, "bool"),
    path_queued: field(9, "uint32"),
    path_free: field(10, "uint32"),
    encoder: field(11, "Encoder"),
    stalled: field(12, "bool"),
    corrections: field(13, "uint32"),
  },
  Encoder: { position: field(1, "float"), velocity: field(2, "float") },
  MotionSettings: {
    max_speed: optional(1, "uint32"),
    acceleration: optional(2, "uint32"),
    brake: optional(3, "Brake"),
    closed_loop: optional(4, "bool"),
  },
  HomingSettings: {
    direction: optional(1, "Direction"),
    speed: optional(2, "uint32"),
    threshold: optional(3, "uint32"),
  },
  PathSegment: {
    speed: field(1, "uint32"),
    steps: field(2, "sint32"),
    relative: field(3, "bool"),
  },
};

// ---------------------------------------------------------------------------
// Protocol Buffers wire format

const VARINT = 0;
const FIXED64 = 1;
const DELIMITED = 2;
const FIXED32 = 5;
const UINT32_MAX = 0xffff_ffff;

function pushVarint(out, value) {
  while (value > 0x7f) {
    out.push((value % 0x80) | 0x80);
    value = Math.floor(value / 0x80);
  }
  out.push(value);
}

function pushFixed(out, size, set) {
  const view = new DataView(new ArrayBuffer(size));
  set(view);
  out.push(...new Uint8Array(view.buffer));
}

function unsigned(value, name) {
  if (!Number.isInteger(value) || value < 0 || value > UINT32_MAX) {
    throw new RangeError(`${name} must be a whole number from 0 to ${UINT32_MAX}`);
  }
  return value;
}

function number(value, name) {
  if (typeof value !== "number" || Number.isNaN(value)) {
    throw new RangeError(`${name} must be a number`);
  }
  return value;
}

/** Reads values off the front of a buffer. */
class Reader {
  constructor(bytes) {
    this.bytes = bytes;
    this.position = 0;
  }

  get done() {
    return this.position >= this.bytes.length;
  }

  take(size) {
    if (this.position + size > this.bytes.length) {
      throw new ProtocolError("message is cut short");
    }
    this.position += size;
    return this.bytes.subarray(this.position - size, this.position);
  }

  varint() {
    let value = 0;
    for (let scale = 1, count = 0; count < 10; scale *= 0x80, count++) {
      const [byte] = this.take(1);
      value += (byte & 0x7f) * scale;
      if (byte < 0x80) {
        return value;
      }
    }
    throw new ProtocolError("varint is too long");
  }

  /** A varint as the 32 bit value its low bits hold. */
  uint32() {
    return this.varint() % (UINT32_MAX + 1);
  }

  fixed(size, get) {
    const bytes = this.take(size);
    return get(new DataView(bytes.buffer, bytes.byteOffset, size));
  }

  delimited() {
    return this.take(this.varint());
  }

  skip(wireType) {
    if (wireType === VARINT) this.varint();
    else if (wireType === FIXED64) this.take(8);
    else if (wireType === DELIMITED) this.delimited();
    else if (wireType === FIXED32) this.take(4);
    else throw new ProtocolError(`unsupported wire type ${wireType}`);
  }
}

/** How each scalar type is written and read, and what it is when absent. */
const SCALARS = {
  uint32: {
    wireType: VARINT,
    default: () => 0,
    write: (out, value, name) => pushVarint(out, unsigned(value, name)),
    read: (reader) => reader.uint32(),
  },
  sint32: {
    wireType: VARINT,
    default: () => 0,
    write(out, value, name) {
      if (!Number.isInteger(value) || value < -0x8000_0000 || value > 0x7fff_ffff) {
        throw new RangeError(`${name} must be a 32 bit whole number`);
      }
      pushVarint(out, ((value << 1) ^ (value >> 31)) >>> 0);
    },
    read(reader) {
      const zigzag = reader.uint32();
      return (zigzag >>> 1) ^ -(zigzag & 1);
    },
  },
  bool: {
    wireType: VARINT,
    default: () => false,
    write: (out, value) => out.push(value ? 1 : 0),
    read: (reader) => reader.varint() !== 0,
  },
  fixed32: {
    wireType: FIXED32,
    default: () => 0,
    write: (out, value, name) =>
      pushFixed(out, 4, (view) => view.setUint32(0, unsigned(value, name), true)),
    read: (reader) => reader.fixed(4, (view) => view.getUint32(0, true)),
  },
  float: {
    wireType: FIXED32,
    default: () => 0,
    write: (out, value, name) =>
      pushFixed(out, 4, (view) => view.setFloat32(0, number(value, name), true)),
    read: (reader) => reader.fixed(4, (view) => view.getFloat32(0, true)),
  },
  double: {
    wireType: FIXED64,
    default: () => 0,
    write: (out, value, name) =>
      pushFixed(out, 8, (view) => view.setFloat64(0, number(value, name), true)),
    read: (reader) => reader.fixed(8, (view) => view.getFloat64(0, true)),
  },
  string: {
    wireType: DELIMITED,
    default: () => "",
    write(out, value) {
      const bytes = new TextEncoder().encode(value);
      pushVarint(out, bytes.length);
      out.push(...bytes);
    },
    read: (reader) => new TextDecoder().decode(reader.delimited()),
  },
  bytes: {
    wireType: DELIMITED,
    default: () => new Uint8Array(0),
    write(out, value) {
      pushVarint(out, value.length);
      out.push(...value);
    },
    read: (reader) => reader.delimited().slice(),
  },
};
// Enums travel as the number of their value.
const ENUM = SCALARS.uint32;

function scalar(type) {
  return type in ENUMS ? ENUM : SCALARS[type];
}

function isDefault(value) {
  return value === 0 || value === false || value === "" || value?.length === 0;
}

function pushMessage(out, type, message) {
  for (const [name, { number: fieldNumber, type: fieldType, label }] of Object.entries(SCHEMA[type])) {
    const value = message[name];
    if (value === undefined || value === null) {
      continue;
    }
    if (fieldType in SCHEMA) {
      for (const item of label === "repeated" ? value : [value]) {
        const body = [];
        pushMessage(body, fieldType, item);
        pushVarint(out, (fieldNumber << 3) | DELIMITED);
        pushVarint(out, body.length);
        // Not spread into push(): a body may be long.
        for (const byte of body) out.push(byte);
      }
      continue;
    }
    // Without `optional`, a default value is the same as an absent one.
    if (label !== "optional" && isDefault(value)) {
      continue;
    }
    const codec = scalar(fieldType);
    pushVarint(out, (fieldNumber << 3) | codec.wireType);
    codec.write(out, value, `${type}.${name}`);
  }
}

/** Encodes `message`, of the message type named `type`, to bytes. */
export function encode(type, message) {
  const out = [];
  pushMessage(out, type, message);
  return Uint8Array.from(out);
}

/** Decodes bytes holding a message of the type named `type`. */
export function decode(type, bytes) {
  const fields = SCHEMA[type];
  const byNumber = new Map(Object.entries(fields).map(([name, f]) => [f.number, { name, ...f }]));
  const message = {};
  for (const [name, { type: fieldType, label }] of Object.entries(fields)) {
    if (label === "repeated") {
      message[name] = [];
    } else if (label !== "optional" && !(fieldType in SCHEMA)) {
      message[name] = scalar(fieldType).default();
    }
  }

  const reader = new Reader(bytes);
  while (!reader.done) {
    const tag = reader.varint();
    const wireType = tag % 8;
    const known = byNumber.get(Math.floor(tag / 8));
    if (!known) {
      reader.skip(wireType);
      continue;
    }
    const isMessage = known.type in SCHEMA;
    const expected = isMessage ? DELIMITED : scalar(known.type).wireType;
    if (wireType !== expected) {
      throw new ProtocolError(`${type}.${known.name} has the wrong wire type`);
    }
    const value = isMessage
      ? decode(known.type, reader.delimited())
      : scalar(known.type).read(reader);
    if (known.label === "repeated") {
      message[known.name].push(value);
    } else {
      message[known.name] = value;
    }
  }
  return message;
}

// ---------------------------------------------------------------------------
// Frames
//
//   length  u16, little-endian   number of payload bytes
//   payload                      the encoded message
//   crc     u16, little-endian   CRC-16/CCITT-FALSE of length and payload

/** CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF. */
export function crc16(bytes) {
  let crc = 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1) & 0xffff;
    }
  }
  return crc;
}

/** Puts an encoded message in a frame. */
export function frame(payload) {
  if (payload.length > MAX_PAYLOAD) {
    throw new RangeError("the message is too large for a frame");
  }
  const bytes = new Uint8Array(payload.length + 4);
  const view = new DataView(bytes.buffer);
  view.setUint16(0, payload.length, true);
  bytes.set(payload, 2);
  view.setUint16(payload.length + 2, crc16(bytes.subarray(0, payload.length + 2)), true);
  return bytes;
}

/**
 * Looks for a frame at the start of `bytes`. Returns its payload and the
 * frame's size, or `null` if the frame has not arrived completely yet. Throws
 * a ProtocolError if what is there cannot be a frame.
 */
export function unframe(bytes) {
  if (bytes.length < 2) {
    return null;
  }
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.length);
  const length = view.getUint16(0, true);
  if (length > MAX_PAYLOAD) {
    throw new ProtocolError("response has an impossible length");
  }
  const size = length + 4;
  if (bytes.length < size) {
    return null;
  }
  if (view.getUint16(length + 2, true) !== crc16(bytes.subarray(0, length + 2))) {
    throw new ProtocolError("response has a bad CRC");
  }
  return { payload: bytes.slice(2, length + 2), size };
}
