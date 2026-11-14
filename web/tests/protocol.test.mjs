// Tests of protocol.js. Run with `just test-web` (node --test web/tests).

import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";

import {
  ENUMS,
  MAX_PAYLOAD,
  MotorState,
  ProtocolError,
  SCHEMA,
  crc16,
  decode,
  encode,
  frame,
  unframe,
} from "../protocol.js";
import { ipToInt, ipToString } from "../axis.js";

/** The messages and enums of a .proto file, in the form of SCHEMA and ENUMS. */
function parseProto(text) {
  const messages = {};
  const enums = {};
  let fields = null;
  let values = null;
  for (const line of text.replace(/\/\/.*/g, "").split("\n").map((line) => line.trim())) {
    let match;
    if ((match = /^message (\w+) \{(\})?$/.exec(line))) {
      messages[match[1]] = fields = {};
      if (match[2]) fields = null;
    } else if ((match = /^enum (\w+) \{$/.exec(line))) {
      enums[match[1]] = values = {};
    } else if (values && (match = /^(\w+) = (\d+);$/.exec(line))) {
      values[match[1]] = Number(match[2]);
    } else if (fields && (match = /^(?:(optional|repeated) )?(\w+) (\w+) = (\d+);$/.exec(line))) {
      fields[match[3]] = { number: Number(match[4]), type: match[2], label: match[1] };
    } else if (line === "}" && values) {
      values = null;
    } else if (line && !/^(oneof \w+ \{|\}|syntax .*|package .*)$/.test(line)) {
      assert.fail(`cannot parse this line of the .proto file: ${line}`);
    }
  }
  return { messages, enums };
}

test("the schema matches proto/axis.proto", () => {
  const proto = readFileSync(new URL("../../proto/axis.proto", import.meta.url), "utf8");
  const { messages, enums } = parseProto(proto);
  assert.ok(Object.keys(messages).length > 20, "the .proto file was not parsed");
  assert.deepEqual(JSON.parse(JSON.stringify(SCHEMA)), JSON.parse(JSON.stringify(messages)));
  assert.deepEqual(ENUMS, enums);
});

test("the CRC matches the standard check value", () => {
  assert.equal(crc16(new TextEncoder().encode("123456789")), 0x29b1);
  assert.equal(crc16([]), 0xffff);
});

// The same bytes as the tests in core/src/framing.rs expect.
test("a request encodes and is framed like the firmware expects", () => {
  const payload = encode("Request", { id: 7, get_info: {} });
  assert.deepEqual([...payload], [0x08, 0x07, 0x52, 0x00]);
  const framed = frame(payload);
  const crc = crc16([4, 0, ...payload]);
  assert.deepEqual([...framed], [4, 0, ...payload, crc & 0xff, crc >> 8]);
});

test("a response decodes", () => {
  const response = decode("Response", Uint8Array.of(0x08, 0x05, 0x10, 0x01));
  assert.deepEqual(response, { id: 5, status: 1 });
});

test("messages round trip", () => {
  const requests = [
    { id: 42, move_to: { degrees: -90.5 } },
    { id: 0xffff_ffff, set_motion: { max_speed: 0, closed_loop: false } },
    { id: 1, set_config: { ethernet_address: 0xfa01_a8c0, ethernet_port: 12000, i2c_address: 0 } },
    { id: 2, set_motor_state: { state: MotorState.IDLE } },
    {
      id: 3,
      add_path: {
        segments: Array.from({ length: 16 }, () => ({
          speed: 0xffff_ffff,
          steps: -0x8000_0000,
          relative: true,
        })),
      },
    },
  ];
  for (const request of requests) {
    const payload = encode("Request", request);
    assert.ok(payload.length <= MAX_PAYLOAD);
    assert.deepEqual(decode("Request", payload), request);
  }
});

test("absent fields decode to defaults, and to undefined where presence counts", () => {
  const motor = decode("Motor", encode("Motor", { position: 1.5, motion: {} }));
  assert.equal(motor.state, 0);
  assert.equal(motor.position, 1.5);
  assert.equal(motor.homed, false);
  assert.deepEqual(motor.motion, {});
  assert.equal(motor.encoder, undefined);
  // A default value is not sent, unless the field is optional.
  assert.equal(encode("Motor", { position: 0, homed: false }).length, 0);
  assert.deepEqual([...encode("MotionSettings", { max_speed: 0 })], [0x08, 0x00]);
});

test("unknown fields are skipped", () => {
  // Field 99 as a varint and field 98 as bytes, around a known field.
  const bytes = Uint8Array.of(0x98, 0x06, 0x01, 0x08, 0x09, 0x92, 0x06, 0x02, 0xaa, 0xbb);
  assert.equal(decode("Request", bytes).id, 9);
});

test("values that do not fit are refused", () => {
  assert.throws(() => encode("MotionSettings", { max_speed: -1 }), RangeError);
  assert.throws(() => encode("MotionSettings", { max_speed: 1.5 }), RangeError);
  assert.throws(() => encode("MoveTo", { degrees: NaN }), RangeError);
});

test("damaged messages are refused", () => {
  assert.throws(() => decode("Request", Uint8Array.of(0x08)), ProtocolError);
  // `id` as a length-delimited field.
  assert.throws(() => decode("Request", Uint8Array.of(0x0a, 0x00)), ProtocolError);
});

test("frames are found in a stream", () => {
  const first = frame(encode("Request", { id: 1, get_info: {} }));
  const second = frame(encode("Request", { id: 2, move_to: { degrees: 1 } }));
  const stream = Uint8Array.of(...first, ...second);

  assert.equal(unframe(stream.subarray(0, 1)), null);
  assert.equal(unframe(stream.subarray(0, 5)), null);
  const found = unframe(stream);
  assert.equal(found.size, first.length);
  assert.equal(decode("Request", found.payload).id, 1);
  assert.equal(decode("Request", unframe(stream.subarray(found.size)).payload).id, 2);
});

test("damaged frames are refused", () => {
  const damaged = frame(encode("Request", { id: 1, get_info: {} }));
  damaged[3] ^= 0xff;
  assert.throws(() => unframe(damaged), ProtocolError);
  assert.throws(() => unframe(new Uint8Array(16).fill(0xff)), ProtocolError);
});

test("IPv4 addresses are packed first octet lowest", () => {
  assert.equal(ipToInt("192.168.1.250"), 0xfa01_a8c0);
  assert.equal(ipToString(0xfa01_a8c0), "192.168.1.250");
  assert.equal(ipToInt("255.255.255.255"), 0xffff_ffff);
  for (const bad of ["", "1.2.3", "1.2.3.256", "1.2.3.4.5", "a.b.c.d", " 1.2.3.4"]) {
    assert.equal(ipToInt(bad), null);
  }
});
