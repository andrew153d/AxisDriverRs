// Tests of the Axis class in axis.js, against a simulated board.

import assert from "node:assert/strict";
import { test } from "node:test";

import { Axis, AxisError, AxisTimeout, CommandError, MotorState, Status } from "../axis.js";
import { encode, frame } from "../protocol.js";
import { fakePort } from "./fake-port.mjs";

const open = (answer) => Axis.open(fakePort(answer), { timeout: 30 });
const response = (fields) => frame(encode("Response", fields));

test("a request is answered with the state it concerns", async () => {
  const seen = [];
  const axis = await open((request) => {
    seen.push(request);
    return { motor: { state: MotorState.POSITION, target: 90 } };
  });
  const motor = await axis.moveTo(90);
  assert.equal(motor.state, MotorState.POSITION);
  assert.equal(motor.target, 90);
  assert.deepEqual(seen, [{ id: 1, move_to: { degrees: 90 } }]);
});

test("requests are sent one at a time, in order", async () => {
  const seen = [];
  const axis = await open((request) => {
    seen.push(request.id);
    return { motor: { position: request.id } };
  });
  const motors = await Promise.all([axis.status(), axis.status(), axis.status()]);
  assert.deepEqual(seen, [1, 2, 3]);
  assert.deepEqual(motors.map((motor) => motor.position), [1, 2, 3]);
});

test("a refused request rejects with the status", async () => {
  const axis = await open(() => ({ status: Status.ERROR, motor: {} }));
  await assert.rejects(axis.home(), (error) => {
    assert.ok(error instanceof CommandError);
    assert.equal(error.status, Status.ERROR);
    assert.equal(error.message, "home failed with status ERROR; see the firmware log");
    return true;
  });
});

test("an unanswered request times out, and the next one still works", async () => {
  let silent = true;
  const axis = await open(() => (silent ? null : { motor: {} }));
  await assert.rejects(axis.status(), AxisTimeout);
  silent = false;
  assert.ok(await axis.status());
});

test("a response to something else is refused, and the next one still works", async () => {
  let stale = true;
  const axis = await open((request) =>
    stale ? response({ id: request.id + 100, motor: {} }) : { motor: {} },
  );
  await assert.rejects(axis.status(), /does not match the request/);
  stale = false;
  assert.ok(await axis.status());
});

test("a damaged response is refused", async () => {
  const axis = await open((request) => {
    const bytes = response({ id: request.id, motor: {} });
    bytes[2] ^= 0xff;
    return bytes;
  });
  await assert.rejects(axis.status(), (error) => {
    assert.ok(error instanceof AxisError);
    assert.match(error.message, /bad CRC/);
    return true;
  });
});

test("a response that arrives in pieces is put together", async () => {
  const axis = await open((request) => {
    const bytes = response({ id: request.id, info: { firmware_version: "2.0.0" } });
    return [bytes.subarray(0, 1), bytes.subarray(1, 5), bytes.subarray(5)];
  });
  assert.equal((await axis.info()).firmware_version, "2.0.0");
});

test("the log is read until it is empty", async () => {
  // A character split between two chunks.
  const text = new TextEncoder().encode("1.5 INFO 20 °C\n");
  const chunks = [text.subarray(0, 12), text.subarray(12), new Uint8Array(0)];
  const axis = await open(() => ({ log: { text: chunks.shift() } }));
  assert.equal(await axis.readLog(), "1.5 INFO 20 °C\n");
  assert.equal(chunks.length, 0);
});

test("a board that is pulled out fails requests, but not as an AxisError", async () => {
  const port = fakePort(() => ({ motor: {} }));
  const axis = await Axis.open(port, { timeout: 30 });
  await axis.status();
  port.unplug();
  await assert.rejects(axis.status(), (error) => !(error instanceof AxisError));
  await axis.close();
});

test("closing releases the port", async () => {
  const port = fakePort(() => ({ motor: {} }));
  const axis = await Axis.open(port, { timeout: 30 });
  await axis.status();
  await axis.close();
  assert.equal(port.isOpen, false);
  assert.equal(port.readable.locked, false);
  assert.equal(port.writable.locked, false);
});
