// The Axis class: a connection to one Axis Driver board over Web Serial.
// The counterpart of python/axis/client.py, as far as the page needs it.

import {
  MAX_PAYLOAD,
  MotorState,
  ProtocolError,
  Status,
  decode,
  encode,
  frame,
  unframe,
} from "./protocol.js";

export { MotorState, Status };

/** Steps per shaft revolution: a 200 step motor driven at 8 microsteps. */
export const STEPS_PER_REVOLUTION = 1600;

/** The USB identity of the firmware's serial port (fw/src/usb_serial.rs). */
export const USB_FILTERS = [{ usbVendorId: 0x239a, usbProductId: 0x8031 }];

/** The device answered, but not with something usable. */
export class AxisError extends Error {}

/**
 * The device refused a request. `status` is `Status.ERROR` if the request
 * could not be carried out, in which case the firmware log says why, or
 * `Status.INVALID` if the device could not make sense of it.
 */
export class CommandError extends AxisError {
  constructor(command, status) {
    const name = Object.keys(Status).find((key) => Status[key] === status) ?? status;
    const hint = status === Status.ERROR ? "; see the firmware log" : "";
    super(`${command} failed with status ${name}${hint}`);
    this.command = command;
    this.status = status;
  }
}

/** The device did not answer in time. */
export class AxisTimeout extends Error {}

/**
 * A connection to an Axis Driver board.
 *
 * Positions are in degrees of shaft rotation. Speeds are in steps per second
 * and accelerations in steps per second squared, where one revolution is
 * STEPS_PER_REVOLUTION steps.
 *
 * Every method sends one request and resolves with the device's answer.
 * Requests are sent one at a time, in the order they were made. One the
 * device refuses rejects with a CommandError, one that goes unanswered with
 * an AxisTimeout, and anything else that is not an AxisError means the port
 * is gone.
 */
export class Axis {
  #port;
  #reader;
  #writer;
  #timeout;
  /** Received bytes that have not been made into a frame yet. */
  #buffer = new Uint8Array(0);
  /** Called when bytes arrive or the port stops delivering them. */
  #wake = null;
  /** Why nothing more will be received, once that is so. */
  #lost = null;
  /** Resolves when the request before the next one has been answered. */
  #queue = Promise.resolve();
  #nextId = 1;
  #pumped;

  /** Opens `port`, a SerialPort the user has granted access to. */
  static async open(port, { timeout = 1000 } = {}) {
    await port.open({ baudRate: 115200 });
    return new Axis(port, timeout);
  }

  constructor(port, timeout) {
    this.#port = port;
    this.#timeout = timeout;
    this.#reader = port.readable.getReader();
    this.#writer = port.writable.getWriter();
    this.#pumped = this.#pump();
  }

  async close() {
    await this.#reader.cancel().catch(() => {});
    await this.#pumped;
    this.#writer.releaseLock();
    await this.#port.close().catch(() => {});
  }

  // -- Transport ------------------------------------------------------------

  /** Collects what the port delivers until it stops. */
  async #pump() {
    try {
      for (;;) {
        const { value, done } = await this.#reader.read();
        if (done) {
          this.#lost = new Error("the connection is closed");
          break;
        }
        const joined = new Uint8Array(this.#buffer.length + value.length);
        joined.set(this.#buffer);
        joined.set(value, this.#buffer.length);
        this.#buffer = joined;
        this.#wake?.();
      }
    } catch (error) {
      this.#lost = error;
    }
    this.#reader.releaseLock();
    this.#wake?.();
  }

  /** Drops whatever has been received, to get back in step. */
  #discard() {
    this.#buffer = new Uint8Array(0);
  }

  /** Waits for a whole frame and returns its payload. */
  async #receive() {
    const deadline = performance.now() + this.#timeout;
    for (;;) {
      let found;
      try {
        found = unframe(this.#buffer);
      } catch (error) {
        this.#discard();
        throw new AxisError(error.message);
      }
      if (found) {
        this.#buffer = this.#buffer.subarray(found.size);
        return found.payload;
      }
      if (this.#lost) {
        throw this.#lost;
      }
      const remaining = deadline - performance.now();
      if (remaining <= 0) {
        throw new AxisTimeout(
          this.#buffer.length ? "response cut short" : "no response from the device",
        );
      }
      await new Promise((resolve) => {
        const timer = setTimeout(resolve, remaining);
        this.#wake = () => {
          clearTimeout(timer);
          resolve();
        };
      });
      this.#wake = null;
    }
  }

  /**
   * Sends one command and resolves with the response to it. `command` names
   * a field of the `Request.command` oneof and `message` is its value, e.g.
   * `request("move_to", { degrees: 90 })`.
   */
  request(command, message = {}) {
    const response = this.#queue.then(() => this.#exchange(command, message));
    this.#queue = response.catch(() => {});
    return response;
  }

  async #exchange(command, message) {
    if (this.#lost) {
      throw this.#lost;
    }
    const id = this.#nextId;
    this.#nextId = (this.#nextId % 0xffff_ffff) + 1;
    const payload = encode("Request", { id, [command]: message });
    if (payload.length > MAX_PAYLOAD) {
      throw new RangeError(`${command} request is too large`);
    }
    // A late response to an earlier request must not answer this one.
    this.#discard();
    await this.#writer.write(frame(payload));

    let response;
    try {
      response = decode("Response", await this.#receive());
    } catch (error) {
      if (!(error instanceof ProtocolError)) {
        throw error;
      }
      this.#discard();
      throw new AxisError(`${command}: response cannot be decoded`);
    }
    if (response.id !== id) {
      // An answer to something else: the link is out of step.
      this.#discard();
      throw new AxisError(`${command}: response does not match the request`);
    }
    if (response.status !== Status.OK) {
      throw new CommandError(command, response.status);
    }
    return response;
  }

  // -- Device ---------------------------------------------------------------

  /** Firmware version, MAC address, serial number and self test outcome. */
  async info() {
    return (await this.request("get_info")).info;
  }

  /** Returns, and removes, everything in the firmware's log buffer. */
  async readLog() {
    const decoder = new TextDecoder();
    let text = "";
    for (;;) {
      const chunk = (await this.request("get_log")).log.text;
      if (!chunk.length) {
        return text + decoder.decode();
      }
      text += decoder.decode(chunk, { stream: true });
    }
  }

  /** The settings kept in non-volatile memory. */
  async config() {
    return (await this.request("get_config")).config;
  }

  /**
   * Changes the settings present in `config` and returns them all. The
   * changes are lost at power-off unless followed by `saveConfig`.
   */
  async configure(config) {
    return (await this.request("set_config", config)).config;
  }

  /** Writes the settings to non-volatile memory. */
  async saveConfig() {
    return (await this.request("save_config")).config;
  }

  // -- Motor ----------------------------------------------------------------

  /** Everything about the motor in one snapshot. */
  async status() {
    return (await this.request("get_motor")).motor;
  }

  /** Switches the motor outputs on, holding position. */
  enable() {
    return this.#setState(MotorState.IDLE);
  }

  /** Switches the motor outputs off. A moving motor coasts. */
  disable() {
    return this.#setState(MotorState.OFF);
  }

  /** Brakes to a stop at the configured acceleration and holds. */
  stop() {
    return this.#setState(MotorState.IDLE);
  }

  /** Starts a move to an absolute position. */
  async moveTo(degrees) {
    return (await this.request("move_to", { degrees })).motor;
  }

  /** Accelerates to a constant velocity; negative is counter-clockwise. */
  async spin(stepsPerSecond) {
    return (await this.request("set_velocity", { steps_per_s: stepsPerSecond })).motor;
  }

  /** Redefines the current position without moving. */
  async setPosition(degrees) {
    return (await this.request("set_position", { degrees })).motor;
  }

  /**
   * Changes the motion settings present in `settings`: `max_speed`,
   * `acceleration`, `brake` and `closed_loop`.
   */
  async setMotion(settings) {
    return (await this.request("set_motion", settings)).motor;
  }

  /** Starts the sensorless homing sequence. */
  async home() {
    return (await this.request("home")).motor;
  }

  async #setState(state) {
    return (await this.request("set_motor_state", { state })).motor;
  }
}

/**
 * Packs a dotted IPv4 address the way the device stores it: first octet in
 * the least significant byte. Returns `null` if it is not an address.
 */
export function ipToInt(address) {
  const octets = /^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/.exec(address)?.slice(1).map(Number);
  if (!octets || octets.some((octet) => octet > 255)) {
    return null;
  }
  return octets.reduce((packed, octet, index) => packed + octet * 2 ** (8 * index), 0);
}

/** The reverse of `ipToInt`. */
export function ipToString(address) {
  return [0, 8, 16, 24].map((shift) => (address >>> shift) & 0xff).join(".");
}
