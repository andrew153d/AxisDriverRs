// The control panel: the page's counterpart of python/axis/ui.py.
//
// The Control page shows the motor's state live and lets you enable it, move
// it to a position, jog it, spin it, change its limits and home it. The
// Configuration page shows the board's identity and edits the settings kept
// in its EEPROM. The firmware log is at the bottom of both. Escape stops the
// motor.

import {
  Axis,
  AxisError,
  MotorState,
  STEPS_PER_REVOLUTION,
  USB_FILTERS,
  ipToInt,
  ipToString,
} from "./axis.js";
import { SCHEMA } from "./protocol.js";

/** How often the status is refreshed, and the log every few refreshes. */
const POLL_MS = 100;
const LOG_EVERY_POLLS = 5;
const JOG_STEPS_DEG = [-90, -10, -1, 1, 10, 90];
/** Lines of the log kept on the page. */
const LOG_LINES = 2000;
const STATE_COLORS = {
  [MotorState.OFF]: "--muted",
  [MotorState.IDLE]: "--ok",
  [MotorState.POSITION]: "--accent",
  [MotorState.VELOCITY]: "--accent",
  [MotorState.PATH]: "--accent",
  [MotorState.HOMING]: "--warning",
};
const DIAL_CENTRE = 115;
const DIAL_RADIUS = 101;

/** Something typed into a field makes no sense. Worded for the user. */
class InputError extends Error {}

const $ = (id) => document.getElementById(id);
const configInputs = document.querySelectorAll("[data-config]");
const readouts = byData("readout");
const identity = byData("identity");

/** The connection, while there is one. */
let axis = null;
let connecting = false;
let polls = 0;
/** Whether the device last reported closed loop control as on. */
let closedLoop = false;
/** Whether the device last reported a working encoder. */
let hasEncoder = false;
/** The settings as last read from the device, in the form `enteredConfig`
 * returns; `null` while not connected. */
let deviceConfig = null;
let configNote = "";

function byData(key) {
  const elements = {};
  for (const element of document.querySelectorAll(`[data-${key}]`)) {
    elements[element.dataset[key]] = element;
  }
  return elements;
}

function setMessage(text) {
  $("message").textContent = text;
}

/** A number with its sign, like Python's "+" format. */
function signed(value, digits = 0) {
  // Adding zero turns a rounded "-0" into "0".
  const rounded = Number(value.toFixed(digits)) + 0;
  return (rounded < 0 ? "−" : "+") + Math.abs(rounded).toFixed(digits);
}

/** A number without a sign unless it is negative, and never "-0". */
function plain(value, digits = 0) {
  return (Number(value.toFixed(digits)) + 0).toFixed(digits).replace("-", "−");
}

function hex(bytes, separator = "") {
  return Array.from(bytes, (byte) => byte.toString(16).padStart(2, "0"))
    .join(separator)
    .toUpperCase();
}

/** Point on the dial, with zero degrees at the top and angles clockwise. */
function onDial(radius, degrees) {
  const angle = (degrees * Math.PI) / 180;
  return [DIAL_CENTRE + radius * Math.sin(angle), DIAL_CENTRE - radius * Math.cos(angle)];
}

// -- Pages -----------------------------------------------------------------

function showPage(name) {
  for (const tab of document.querySelectorAll(".tab")) {
    tab.classList.toggle("selected", tab.dataset.page === name);
    $(`page-${tab.dataset.page}`).classList.toggle("shown", tab.dataset.page === name);
  }
}

// -- Connection ------------------------------------------------------------

/**
 * Connects to `port`, or to the one the user picks. `quiet` leaves a failure
 * unmentioned, for an attempt the user did not ask for.
 */
async function connect(port, quiet = false) {
  if (axis || connecting) {
    return;
  }
  setMessage("");
  if (!port) {
    try {
      port = await navigator.serial.requestPort({ filters: USB_FILTERS });
    } catch (error) {
      // NotFoundError: the user closed the chooser without picking a board.
      if (error.name !== "NotFoundError") {
        setMessage(`Could not connect: ${error.message}`);
      }
      return;
    }
  }

  connecting = true;
  let opened = null;
  let info, status, config;
  try {
    opened = await Axis.open(port);
    info = await opened.info();
    status = await opened.status();
    config = await opened.config();
  } catch (error) {
    await opened?.close();
    if (!quiet) {
      setMessage(`Could not connect: ${error.message}`);
    }
    return;
  } finally {
    connecting = false;
  }
  axis = opened;

  const failed = Object.keys(SCHEMA.SelfTest)
    .filter((name) => !info.self_test?.[name])
    .map((name) => name.replaceAll("_", " "));
  if (failed.length) {
    setMessage(`Self test failed: ${failed.join(", ")}`);
  }
  $("max-speed").value = status.motion.max_speed;
  $("acceleration").value = status.motion.acceleration;
  setSpinLimit(status.motion.max_speed);
  setConnected(true, `Connected over USB  ·  firmware ${info.firmware_version}`);
  identity.firmware.textContent = info.firmware_version;
  identity.mac.textContent = hex(info.mac_address, ":");
  identity.serial.textContent = hex(info.serial_number);
  identity.eeprom.textContent = info.self_test?.eeprom ? "passed" : "FAILED";
  showConfig(config, "Read from the device");
  showStatus(status);
  poll();
}

function dropConnection(description) {
  axis?.close();
  axis = null;
  setConnected(false, description);
}

function setConnected(connected, description = "Not connected") {
  $("connection").textContent = description;
  $("connection-dot").classList.toggle("connected", connected);
  $("connect").textContent = connected ? "Disconnect" : "Connect";
  for (const control of document.querySelectorAll("[data-control]")) {
    control.disabled = !connected;
  }
  if (!connected) {
    showStatus(null);
    for (const element of Object.values(identity)) {
      element.textContent = "–";
    }
    deviceConfig = null;
    showConfigState();
  }
}

// -- Commands --------------------------------------------------------------

/** Sends a command, reporting a refusal instead of throwing. */
async function run(action) {
  const current = axis;
  if (!current) {
    return;
  }
  setMessage("");
  try {
    await action(current);
  } catch (error) {
    if (axis !== current) {
      return;
    }
    const refused =
      error instanceof AxisError || error instanceof InputError || error instanceof RangeError;
    if (!refused) {
      dropConnection("Connection lost");
    }
    setMessage(error.message);
  }
}

/** The number typed into `input`; `what` describes it if it is not one. */
function entered(input, what, { whole = false } = {}) {
  const text = input.value.trim();
  const value = Number(text);
  if (!text || !Number.isFinite(value) || (whole && !Number.isInteger(value))) {
    throw new InputError(`${what} must be a ${whole ? "whole " : ""}number`);
  }
  return value;
}

function jog(degrees) {
  // Relative to where the motor is now, which is what a jog button should
  // mean even right after spinning or homing.
  run(async (axis) => axis.moveTo((await axis.status()).position + degrees));
}

function applyLimits() {
  run(async (axis) => {
    const status = await axis.setMotion({
      max_speed: entered($("max-speed"), "The max speed", { whole: true }),
      acceleration: entered($("acceleration"), "The acceleration", { whole: true }),
    });
    setSpinLimit(status.motion.max_speed);
  });
}

// -- Spinning --------------------------------------------------------------

const slider = $("spin-slider");

function setSpinLimit(limit) {
  limit = Math.max(limit, 1);
  const speed = Math.max(-limit, Math.min(limit, slider.valueAsNumber));
  slider.min = -limit;
  slider.max = limit;
  setSpinSpeed(speed);
}

function setSpinSpeed(speed) {
  const limit = Number(slider.max);
  // Snap to zero near the centre, so that "stopped" is easy to hit.
  slider.value = Math.abs(speed) < limit * 0.03 ? 0 : speed;
  speed = slider.valueAsNumber;
  slider.style.setProperty("--fraction", (speed / limit + 1) / 2);
  const rpm = (speed / STEPS_PER_REVOLUTION) * 60;
  $("spin-label").textContent = `${signed(speed)} steps/s   ·   ${signed(rpm, 1)} rpm`;
}

// -- Configuration ---------------------------------------------------------

/** The settings in the fields, as a `Config` message. Throws an InputError
 * if one makes no sense. */
function enteredConfig() {
  const address = ipToInt($("ethernet-address").value.trim());
  if (address === null) {
    throw new InputError("The IP address must be an IPv4 address such as 192.168.0.10");
  }
  const port = $("ethernet-port").value.trim();
  if (!/^\d+$/.test(port) || Number(port) > 0xffff) {
    throw new InputError("The UDP port must be a number from 0 to 65535");
  }
  // Hexadecimal with a 0x prefix as well as decimal.
  const i2c = $("i2c-address").value.trim();
  if (!/^(0x[0-9a-f]+|\d+)$/i.test(i2c) || Number(i2c) > 0x7f) {
    throw new InputError("The I2C address must be from 0x00 to 0x7F");
  }
  return {
    ethernet_address: address,
    ethernet_port: Number(port),
    i2c_address: Number(i2c),
  };
}

/** Fills the fields with the device's settings. `note` says where they
 * stand, and is shown for as long as the fields are left alone. */
function showConfig(config, note) {
  deviceConfig = {
    ethernet_address: config.ethernet_address ?? 0,
    ethernet_port: config.ethernet_port ?? 0,
    i2c_address: config.i2c_address ?? 0,
  };
  configNote = note;
  $("ethernet-address").value = ipToString(deviceConfig.ethernet_address);
  $("ethernet-port").value = deviceConfig.ethernet_port;
  $("i2c-address").value = "0x" + hex([deviceConfig.i2c_address]);
  showConfigState();
}

/** Says whether the fields differ from the device's settings. */
function showConfigState() {
  const state = $("config-state");
  if (!deviceConfig) {
    state.textContent = "–";
    state.style.color = "";
    return;
  }
  let changed = true;
  try {
    const config = enteredConfig();
    changed = Object.keys(deviceConfig).some((name) => config[name] !== deviceConfig[name]);
  } catch (error) {
    if (!(error instanceof InputError)) {
      throw error;
    }
  }
  state.textContent = changed ? "●  Unsaved changes" : `●  ${configNote}`;
  state.style.color = `var(${changed ? "--warning" : "--ok"})`;
}

function saveConfig() {
  run(async (axis) => {
    await axis.configure(enteredConfig());
    showConfig(await axis.saveConfig(), "Saved to EEPROM");
  });
}

// -- Live status -----------------------------------------------------------

async function poll() {
  const current = axis;
  if (!current) {
    return;
  }
  try {
    const status = await current.status();
    polls += 1;
    const log = polls % LOG_EVERY_POLLS === 0 ? await current.readLog() : "";
    if (axis !== current) {
      return;
    }
    showStatus(status);
    if (log) {
      appendLog(log);
    }
  } catch (error) {
    if (axis === current) {
      dropConnection("Connection lost");
      setMessage(error.message);
    }
    return;
  }
  setTimeout(poll, POLL_MS);
}

/** Updates the readouts; `null` blanks them. */
function showStatus(status) {
  showDial(status);
  const state = $("state");
  const card = state.closest(".card");
  if (!status) {
    state.textContent = "–";
    state.style.color = "";
    card.style.removeProperty("--state");
    $("position").textContent = "–";
    for (const element of Object.values(readouts)) {
      element.textContent = "–";
    }
    return;
  }

  const color = `var(${STATE_COLORS[status.state] ?? "--muted"})`;
  const name = Object.keys(MotorState).find((key) => MotorState[key] === status.state);
  state.textContent = `●  ${name ?? status.state}`;
  state.style.color = color;
  card.style.setProperty("--state", color);
  $("position").textContent = `${plain(status.position, 2)}°`;
  readouts.target.textContent = `${plain(status.target, 2)}°`;
  const rpm = (status.velocity / STEPS_PER_REVOLUTION) * 60;
  readouts.velocity.textContent = `${plain(status.velocity)} steps/s   ·   ${plain(rpm, 1)} rpm`;

  const encoder = status.encoder;
  hasEncoder = Boolean(encoder);
  if (encoder) {
    readouts.encoder.textContent =
      `${plain(encoder.position, 1)}°   ·   ${plain(encoder.velocity)}°/s`;
    // How far the shaft is from where it was told to go.
    readouts["following-error"].textContent = `${signed(encoder.position - status.position, 1)}°`;
  } else {
    readouts.encoder.textContent = "not available";
    readouts["following-error"].textContent = "–";
  }
  closedLoop = Boolean(status.motion?.closed_loop);
  if (status.stalled) {
    readouts["closed-loop"].textContent = "STALLED: shaft blocked";
  } else if (closedLoop) {
    readouts["closed-loop"].textContent = `on   ·   ${status.corrections} corrections`;
  } else {
    readouts["closed-loop"].textContent = "off";
  }
  $("closed-loop").textContent = closedLoop ? "Open the loop" : "Close the loop";
  readouts.homed.textContent = status.homed ? "yes" : "no";
  $("home").disabled = !hasEncoder;
}

/** Draws the shaft angle: commanded position, target and encoder reading. */
function showDial(status) {
  $("dial-live").style.display = status ? "" : "none";
  if (!status) {
    return;
  }
  // The part of the turn the shaft has gone through, as an arc.
  const turn = ((status.position % 360) + 360) % 360;
  const [arcX, arcY] = onDial(DIAL_RADIUS, turn);
  $("dial-arc").setAttribute(
    "d",
    `M ${DIAL_CENTRE} ${DIAL_CENTRE - DIAL_RADIUS} ` +
      `A ${DIAL_RADIUS} ${DIAL_RADIUS} 0 ${turn > 180 ? 1 : 0} 1 ${arcX} ${arcY}`,
  );
  $("dial-arc").style.visibility = turn ? "" : "hidden";

  const target = $("dial-target");
  const [targetX, targetY] = onDial(DIAL_RADIUS, status.target);
  target.setAttribute("cx", targetX);
  target.setAttribute("cy", targetY);
  target.style.visibility = status.target !== status.position ? "" : "hidden";

  const encoder = $("dial-encoder");
  const [encoderX, encoderY] = onDial(DIAL_RADIUS - 30, status.encoder?.position ?? 0);
  encoder.setAttribute("cx", encoderX);
  encoder.setAttribute("cy", encoderY);
  encoder.style.visibility = status.encoder ? "" : "hidden";

  const [tipX, tipY] = onDial(DIAL_RADIUS - 26, status.position);
  $("dial-needle").setAttribute("x2", tipX);
  $("dial-needle").setAttribute("y2", tipY);
}

function appendLog(text) {
  const log = $("log");
  const atEnd = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
  for (const line of text.match(/[^\n]*\n|[^\n]+$/g) ?? []) {
    const level = ["WARN", "ERROR"].find((level) => line.includes(` ${level} `));
    const span = document.createElement("span");
    span.textContent = line;
    if (level) {
      span.className = level.toLowerCase();
    }
    log.append(span);
  }
  while (log.childElementCount > LOG_LINES) {
    log.firstElementChild.remove();
  }
  // Follow the log, unless the user has scrolled back to read something.
  if (atEnd) {
    log.scrollTop = log.scrollHeight;
  }
}

// -- Setting up ------------------------------------------------------------

function build() {
  const ticks = $("dial-ticks");
  for (let tick = 0; tick < 360; tick += 30) {
    const major = tick % 90 === 0;
    const [x1, y1] = onDial(DIAL_RADIUS - (major ? 22 : 17), tick);
    const [x2, y2] = onDial(DIAL_RADIUS - 11, tick);
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    Object.entries({ x1, y1, x2, y2 }).forEach(([name, value]) => line.setAttribute(name, value));
    line.classList.toggle("major", major);
    ticks.append(line);
  }

  for (const degrees of JOG_STEPS_DEG) {
    const button = document.createElement("button");
    button.type = "button";
    button.dataset.control = "";
    button.textContent = `${signed(degrees)}°`;
    button.addEventListener("click", () => jog(degrees));
    $("jog").append(button);
  }
}

function listen() {
  for (const tab of document.querySelectorAll(".tab")) {
    tab.addEventListener("click", () => showPage(tab.dataset.page));
  }
  $("connect").addEventListener("click", () =>
    axis ? dropConnection("Not connected") : connect(),
  );

  $("enable").addEventListener("click", () => run((axis) => axis.enable()));
  $("disable").addEventListener("click", () => run((axis) => axis.disable()));
  $("stop").addEventListener("click", () => run((axis) => axis.stop()));
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      run((axis) => axis.stop());
    }
  });

  $("goto-form").addEventListener("submit", (event) => {
    event.preventDefault();
    run((axis) => axis.moveTo(entered($("goto"), "The position")));
  });
  $("set-zero").addEventListener("click", () => run((axis) => axis.setPosition(0)));

  slider.addEventListener("input", () => setSpinSpeed(slider.valueAsNumber));
  $("spin-zero").addEventListener("click", () => setSpinSpeed(0));
  $("spin").addEventListener("click", () => run((axis) => axis.spin(slider.valueAsNumber)));

  $("limits-form").addEventListener("submit", (event) => {
    event.preventDefault();
    applyLimits();
  });
  $("closed-loop").addEventListener("click", () =>
    run((axis) => axis.setMotion({ closed_loop: !closedLoop })),
  );
  $("home").addEventListener("click", () => run((axis) => axis.home()));

  for (const input of configInputs) {
    input.addEventListener("input", showConfigState);
  }
  // Discards what was typed in favour of the device's settings.
  $("reload-config").addEventListener("click", () =>
    run(async (axis) => showConfig(await axis.config(), "Read from the device")),
  );
  $("save-config").addEventListener("click", saveConfig);

  // Leaving the page switches the motor outputs off, as far as the browser
  // lets a page do anything while it goes away.
  window.addEventListener("pagehide", () => axis?.disable().catch(() => {}));
}

async function start() {
  build();
  showPage("control");
  listen();
  setSpinSpeed(0);
  setConnected(false);

  if (!("serial" in navigator)) {
    $("connect").disabled = true;
    setMessage(
      "This browser cannot reach USB serial ports. Use Chrome, Edge or a recent " +
        "Firefox on a computer, and open the page over HTTPS or from localhost.",
    );
    return;
  }
  // A board the user has connected to before needs no asking.
  const ports = await navigator.serial.getPorts();
  if (ports.length === 1) {
    connect(ports[0], true);
  }
}

start();
