"""A control panel for one Axis Driver board.

    python -m axis ui [--port PORT]

Shows the motor's state live and lets you enable it, move it to a position,
jog it, spin it, change its limits, home it and read the firmware log.
Escape stops the motor. Closing the window switches the motor outputs off.
"""

from __future__ import annotations

import math
import tkinter as tk
import tkinter.font as tkfont
from typing import Callable, Optional

from . import DEFAULT_PORT, STEPS_PER_REVOLUTION, Axis, AxisError, MotorState
from . import axis_pb2 as pb

#: How often the status is refreshed, and the log every few refreshes.
POLL_MS = 100
LOG_EVERY_POLLS = 5
JOG_STEPS_DEG = (-90, -10, -1, 1, 10, 90)


class Theme:
    """Colors and fonts of the panel."""

    WINDOW = "#0e1116"
    CARD = "#171b23"
    CARD_BORDER = "#232936"
    FIELD = "#0e1116"
    FIELD_BORDER = "#2b3342"
    TEXT = "#e7eaf0"
    MUTED = "#8a93a6"
    FAINT = "#3a4354"
    ACCENT = "#4c8dff"
    ACCENT_HOVER = "#6aa1ff"
    BUTTON = "#252b38"
    BUTTON_HOVER = "#303848"
    DANGER = "#e5484d"
    DANGER_HOVER = "#f2686c"
    OK = "#3ecf8e"
    WARNING = "#f5a524"
    DISABLED_TEXT = "#565f72"

    STATE = {
        MotorState.OFF: MUTED,
        MotorState.IDLE: OK,
        MotorState.POSITION: ACCENT,
        MotorState.VELOCITY: ACCENT,
        MotorState.PATH: ACCENT,
        MotorState.HOMING: WARNING,
    }

    def __init__(self, root: tk.Tk):
        available = set(tkfont.families(root))
        sans = _first_available(
            ("Inter", "Ubuntu", "Roboto", "Noto Sans", "DejaVu Sans"), available
        )
        mono = _first_available(
            ("JetBrains Mono", "Ubuntu Mono", "Noto Sans Mono", "DejaVu Sans Mono"), available
        )
        self.body = (sans, 10)
        self.small = (sans, 9)
        self.heading = (sans, 9, "bold")
        self.title = (sans, 14, "bold")
        self.button = (sans, 10, "bold")
        self.readout = (sans, 30, "bold")
        self.mono = (mono, 10)


def _first_available(candidates: tuple[str, ...], available: set[str]) -> str:
    return next((name for name in candidates if name in available), "TkDefaultFont")


def _rounded(
    canvas: tk.Canvas, x0: float, y0: float, x1: float, y1: float, radius: float, **options
) -> int:
    """Draws a rectangle with rounded corners."""
    radius = min(radius, (x1 - x0) / 2, (y1 - y0) / 2)
    points = [
        x0 + radius, y0, x1 - radius, y0, x1, y0, x1, y0 + radius,
        x1, y1 - radius, x1, y1, x1 - radius, y1, x0 + radius, y1,
        x0, y1, x0, y1 - radius, x0, y0 + radius, x0, y0,
    ]  # fmt: skip
    return canvas.create_polygon(points, smooth=True, **options)


class Card(tk.Frame):
    """A rounded panel with a small heading. Put content in ``self.body``."""

    PADDING = 14

    def __init__(self, parent: tk.Widget, theme: Theme, title: str):
        super().__init__(parent, bg=theme.WINDOW)
        self._theme = theme
        self._background = tk.Canvas(self, bg=theme.WINDOW, highlightthickness=0)
        self._background.place(x=0, y=0, relwidth=1, relheight=1)
        self.bind("<Configure>", self._redraw)

        self.body = tk.Frame(self, bg=theme.CARD)
        self.body.pack(fill="both", expand=True, padx=self.PADDING, pady=self.PADDING)
        tk.Label(
            self.body, text=title.upper(), font=theme.heading, fg=theme.MUTED, bg=theme.CARD
        ).pack(anchor="w", pady=(0, 8))

    def _redraw(self, event: tk.Event) -> None:
        canvas = self._background
        canvas.delete("all")
        _rounded(
            canvas, 1, 1, event.width - 1, event.height - 1, 14,
            fill=self._theme.CARD, outline=self._theme.CARD_BORDER,
        )  # fmt: skip


class Button(tk.Canvas):
    """A flat, rounded button."""

    def __init__(
        self,
        parent: tk.Widget,
        theme: Theme,
        text: str,
        command: Callable[[], None],
        kind: str = "normal",
        width: int = 0,
        background: Optional[str] = None,
    ):
        self._theme = theme
        self._text = text
        self._command = command
        self._enabled = True
        self._hover = False
        self._fill, self._fill_hover, self._ink = {
            "normal": (theme.BUTTON, theme.BUTTON_HOVER, theme.TEXT),
            "accent": (theme.ACCENT, theme.ACCENT_HOVER, "#ffffff"),
            "danger": (theme.DANGER, theme.DANGER_HOVER, "#ffffff"),
        }[kind]
        font = tkfont.Font(font=theme.button)
        super().__init__(
            parent,
            width=max(width, font.measure(text) + 28),
            height=font.metrics("linespace") + 16,
            bg=background or theme.CARD,
            highlightthickness=0,
            cursor="hand2",
        )
        self.bind("<Configure>", lambda _event: self._redraw())
        self.bind("<Enter>", lambda _event: self._set_hover(True))
        self.bind("<Leave>", lambda _event: self._set_hover(False))
        self.bind("<ButtonRelease-1>", self._click)

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        self.configure(cursor="hand2" if enabled else "arrow")
        self._redraw()

    def set_text(self, text: str) -> None:
        self._text = text
        self._redraw()

    def _set_hover(self, hover: bool) -> None:
        self._hover = hover
        self._redraw()

    def _click(self, event: tk.Event) -> None:
        inside = 0 <= event.x <= self.winfo_width() and 0 <= event.y <= self.winfo_height()
        if self._enabled and inside:
            self._command()

    def _redraw(self) -> None:
        self.delete("all")
        width, height = self.winfo_width(), self.winfo_height()
        if not self._enabled:
            fill, ink = self._theme.BUTTON, self._theme.DISABLED_TEXT
        else:
            fill, ink = (self._fill_hover if self._hover else self._fill), self._ink
        _rounded(self, 0, 0, width, height, 9, fill=fill)
        self.create_text(
            width / 2, height / 2, text=self._text, fill=ink, font=self._theme.button
        )


class Slider(tk.Canvas):
    """A horizontal slider over ``-limit..limit`` that fills from the centre."""

    KNOB = 9

    def __init__(self, parent: tk.Widget, theme: Theme, on_change: Callable[[float], None]):
        super().__init__(parent, height=28, bg=theme.CARD, highlightthickness=0, cursor="hand2")
        self._theme = theme
        self._on_change = on_change
        self._enabled = True
        self.limit = 1.0
        self.value = 0.0
        self.bind("<Configure>", lambda _event: self._redraw())
        self.bind("<Button-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        self._redraw()

    def set_limit(self, limit: float) -> None:
        self.limit = max(limit, 1.0)
        self.set(self.value)

    def set(self, value: float) -> None:
        self.value = max(-self.limit, min(self.limit, value))
        self._redraw()
        self._on_change(self.value)

    def _span(self) -> tuple[float, float]:
        return self.KNOB + 2, self.winfo_width() - self.KNOB - 2

    def _drag(self, event: tk.Event) -> None:
        if not self._enabled:
            return
        left, right = self._span()
        fraction = (event.x - left) / max(right - left, 1)
        value = (2 * fraction - 1) * self.limit
        # Snap to zero near the centre, so that "stopped" is easy to hit.
        self.set(0.0 if abs(value) < self.limit * 0.03 else value)

    def _redraw(self) -> None:
        self.delete("all")
        theme = self._theme
        left, right = self._span()
        middle, y = (left + right) / 2, self.winfo_height() / 2
        x = middle + self.value / self.limit * (right - left) / 2
        accent = theme.ACCENT if self._enabled else theme.FAINT
        self.create_line(left, y, right, y, fill=theme.FIELD_BORDER, width=4, capstyle="round")
        self.create_line(middle, y - 6, middle, y + 6, fill=theme.FAINT, width=2)
        if x != middle:
            self.create_line(middle, y, x, y, fill=accent, width=4, capstyle="round")
        self.create_oval(
            x - self.KNOB, y - self.KNOB, x + self.KNOB, y + self.KNOB,
            fill=accent, outline=theme.CARD, width=3,
        )  # fmt: skip


class Dial(tk.Canvas):
    """The shaft angle: commanded position, target and encoder reading."""

    SIZE = 230

    def __init__(self, parent: tk.Widget, theme: Theme):
        super().__init__(
            parent, width=self.SIZE, height=self.SIZE, bg=theme.CARD, highlightthickness=0
        )
        self._theme = theme
        self.show(None)

    def show(self, status: Optional[pb.Motor]) -> None:
        self.delete("all")
        theme = self._theme
        centre, radius = self.SIZE / 2, self.SIZE / 2 - 14
        box = (centre - radius, centre - radius, centre + radius, centre + radius)
        self.create_oval(*box, outline=theme.CARD_BORDER, width=10)
        for tick in range(0, 360, 30):
            major = tick % 90 == 0
            outer = _on_circle(centre, radius - 11, tick)
            inner = _on_circle(centre, radius - (22 if major else 17), tick)
            self.create_line(
                *inner, *outer, fill=theme.MUTED if major else theme.FAINT, width=2 if major else 1
            )
        if status is None:
            return

        color = theme.STATE[MotorState(status.state)]
        # The part of the turn the shaft has gone through, as an arc.
        turn = status.position % 360
        if turn:
            self.create_arc(*box, start=90, extent=-turn, style="arc", outline=color, width=10)
        if status.target != status.position:
            x, y = _on_circle(centre, radius, status.target)
            self.create_oval(x - 5, y - 5, x + 5, y + 5, fill=theme.WINDOW, outline=color, width=2)
        if status.HasField("encoder"):
            x, y = _on_circle(centre, radius - 30, status.encoder.position)
            self.create_oval(x - 4, y - 4, x + 4, y + 4, fill=theme.WARNING, outline="")
        tip = _on_circle(centre, radius - 26, status.position)
        self.create_line(centre, centre, *tip, fill=theme.TEXT, width=4, capstyle="round")
        self.create_oval(
            centre - 8, centre - 8, centre + 8, centre + 8,
            fill=theme.CARD, outline=theme.TEXT, width=3,
        )  # fmt: skip


class ControlPanel(tk.Frame):
    def __init__(self, root: tk.Tk, port: str = DEFAULT_PORT):
        self.theme = theme = Theme(root)
        super().__init__(root, bg=theme.WINDOW, padx=18, pady=16)
        self.root = root
        self.axis: Optional[Axis] = None
        self._polls = 0
        #: Widgets that only make sense while connected.
        self._controls: list[tk.Widget] = []

        self.port = tk.StringVar(value=port)
        self.message = tk.StringVar()
        self.position = tk.StringVar(value="–")
        self.goto = tk.StringVar(value="0")
        self.spin_label = tk.StringVar()
        self.max_speed = tk.StringVar()
        self.acceleration = tk.StringVar()
        #: The small readouts under the dial, by name.
        self.readouts: dict[str, tk.StringVar] = {}

        root.configure(bg=theme.WINDOW)
        self._build()
        self.pack(fill="both", expand=True)
        root.bind("<Escape>", lambda _event: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.close)
        self._set_connected(False)
        self.after(POLL_MS, self._poll)

    # -- Layout ------------------------------------------------------------

    def _label(self, parent: tk.Widget, text: str = "", muted: bool = True, **options) -> tk.Label:
        """A text label matching its parent's background."""
        theme = self.theme
        options.setdefault("font", theme.body)
        options.setdefault("fg", theme.MUTED if muted else theme.TEXT)
        return tk.Label(parent, text=text, bg=parent["bg"], **options)

    def _build(self) -> None:
        theme = self.theme
        self.columnconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)

        header = tk.Frame(self, bg=theme.WINDOW)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        self._label(header, "Axis Driver", muted=False, font=theme.title).pack(side="left")
        self.connection_dot = tk.Canvas(
            header, width=12, height=12, bg=theme.WINDOW, highlightthickness=0
        )
        self.connection_dot.pack(side="left", padx=(16, 6))
        self.connection = self._label(header)
        self.connection.pack(side="left")
        self.connect_button = Button(
            header, theme, "Connect", self.toggle_connection, kind="accent", width=110,
            background=theme.WINDOW,
        )  # fmt: skip
        self.connect_button.pack(side="right")
        self._entry(header, self.port, width=16).pack(side="right", padx=10, ipady=5)
        self._label(header, "Port").pack(side="right")

        self._build_status().grid(row=1, column=0, sticky="nsew", padx=(0, 14))
        self._build_controls().grid(row=1, column=1, sticky="nsew")

        log = Card(self, theme, "Firmware log")
        log.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(14, 0))
        self.log = tk.Text(
            log.body, height=7, width=96, state="disabled", wrap="none", font=theme.mono,
            bg=theme.CARD, fg=theme.MUTED, relief="flat", highlightthickness=0, borderwidth=0,
        )  # fmt: skip
        self.log.pack(fill="both", expand=True)
        self.log.tag_configure("WARN", foreground=theme.WARNING)
        self.log.tag_configure("ERROR", foreground=theme.DANGER)

        tk.Label(
            self, textvariable=self.message, font=theme.body, fg=theme.DANGER, bg=theme.WINDOW,
            anchor="w",
        ).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(10, 0))  # fmt: skip

    def _build_status(self) -> Card:
        theme = self.theme
        card = Card(self, theme, "Status")
        body = card.body

        self.state = self._label(body, "–", font=theme.heading)
        self.state.place(relx=1.0, y=0, anchor="ne")

        self.dial = Dial(body, theme)
        self.dial.pack(pady=(2, 0))
        self._label(body, muted=False, font=theme.readout, textvariable=self.position).pack()
        self._label(body, "commanded position", font=theme.small).pack(pady=(0, 12))

        grid = tk.Frame(body, bg=theme.CARD)
        grid.pack(fill="x")
        grid.columnconfigure(1, weight=1)
        for row, name in enumerate(("Target", "Velocity", "Encoder", "Following error", "Homed")):
            self.readouts[name] = variable = tk.StringVar(value="–")
            self._label(grid, name).grid(row=row, column=0, sticky="w", pady=2)
            self._label(grid, muted=False, textvariable=variable, width=24, anchor="e").grid(
                row=row, column=1, sticky="e"
            )
        return card

    def _build_controls(self) -> tk.Frame:
        theme = self.theme
        column = tk.Frame(self, bg=theme.WINDOW)
        column.columnconfigure(0, weight=1, uniform="half")
        column.columnconfigure(1, weight=1, uniform="half")

        power = Card(column, theme, "Power")
        power.grid(row=0, column=0, columnspan=2, sticky="ew")
        row = tk.Frame(power.body, bg=theme.CARD)
        row.pack(fill="x")
        self._button(row, "Enable", self.enable, width=90).pack(side="left")
        self._button(row, "Disable", self.disable, width=90).pack(side="left", padx=8)
        self._button(row, "Stop  ·  Esc", self.stop, kind="danger", width=150).pack(side="right")

        position = Card(column, theme, "Position  ·  degrees")
        position.grid(row=1, column=0, columnspan=2, sticky="ew", pady=14)
        row = tk.Frame(position.body, bg=theme.CARD)
        row.pack(fill="x")
        entry = self._entry(row, self.goto, width=10)
        entry.pack(side="left", ipady=6)
        entry.bind("<Return>", lambda _event: self.move_to())
        self._button(row, "Go", self.move_to, kind="accent", width=70).pack(side="left", padx=8)
        self._button(row, "Set as zero", self.set_zero).pack(side="right")
        jog = tk.Frame(position.body, bg=theme.CARD)
        jog.pack(fill="x", pady=(10, 0))
        for index, degrees in enumerate(JOG_STEPS_DEG):
            jog.columnconfigure(index, weight=1, uniform="jog")
            self._button(jog, f"{degrees:+d}°", lambda degrees=degrees: self.jog(degrees)).grid(
                row=0, column=index, sticky="ew", padx=(0 if index == 0 else 6, 0)
            )

        velocity = Card(column, theme, "Velocity")
        velocity.grid(row=2, column=0, columnspan=2, sticky="ew")
        self.spin_slider = Slider(velocity.body, theme, self._show_spin_speed)
        self.spin_slider.pack(fill="x")
        self._controls.append(self.spin_slider)
        row = tk.Frame(velocity.body, bg=theme.CARD)
        row.pack(fill="x", pady=(8, 0))
        self._label(row, muted=False, textvariable=self.spin_label).pack(side="left")
        self._button(row, "Spin", self.spin, kind="accent", width=80).pack(side="right")
        self._button(row, "Zero", lambda: self.spin_slider.set(0)).pack(side="right", padx=8)

        limits = Card(column, theme, "Limits")
        limits.grid(row=3, column=0, sticky="nsew", pady=(14, 0), padx=(0, 7))
        fields = tk.Frame(limits.body, bg=theme.CARD)
        fields.pack(fill="x")
        fields.columnconfigure(0, weight=1)
        rows = (
            ("Max speed", "steps/s", self.max_speed),
            ("Acceleration", "steps/s²", self.acceleration),
        )
        for row_index, (label, unit, variable) in enumerate(rows):
            self._label(fields, label).grid(row=row_index, column=0, sticky="w", pady=3)
            entry = self._entry(fields, variable, width=8)
            entry.grid(row=row_index, column=1, ipady=4, padx=6)
            entry.bind("<Return>", lambda _event: self.apply_limits())
            self._label(fields, unit, font=theme.small).grid(row=row_index, column=2, sticky="w")
        self._button(limits.body, "Apply", self.apply_limits).pack(anchor="e", pady=(8, 0))

        homing = Card(column, theme, "Homing")
        homing.grid(row=3, column=1, sticky="nsew", pady=(14, 0), padx=(7, 0))
        self._label(
            homing.body, "Runs into the end stop and\nbacks off, using the encoder.",
            font=theme.small, justify="left",
        ).pack(anchor="w")  # fmt: skip
        self.home_button = self._button(homing.body, "Home", self.home, width=90)
        self.home_button.pack(anchor="e", side="bottom")
        return column

    def _button(
        self, parent: tk.Widget, text: str, command: Callable[[], None], **options
    ) -> Button:
        button = Button(parent, self.theme, text, command, **options)
        self._controls.append(button)
        return button

    def _entry(self, parent: tk.Widget, variable: tk.StringVar, width: int) -> tk.Entry:
        theme = self.theme
        return tk.Entry(
            parent, textvariable=variable, width=width, font=theme.body, justify="center",
            bg=theme.FIELD, fg=theme.TEXT, insertbackground=theme.TEXT, relief="flat",
            highlightthickness=1, highlightbackground=theme.FIELD_BORDER,
            highlightcolor=theme.ACCENT, selectbackground=theme.ACCENT,
            selectforeground="#ffffff",
        )  # fmt: skip

    # -- Connection --------------------------------------------------------

    def toggle_connection(self) -> None:
        if self.axis:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        self.message.set("")
        try:
            self.axis = Axis(self.port.get())
            info = self.axis.info()
            status = self.axis.status()
        except (OSError, AxisError, TimeoutError) as error:
            self._drop_connection("Not connected")
            self.message.set(f"Could not connect: {error}")
            return

        failed = [
            field.name.replace("_", " ")
            for field in info.self_test.DESCRIPTOR.fields
            if not getattr(info.self_test, field.name)
        ]
        if failed:
            self.message.set("Self test failed: " + ", ".join(failed))
        self.max_speed.set(str(status.motion.max_speed))
        self.acceleration.set(str(status.motion.acceleration))
        self.spin_slider.set_limit(status.motion.max_speed)
        self._set_connected(True, f"Connected  ·  firmware {info.firmware_version}")

    def disconnect(self) -> None:
        self._drop_connection("Not connected")

    def _drop_connection(self, description: str) -> None:
        if self.axis:
            try:
                self.axis.close()
            except OSError:
                pass
        self.axis = None
        self._set_connected(False, description)

    def _set_connected(self, connected: bool, description: str = "Not connected") -> None:
        theme = self.theme
        self.connection.configure(text=description)
        self.connection_dot.delete("all")
        self.connection_dot.create_oval(
            1, 1, 11, 11, fill=theme.OK if connected else theme.FAINT, outline=""
        )
        self.connect_button.set_text("Disconnect" if connected else "Connect")
        for widget in self._controls:
            widget.set_enabled(connected)
        if not connected:
            self._show_status(None)

    def close(self) -> None:
        """Switches the motor off and closes the window."""
        if self.axis:
            try:
                self.axis.disable()
            except (OSError, AxisError, TimeoutError):
                pass
            self._drop_connection("Not connected")
        self.root.destroy()

    # -- Commands ----------------------------------------------------------

    def _run(self, action: Callable[[Axis], object]) -> None:
        """Sends a command, reporting a refusal instead of raising."""
        if not self.axis:
            return
        self.message.set("")
        try:
            action(self.axis)
        except (AxisError, ValueError) as error:
            self.message.set(str(error))
        except (OSError, TimeoutError) as error:
            self._drop_connection("Connection lost")
            self.message.set(str(error))

    def enable(self) -> None:
        self._run(lambda axis: axis.enable())

    def disable(self) -> None:
        self._run(lambda axis: axis.disable())

    def stop(self) -> None:
        self._run(lambda axis: axis.stop(wait=False))

    def move_to(self) -> None:
        self._run(lambda axis: axis.move_to(float(self.goto.get())))

    def jog(self, degrees: float) -> None:
        # Relative to where the motor is now, which is what a jog button
        # should mean even right after spinning or homing.
        self._run(lambda axis: axis.move_to(axis.position + degrees))

    def set_zero(self) -> None:
        self._run(lambda axis: axis.set_position(0))

    def spin(self) -> None:
        self._run(lambda axis: axis.spin(round(self.spin_slider.value)))

    def home(self) -> None:
        self._run(lambda axis: axis.home())

    def apply_limits(self) -> None:
        def apply(axis: Axis) -> None:
            status = axis.set_motion(
                max_speed=int(self.max_speed.get()), acceleration=int(self.acceleration.get())
            )
            self.spin_slider.set_limit(status.motion.max_speed)

        self._run(apply)

    def _show_spin_speed(self, speed: float) -> None:
        rpm = round(speed) / STEPS_PER_REVOLUTION * 60
        self.spin_label.set(f"{round(speed):+d} steps/s   ·   {rpm:+.1f} rpm")

    # -- Live status -------------------------------------------------------

    def _poll(self) -> None:
        self.after(POLL_MS, self._poll)
        if not self.axis:
            return
        try:
            status = self.axis.status()
            self._polls += 1
            log = self.axis.read_log() if self._polls % LOG_EVERY_POLLS == 0 else ""
        except (OSError, AxisError, TimeoutError) as error:
            self._drop_connection("Connection lost")
            self.message.set(str(error))
            return
        self._show_status(status)
        if log:
            self._append_log(log)

    def _show_status(self, status: Optional[pb.Motor]) -> None:
        """Updates the readouts; ``None`` blanks them."""
        theme = self.theme
        self.dial.show(status)
        if status is None:
            self.state.configure(text="–", fg=theme.MUTED)
            self.position.set("–")
            for variable in self.readouts.values():
                variable.set("–")
            return

        state = MotorState(status.state)
        self.state.configure(text=f"●  {state.name}", fg=theme.STATE[state])
        self.position.set(f"{status.position:.2f}°")
        self.readouts["Target"].set(f"{status.target:.2f}°")
        rpm = status.velocity / STEPS_PER_REVOLUTION * 60
        # Adding zero turns a rounded "-0" into "0".
        self.readouts["Velocity"].set(
            f"{round(status.velocity) + 0} steps/s   ·   {round(rpm, 1) + 0:.1f} rpm"
        )
        has_encoder = status.HasField("encoder")
        if has_encoder:
            encoder = status.encoder
            self.readouts["Encoder"].set(
                f"{encoder.position:.1f}°   ·   {round(encoder.velocity) + 0}°/s"
            )
            # How far the shaft is from where it was told to go.
            self.readouts["Following error"].set(f"{encoder.position - status.position:+.1f}°")
        else:
            self.readouts["Encoder"].set("not available")
            self.readouts["Following error"].set("–")
        self.readouts["Homed"].set("yes" if status.homed else "no")
        self.home_button.set_enabled(has_encoder)

    def _append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        for line in text.splitlines(keepends=True):
            level = next((level for level in ("WARN", "ERROR") if f" {level} " in line), None)
            self.log.insert("end", line, (level,) if level else ())
        self.log.see("end")
        self.log.configure(state="disabled")


def _on_circle(centre: float, radius: float, degrees: float) -> tuple[float, float]:
    """Point on a circle, with zero degrees at the top and angles clockwise."""
    angle = math.radians(degrees)
    return centre + radius * math.sin(angle), centre - radius * math.cos(angle)


def main(port: str = DEFAULT_PORT, connect: bool = True) -> None:
    root = tk.Tk()
    root.title("Axis Driver")
    panel = ControlPanel(root, port)
    if connect:
        panel.connect()
    root.mainloop()
