#!/usr/bin/env python3
"""
mock_ui.py — Interactive Mock System Debugger UI
=================================================

Combines mock_encoder.py (ROS2 /encoder_raw) and mock_robot_controller.py
(LSL ActualStates / EventTrigger) into a single interactive tkinter window.

Drag the knobs to set position. Toggle Sync to mirror encoder → actual states.
Press Reset under a knob to zero the accumulated angle (simulates controller reset).
Type commands in the input field to send EventTrigger events.

Usage:
    source /opt/ros/<distro>/setup.bash
    source install/setup.bash
    python3 src/claude_visualizer/mock_UI/mock_ui.py
"""

import argparse
import json
import math
import os
import sys
import time
import tkinter as tk
from pathlib import Path

import pylsl
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from claude_visualizer_interface.msg import EncoderRaw, EventTrigger
except ImportError:
    sys.exit(
        "ERROR: rclpy or claude_visualizer_interface not found.\n"
        "Source your ROS2 workspace first:  source install/setup.bash"
    )

from claude_visualizer.utils import create_outlet


# ── Constants ─────────────────────────────────────────────────────────────────

PUBLISH_PERIOD_MS  = 20    # 50 Hz
ROS_SPIN_PERIOD_MS = 50    # 20 Hz

# Encoder resolution is no longer hardcoded — it is read from the shared
# params.yaml (mock_encoder.ros__parameters.ticks_per_rev) so the mock UI always
# matches the encoder / Kalman filter. See _load_ticks_per_rev() below; the value
# is resolved per-node in MockUINode.__init__ (self._ticks_per_rev / _ticks_per_rad).
# TICKS_PER_REV = 4096
# TICKS_PER_RAD = TICKS_PER_REV / (2.0 * math.pi)

CONFIG_FILENAME       = "params.yaml"
DEFAULT_TICKS_PER_REV = 8192          # valid hardware value; fallback if params.yaml is missing


def _find_params_yaml() -> "Path | None":
    """Locate params.yaml: installed share dir first, then repo-relative fallback.

    Mirrors mock_robot_controller._find_config so both stand-in tools resolve the
    same shared config whether run via `ros2 run` or `python3 mock_ui.py`.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory("claude_visualizer")) / "config" / CONFIG_FILENAME
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    # repo-relative: .../claude_visualizer/mock_UI/mock_ui.py -> .../claude_visualizer/config/params.yaml
    candidate = Path(__file__).resolve().parent.parent / "config" / CONFIG_FILENAME
    if candidate.is_file():
        return candidate
    return None


def _load_ticks_per_rev() -> int:
    """Read ticks_per_rev from params.yaml (mock_encoder section).

    Returns DEFAULT_TICKS_PER_REV if the file/key can't be resolved so the debug UI
    still launches instead of crashing.
    """
    path = _find_params_yaml()
    if path is None:
        return DEFAULT_TICKS_PER_REV
    try:
        with open(path, "r") as f:
            doc = yaml.safe_load(f) or {}
        val = doc.get("mock_encoder", {}).get("ros__parameters", {}).get("ticks_per_rev")
        return int(val) if val is not None else DEFAULT_TICKS_PER_REV
    except Exception:
        return DEFAULT_TICKS_PER_REV

ACTUAL_STATES_CFG = {
    "name":             "ActualStates",
    "type":             "States",
    "channel":          ["actual_position", "actual_velocity", "actual_acceleration"],
    "outlet_type":      "REGULAR",
    "sampling_rate_hz": 50.0,
    "source_id":        "mock_ui-actual_states",
}

EVENT_TRIGGER_CFG = {
    "name":        "EventTrigger",
    "type":        "Trigger",
    "channel":     ["json_payload"],
    "outlet_type": "IRREGULAR",
    "source_id":   "mock_ui-event_trigger",
}


# def _pair_suffix(pair_id=None) -> str:
#     """Per-pair LSL suffix: explicit arg wins, else CV_PAIR_ID env. 0/empty = none."""
#     pid = str(pair_id if pair_id is not None else os.environ.get("CV_PAIR_ID", "")).strip()
#     return f"_{pid}" if pid and pid != "0" else ""
def _group_suffix(group_number=None) -> str:
    """Per-group LSL suffix (UNCHANGED behaviour): explicit arg wins, else
    CV_GROUP_NUMBER env. 0/empty = none, N → '_N'."""
    n = str(group_number if group_number is not None else os.environ.get("CV_GROUP_NUMBER", "")).strip()
    return f"_{n}" if n and n != "0" else ""


def _group_namespace(group_number=None) -> str:
    """Per-group ROS namespace token 'G<N>' — ALWAYS present (default N=0 → 'G0')."""
    n = str(group_number if group_number is not None else os.environ.get("CV_GROUP_NUMBER", "")).strip() or "0"
    return f"G{n}"

# Dial geometry (all in pixels from canvas centre)
_DIAL_R    = 95    # outer edge of tick ring / background circle
_T_BASE    = 87    # inner base of all ticks
_T_MINOR   = 92    # tip of 1° minor ticks
_T_MAJOR   = 97    # tip of 45° major ticks
_KNOB_R    = 80    # filled knob disc radius
_NEEDLE_L  = 66    # needle length from centre
_LABEL_R   = 108   # degree-label distance from centre

_CS = (_LABEL_R + 24) * 2   # canvas pixel size  ≈ 264


# ══════════════════════════════════════════════════════════════════════════════
# KnobWidget
# ══════════════════════════════════════════════════════════════════════════════

class KnobWidget(tk.Frame):
    """
    360° encoder knob with 1° tick ring and a Reset button.

    Tracks un-wrapped accumulated angle (can exceed ±360°), matching the
    continuous-position convention used by mock_encoder.py and
    mock_robot_controller.py.  The needle shows accumulated_deg % 360 on the
    fixed dial face.
    """

    SENSITIVITY_ROUGH = 1.0   # degrees per pixel of drag — rough mode
    SENSITIVITY_FINE  = 0.1   # degrees per pixel of drag — fine mode

    def __init__(self, parent, unit: str = "deg", **kwargs) -> None:
        super().__init__(parent, bg="#1e1f22", **kwargs)
        self._unit = unit
        self._accumulated_deg: float = 0.0
        self._dragging = False
        self._last_y = 0
        self._sensitivity = self.SENSITIVITY_ROUGH

        self._canvas = tk.Canvas(
            self, width=_CS, height=_CS,
            bg="#1e1f22", highlightthickness=0,
        )
        self._canvas.pack()

        readout_color = "#ff6b35" if unit == "rad" else "#4ec9b0"
        self._readout = tk.Label(
            self, text=self._fmt(),
            bg="#1e1f22", fg=readout_color,
            font=("Courier", 10, "bold"),
        )
        self._readout.pack(pady=(2, 0))

        tk.Button(
            self, text="Reset", width=8,
            bg="#3c3f41", fg="#aaaaaa",
            activebackground="#4c4f51",
            relief=tk.FLAT,
            command=self.reset,
        ).pack(pady=(4, 6))

        self._canvas.bind("<ButtonPress-1>",   self._on_press)
        self._canvas.bind("<B1-Motion>",       self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)
        self._canvas.bind("<MouseWheel>",      self._on_scroll)
        self._canvas.bind("<Button-4>",        self._on_scroll)  # Linux scroll up
        self._canvas.bind("<Button-5>",        self._on_scroll)  # Linux scroll down

        self._draw_background()
        self._redraw_needle()

    # ── Public API ────────────────────────────────────────────────────────────

    def get_physical_rad(self) -> float:
        return math.radians(self._accumulated_deg)

    def get_physical_deg(self) -> float:
        return self._accumulated_deg

    def get_angle_deg(self) -> float:
        return self._accumulated_deg

    def set_angle_deg(self, angle: float) -> None:
        self._accumulated_deg = float(angle)
        self._readout.config(text=self._fmt())
        self._redraw_needle()

    def set_readout_text(self, text: str) -> None:
        self._readout.config(text=text)

    def reset(self) -> None:
        self._accumulated_deg = 0.0
        self._readout.config(text=self._fmt())
        self._redraw_needle()

    def set_sensitivity(self, fine: bool) -> None:
        self._sensitivity = self.SENSITIVITY_FINE if fine else self.SENSITIVITY_ROUGH

    # ── Internal drawing ──────────────────────────────────────────────────────

    @staticmethod
    def _cx() -> int:
        return _CS // 2

    @staticmethod
    def _cy() -> int:
        return _CS // 2

    def _fmt(self) -> str:
        deg = self._accumulated_deg
        rad = math.radians(deg)
        return f"{rad:+.3f} rad\n{deg:+.1f}°"

    def _draw_background(self) -> None:
        cx, cy = self._cx(), self._cy()

        # Dark background disc
        self._canvas.create_oval(
            cx - _DIAL_R, cy - _DIAL_R,
            cx + _DIAL_R, cy + _DIAL_R,
            fill="#2b2b2b", outline="#444444", width=2,
        )

        # 1° minor ticks
        for deg in range(360):
            rad = math.radians(deg)
            s, c = math.sin(rad), math.cos(rad)
            self._canvas.create_line(
                cx + _T_BASE  * s, cy - _T_BASE  * c,
                cx + _T_MINOR * s, cy - _T_MINOR * c,
                fill="#555555", width=1,
            )

        # 45° major ticks + labels
        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            s, c = math.sin(rad), math.cos(rad)
            self._canvas.create_line(
                cx + (_T_BASE - 2) * s, cy - (_T_BASE - 2) * c,
                cx + _T_MAJOR      * s, cy - _T_MAJOR      * c,
                fill="#aaaaaa", width=2,
            )
            self._canvas.create_text(
                cx + _LABEL_R * s,
                cy - _LABEL_R * c,
                text=str(deg),
                fill="#cccccc",
                font=("Helvetica", 7),
            )

        # Knob disc — on top of tick ring
        self._canvas.create_oval(
            cx - _KNOB_R, cy - _KNOB_R,
            cx + _KNOB_R, cy + _KNOB_R,
            fill="#3c3f41", outline="#666666", width=2,
        )

    def _redraw_needle(self) -> None:
        cx, cy = self._cx(), self._cy()
        self._canvas.delete("needle")

        visual_rad = math.radians(self._accumulated_deg % 360.0)
        nx = cx + _NEEDLE_L * math.sin(visual_rad)
        ny = cy - _NEEDLE_L * math.cos(visual_rad)

        self._canvas.create_line(
            cx, cy, nx, ny,
            fill="#ff6b35", width=3, capstyle=tk.ROUND,
            tags="needle",
        )
        self._canvas.create_oval(
            cx - 5, cy - 5, cx + 5, cy + 5,
            fill="#ff6b35", outline="",
            tags="needle",
        )

    # ── Input handlers ────────────────────────────────────────────────────────

    def _on_press(self, event) -> None:
        self._dragging = True
        self._last_y = event.y

    def _on_drag(self, event) -> None:
        if not self._dragging:
            return
        delta = (event.y - self._last_y) * self._sensitivity
        self._last_y = event.y
        self._accumulated_deg += delta
        self._readout.config(text=self._fmt())
        self._redraw_needle()

    def _on_release(self, _event) -> None:
        self._dragging = False

    def _on_scroll(self, event) -> None:
        step = self._sensitivity
        if event.num == 5 or (hasattr(event, "delta") and event.delta < 0):
            self._accumulated_deg += step   # scroll down = clockwise
        else:
            self._accumulated_deg -= step   # scroll up = anticlockwise
        self._readout.config(text=self._fmt())
        self._redraw_needle()


# ══════════════════════════════════════════════════════════════════════════════
# MockUINode
# ══════════════════════════════════════════════════════════════════════════════

class MockUINode(Node):
    """Minimal rclpy node — publishes /encoder_raw only."""

    def __init__(self, namespace: str = "G0") -> None:
        # super().__init__("mock_ui")
        super().__init__("mock_ui", namespace=namespace)   # /G<N>/… topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Relative topic names so the node's namespace (/G<N>) is prepended.
        # self._pub = self.create_publisher(EncoderRaw, "/encoder_raw", qos)
        # self._event_pub = self.create_publisher(EventTrigger, "/event_trigger", qos)
        self._pub = self.create_publisher(EncoderRaw, "encoder_raw", qos)
        self._event_pub = self.create_publisher(EventTrigger, "event_trigger", qos)
        # encoder resolution from the shared params.yaml (matches mock_encoder / KF)
        self._ticks_per_rev = _load_ticks_per_rev()
        self._ticks_per_rad = self._ticks_per_rev / (2.0 * math.pi)
        self.get_logger().info(
            f"mock_ui: /encoder_raw publisher ready  ticks/rev={self._ticks_per_rev}"
        )

    def publish_encoder(self, angle_rad: float, ros_stamp, dt_us: int) -> None:
        # ticks = int(angle_rad * TICKS_PER_RAD)
        ticks = int(angle_rad * self._ticks_per_rad)
        msg = EncoderRaw()
        msg.header.stamp    = ros_stamp.to_msg()
        msg.header.frame_id = "mock_ui"
        msg.ticks           = ticks
        # msg.raw_position    = ticks / TICKS_PER_RAD
        msg.raw_position    = ticks / self._ticks_per_rad
        msg.dt_us           = max(0, dt_us)
        self._pub.publish(msg)

    def publish_event_trigger(self, payload: dict) -> None:
        msg = EventTrigger()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "mock_ui"
        msg.event    = json.dumps(payload)
        self._event_pub.publish(msg)

    def shutdown(self) -> None:
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════════
# MockUI
# ══════════════════════════════════════════════════════════════════════════════

class MockUI:
    """Top-level tkinter application."""

    def __init__(self, group_number=None) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        # self._ros_node = MockUINode()
        self._ros_node = MockUINode(namespace=_group_namespace(group_number))

        # Per-group LSL isolation: suffix stream name + source_id (matches verifier + other mocks).
        # _suffix = _pair_suffix(pair_id)
        _suffix = _group_suffix(group_number)
        _act_cfg = {**ACTUAL_STATES_CFG,
                    "name":      ACTUAL_STATES_CFG["name"] + _suffix,
                    "source_id": ACTUAL_STATES_CFG["source_id"] + _suffix}
        _evt_cfg = {**EVENT_TRIGGER_CFG,
                    "name":      EVENT_TRIGGER_CFG["name"] + _suffix,
                    "source_id": EVENT_TRIGGER_CFG["source_id"] + _suffix}
        # self._actual_states_outlet = create_outlet(ACTUAL_STATES_CFG, pylsl.cf_float32)
        # self._event_trigger_outlet = create_outlet(EVENT_TRIGGER_CFG, pylsl.cf_string)
        self._actual_states_outlet = create_outlet(_act_cfg, pylsl.cf_float32)
        self._event_trigger_outlet = create_outlet(_evt_cfg, pylsl.cf_string)

        self._root = tk.Tk()
        self._root.title("Mock UI Debugger")
        self._root.resizable(False, False)
        self._root.configure(bg="#1e1f22")

        self._sync_var     = tk.BooleanVar(value=False)
        self._fine_var     = tk.BooleanVar(value=False)
        self._enc_last_deg: float = 0.0
        # track the actual-states knob too so Sync can mirror in BOTH directions
        self._act_last_deg: float = 0.0
        self._running      = True

        # State for ActualStates velocity / acceleration (numerical differentiation)
        self._t_last    = time.monotonic()
        self._pos_last  = 0.0
        self._vel_last  = 0.0

        self._build_ui()
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._root.after(PUBLISH_PERIOD_MS,  self._publish_loop)
        self._root.after(ROS_SPIN_PERIOD_MS, self._ros_spin_loop)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        PAD = {"padx": 12, "pady": 8}

        # ── Row 0: two knobs side by side ─────────────────────────────────────
        knob_frame = tk.Frame(self._root, bg="#1e1f22")
        knob_frame.grid(row=0, column=0, **PAD)

        enc_lf = tk.LabelFrame(
            knob_frame, text="  Encoder  /encoder_raw  (ROS2)  ",
            bg="#1e1f22", fg="#aaaaaa", font=("Helvetica", 9),
        )
        enc_lf.grid(row=0, column=0, padx=10, pady=6)
        self._encoder_knob = KnobWidget(enc_lf, unit="rad")
        self._encoder_knob.pack(padx=8, pady=8)

        act_lf = tk.LabelFrame(
            knob_frame, text="  ActualStates  LSL  ",
            bg="#1e1f22", fg="#aaaaaa", font=("Helvetica", 9),
        )
        act_lf.grid(row=0, column=1, padx=10, pady=6)
        self._actual_knob = KnobWidget(act_lf, unit="deg")
        self._actual_knob.pack(padx=8, pady=8)

        # Wrap actual-knob reset to also clear vel/accel history so there is
        # no spurious velocity spike from the position discontinuity.
        _orig_actual_reset = self._actual_knob.reset
        def _actual_reset_with_clear() -> None:
            _orig_actual_reset()
            self._pos_last = 0.0
            self._vel_last = 0.0
        self._actual_knob.reset = _actual_reset_with_clear  # type: ignore[method-assign]

        # ── Row 1: Sync + Fine/Rough toggles ─────────────────────────────────
        sync_frame = tk.Frame(self._root, bg="#1e1f22")
        sync_frame.grid(row=1, column=0, pady=4)
        self._sync_btn = tk.Button(
            sync_frame, text="Sync: OFF", width=14,
            bg="#3c3f41", fg="#aaaaaa",
            activebackground="#4c4f51",
            relief=tk.FLAT,
            command=self._toggle_sync,
        )
        self._sync_btn.pack(side=tk.LEFT, padx=(0, 8))
        self._fine_btn = tk.Button(
            sync_frame, text="Mode: ROUGH", width=14,
            bg="#3c3f41", fg="#aaaaaa",
            activebackground="#4c4f51",
            relief=tk.FLAT,
            command=self._toggle_fine,
        )
        self._fine_btn.pack(side=tk.LEFT)

        # ── Row 2: command input ──────────────────────────────────────────────
        cmd_frame = tk.Frame(self._root, bg="#1e1f22")
        cmd_frame.grid(row=2, column=0, sticky="ew", padx=12, pady=6)

        tk.Label(
            cmd_frame, text="Command:", bg="#1e1f22", fg="#aaaaaa",
            font=("Helvetica", 10),
        ).pack(side=tk.LEFT, padx=(0, 6))

        self._cmd_entry = tk.Entry(
            cmd_frame, width=36,
            bg="#3c3f41", fg="#ffffff",
            insertbackground="white",
            font=("Courier", 10),
        )
        self._cmd_entry.pack(side=tk.LEFT)
        self._cmd_entry.bind("<Return>", lambda _e: self._send_command())

        tk.Button(
            cmd_frame, text="Send", width=6,
            bg="#4a7c59", fg="#ffffff",
            activebackground="#5a9c69",
            relief=tk.FLAT,
            command=self._send_command,
        ).pack(side=tk.LEFT, padx=(6, 0))

        # ── Row 3: hint ───────────────────────────────────────────────────────
        tk.Label(
            self._root,
            text=(
                "ptp <val> <unit>  |  pp <n> <seq> <dirs> <grip>  |  "
                "perf <spd> <acc>  |  prec <i> <t> <r> <u>  |  stop"
            ),
            bg="#1e1f22", fg="#555555",
            font=("Helvetica", 7),
        ).grid(row=3, column=0, padx=12, pady=(0, 2))

        # ── Row 4: status ─────────────────────────────────────────────────────
        self._status_label = tk.Label(
            self._root, text="Ready.",
            bg="#1e1f22", fg="#6a9955",
            font=("Helvetica", 9), anchor="w",
        )
        self._status_label.grid(row=4, column=0, sticky="ew", padx=12, pady=(0, 8))

    # ── Publish loops ─────────────────────────────────────────────────────────

    def _publish_loop(self) -> None:
        if not self._running:
            return

        # Capture both timestamps at the same instant so the two streams
        # reference the same wall-clock moment.
        t_now     = time.monotonic()
        ros_stamp = self._ros_node.get_clock().now()
        lsl_stamp = pylsl.local_clock()

        dt = t_now - self._t_last
        self._t_last = t_now
        dt_us = max(0, int(dt * 1e6))

        # ── previous one-way sync (encoder → actual only) ──
        # enc_deg_now = self._encoder_knob.get_physical_deg()
        # if self._sync_var.get():
        #     enc_delta = enc_deg_now - self._enc_last_deg
        #     self._actual_knob.set_angle_deg(
        #         self._actual_knob.get_physical_deg() + enc_delta
        #     )
        # self._enc_last_deg = enc_deg_now

        # Sync: mirror movement in BOTH directions. Whichever knob the user moved
        # this frame is the master and drives the other by the same delta; the
        # encoder wins ties (only one knob is draggable at a time, so ties are rare).
        enc_deg_now = self._encoder_knob.get_physical_deg()
        act_deg_now = self._actual_knob.get_physical_deg()
        if self._sync_var.get():
            enc_delta = enc_deg_now - self._enc_last_deg
            act_delta = act_deg_now - self._act_last_deg
            if enc_delta != 0.0:                       # encoder moved → drive actual
                act_deg_now = act_deg_now + enc_delta
                self._actual_knob.set_angle_deg(act_deg_now)
            elif act_delta != 0.0:                     # actual moved → drive encoder
                enc_deg_now = enc_deg_now + act_delta
                self._encoder_knob.set_angle_deg(enc_deg_now)
        # update trackers unconditionally so toggling Sync ON never causes a jump
        self._enc_last_deg = enc_deg_now
        self._act_last_deg = act_deg_now

        # ── EncoderRaw (ROS2) ─────────────────────────────────────────────────
        self._ros_node.publish_encoder(
            self._encoder_knob.get_physical_rad(), ros_stamp, dt_us
        )

        # ── ActualStates (LSL) — velocity and acceleration derived the same
        # way as mock_robot_controller.py lines 229-232 ───────────────────────
        pos_deg = self._actual_knob.get_physical_deg()
        vel   = (pos_deg - self._pos_last) / dt if dt > 0 else 0.0
        accel = (vel - self._vel_last)     / dt if dt > 0 else 0.0
        self._pos_last = pos_deg
        self._vel_last = vel

        self._actual_states_outlet.push_sample(
            [pos_deg, vel, accel], timestamp=lsl_stamp
        )

        pos_rad = math.radians(pos_deg)
        self._actual_knob.set_readout_text(
            f"{pos_rad:+.3f} rad  {pos_deg:+.1f}°\nv:{vel:+.2f}  a:{accel:+.2f}"
        )

        self._root.after(PUBLISH_PERIOD_MS, self._publish_loop)

    def _ros_spin_loop(self) -> None:
        if not self._running:
            return
        try:
            rclpy.spin_once(self._ros_node, timeout_sec=0)
        except Exception:
            pass
        self._root.after(ROS_SPIN_PERIOD_MS, self._ros_spin_loop)

    # ── Command handling ──────────────────────────────────────────────────────

    def _toggle_sync(self) -> None:
        new_val = not self._sync_var.get()
        self._sync_var.set(new_val)
        self._sync_btn.config(
            text="Sync: ON"  if new_val else "Sync: OFF",
            fg="#4ec9b0"     if new_val else "#aaaaaa",
        )

    def _toggle_fine(self) -> None:
        new_val = not self._fine_var.get()
        self._fine_var.set(new_val)
        self._fine_btn.config(
            text="Mode: FINE"  if new_val else "Mode: ROUGH",
            fg="#c586c0"       if new_val else "#aaaaaa",
        )
        self._encoder_knob.set_sensitivity(new_val)
        self._actual_knob.set_sensitivity(new_val)

    def _send_command(self) -> None:
        raw = self._cmd_entry.get().strip()
        if not raw:
            return
        parts = raw.split()
        cmd = parts[0].lower()
        try:
            payload = self._parse_command(cmd, parts[1:])
        except (ValueError, IndexError) as exc:
            self._set_status(f"Error: {exc}", error=True)
            return
        self._event_trigger_outlet.push_sample([json.dumps(payload)])
        self._ros_node.publish_event_trigger(payload)
        self._cmd_entry.delete(0, tk.END)
        self._set_status(f"Sent: {json.dumps(payload)[:72]}")

    @staticmethod
    def _parse_command(cmd: str, args: list) -> dict:
        if cmd == "ptp":
            if len(args) != 2:
                raise ValueError("ptp <value> <unit>")
            return {
                "mode": "Auto", "action": "point_to_point",
                "value": float(args[0]), "unit": args[1],
            }
        if cmd == "pp":
            if len(args) != 4:
                raise ValueError("pp <n> <seq> <dirs> <gripper>")
            return {
                "mode": "Auto", "action": "pick_place",
                "num":         int(args[0]),
                "order_sequence":    [int(x) for x in args[1].split(",")],
                "directions":  args[2].split(","),
                "use_gripper": args[3].lower() in ("true", "1", "yes"),
            }
        if cmd == "perf":
            if len(args) != 2:
                raise ValueError("perf <speed> <accel>")
            return {
                "mode": "Test", "action": "performance",
                "speed": float(args[0]), "accel": float(args[1]),
            }
        if cmd == "prec":
            if len(args) != 4:
                raise ValueError("prec <init_pos> <tar_pos> <repeat> <unit>")
            return {
                "mode": "Test", "action": "precision",
                "init_pos": int(args[0]), "target_pos": int(args[1]),
                "repeat":   int(args[2]), "unit": args[3],
            }
        if cmd == "stop":
            return {"mode": "STOP", "action": "stop"}
        raise ValueError(f"unknown '{cmd}' — use: ptp | pp | perf | prec | stop")

    def _set_status(self, msg: str, error: bool = False) -> None:
        self._status_label.config(
            text=msg, fg="#f44747" if error else "#6a9955"
        )
        self._root.after(4000, self._clear_status)

    def _clear_status(self) -> None:
        if self._running:
            self._status_label.config(text="Ready.", fg="#6a9955")

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def _on_close(self) -> None:
        self._running = False
        self._ros_node.shutdown()
        self._root.destroy()

    def run(self) -> None:
        self._root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # MockUI().run()
    parser = argparse.ArgumentParser(description="Mock UI Debugger")
    parser.add_argument(
        # "--pair-id",
        "--group-number",
        default=None,
        help="Group number N: ROS namespace /G<N> + LSL suffix (_N, none for 0). "
             "Overrides CV_GROUP_NUMBER env.",
    )
    args = parser.parse_args()
    # MockUI(pair_id=args.pair_id).run()
    MockUI(group_number=args.group_number).run()


if __name__ == "__main__":
    main()
