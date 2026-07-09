#!/usr/bin/env python3
"""
mock_robot_controller — synthetic LSL publisher (Phase 2 stand-in)
==================================================================

NOT a ROS node. Pure Python + pylsl. Stands in for the real
robot_controller.py (Phase 4) so we can exercise the LSL ↔ ROS bridge
end-to-end before the actual controller exists.

Configuration comes from the shared params.yaml under
  mock_robot_controller: ros__parameters:
The wrapper is required by ROS's parameter parser even though this script
is not a ROS node — ROS flattens the nested keys to dotted parameter names
and the script reads them via PyYAML.
The CLI --waveform flag, if given, overrides the YAML choice.

LSL outlets:
  "ActualStates"  — 3 float channels:
      [actual_position, actual_velocity, actual_acceleration]

  "EventTrigger"  — 1 string channel, irregular rate, full JSON payload

Commands:
  ptp <value> <unit>                  — point_to_point  (unit: index|degree|rad)
  pp  <n> <seq> <dirs> <gripper>     — pick_place       (seq: comma-separated indices)
  perf <speed> <accel>               — performance
  prec <init> <tar> <repeat> <unit>  — precision
  stop                               — send STOP trigger
  quit / q                           — exit

Examples:
  ptp 5 index
  pp 3 0,36,72 CW,CCW,CW true
  perf 1.0 2.0
  prec 0 36 10 index

The web_visualizer ROS node picks up both LSL streams and forwards them
to the JS frontend over WebSocket.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import pylsl
import yaml

from claude_visualizer.utils import create_outlet


# ── Config loading ────────────────────────────────────────────────────────────

CONFIG_FILENAME = "params.yaml"


def _find_config(explicit: str | None) -> Path:
    """Locate params.yaml. Priority:
        1. --config CLI flag
        2. Installed share dir via ament_index_python (works under `ros2 run`)
        3. Repo-relative fallback (works for direct `python3 mock_robot_controller.py`)
    """
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(f"--config not found: {p}")
        return p

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("claude_visualizer"))
        candidate = share / "config" / CONFIG_FILENAME
        if candidate.is_file():
            return candidate
    except Exception:
        pass

    here = Path(__file__).resolve().parent
    candidate = here.parent / "config" / CONFIG_FILENAME
    if candidate.is_file():
        return candidate

    raise FileNotFoundError(
        f"Could not locate {CONFIG_FILENAME}. Pass --config <path> explicitly."
    )


def load_config(explicit_path: str | None) -> dict:
    path = _find_config(explicit_path)
    with open(path, "r") as f:
        doc = yaml.safe_load(f) or {}
    section = doc.get("mock_robot_controller", {}).get("ros__parameters", {})
    if "lsl_params" not in section or "waveform_config" not in section:
        raise KeyError(
            f"Expected 'mock_robot_controller.ros__parameters.lsl_params' and "
            f"'.waveform_config' in {path}"
        )
    print(f"[mock_robot_controller] config: {path}", file=sys.stderr)
    return section


# ── LSL outlet builders ───────────────────────────────────────────────────────

# def create_outlet(stream_cfg: dict, msg_format) -> pylsl.StreamOutlet:
#     outlet_type = stream_cfg.get("outlet_type")
#     if outlet_type == "REGULAR":
#         nominal_sampling_rate = float(stream_cfg.get("sampling_rate_hz", 100.0))
#     elif outlet_type == "IRREGULAR":
#         nominal_sampling_rate = pylsl.IRREGULAR_RATE
#     else:
#         raise ValueError(
#             f"unknown outlet_type {outlet_type!r} for stream "
#             f"{stream_cfg.get('name', '?')!r}; expected 'REGULAR' or 'IRREGULAR'"
#         )

#     info = pylsl.StreamInfo(
#         name=str(stream_cfg.get("name", "")),
#         type=str(stream_cfg.get("type", "")),
#         channel_count=len(stream_cfg.get("channel", [])),
#         nominal_srate=nominal_sampling_rate,
#         channel_format=msg_format,
#         source_id=str(stream_cfg.get("source_id", "")),
#     )

#     channels = info.desc().append_child("channels")
#     for label in stream_cfg.get("channel", []):
#         ch = channels.append_child("channel")
#         ch.append_child_value("label", label)
#     return pylsl.StreamOutlet(info)


# ── Waveforms ─────────────────────────────────────────────────────────────────

class SineWaveform:
    """pos(t) = A · sin(2π·f·t)"""

    def __init__(self, amplitude: float, frequency_hz: float) -> None:
        self.amplitude = amplitude
        self.frequency = frequency_hz

    def position(self, t: float) -> float:
        return self.amplitude * math.sin(2.0 * math.pi * self.frequency * t)


class TrapezoidWaveform:
    """Symmetric trapezoidal velocity profile. Same shape as mock_encoder
    uses so the 'commanded' line overlays cleanly on the encoder estimate."""

    def __init__(self, v_max: float, a: float, cruise: float) -> None:
        self.v_max = v_max
        self.a = a
        self.t_ramp = v_max / a
        self.t_cruise = cruise
        self.t_half = self.t_ramp + cruise + self.t_ramp
        self.t_period = self.t_half * 2.0
        self.d_ramp = 0.5 * a * self.t_ramp ** 2
        self.d_cruise = v_max * cruise
        self.d_half = self.d_ramp + self.d_cruise + self.d_ramp

    def position(self, t: float) -> float:
        t_mod = math.fmod(t, self.t_period)
        if t_mod >= self.t_half:
            return self.d_half - self._half(t_mod - self.t_half)
        return self._half(t_mod)

    def _half(self, t: float) -> float:
        if t < self.t_ramp:
            return 0.5 * self.a * t ** 2
        t -= self.t_ramp
        if t < self.t_cruise:
            return self.d_ramp + self.v_max * t
        t -= self.t_cruise
        return self.d_ramp + self.d_cruise + self.v_max * t - 0.5 * self.a * t ** 2


class StepWaveform:
    """Alternates between 0 and amplitude every half-period — stress-tests the KF."""

    def __init__(self, amplitude: float, period: float) -> None:
        self.amplitude = amplitude
        self.period = period

    def position(self, t: float) -> float:
        return self.amplitude if math.fmod(t, self.period) >= self.period / 2.0 else 0.0


def build_waveform(name: str, waveform_cfg: dict):
    if name == "sine":
        return SineWaveform(
            float(waveform_cfg.get("sine_amplitude", 1.0)), 
            float(waveform_cfg.get("sine_frequency_hz", 0.5))
        )
    if name == "trapezoid":
        return TrapezoidWaveform(
            float(waveform_cfg.get("trap_max_velocity", 2.0)), 
            float(waveform_cfg.get("trap_acceleration", 2.0)), 
            float(waveform_cfg.get("trap_cruise_time_s", 1.0))
        )
    if name == "step":
        return StepWaveform(
            float(waveform_cfg.get("step_amplitude", 4.0)), 
            float(waveform_cfg.get("step_period_s", 2.0))
        )
    raise ValueError(f"unknown waveform: {name!r} (use trapezoid|sine|step)")


# ── Telemetry thread ──────────────────────────────────────────────────────────

def telemetry_loop(outlet: pylsl.StreamOutlet, stop_event: threading.Event,
                   waveform, rate_hz: float) -> None:
    # Waveform phase is derived from wall-clock time (time.time()) so that this
    # process and mock_encoder produce the same waveform value at the same
    # instant even though they started at different moments.
    # monotonic time is used only for dt (numerical derivatives, immune to NTP).
    period = 1.0 / rate_hz
    t_last = time.monotonic()
    p_last = 0.0
    v_last = 0.0
    next_tick = t_last

    while not stop_event.is_set():
        now = time.monotonic()
        dt = now - t_last
        t_last = now

        p = waveform.position(time.time())
        v = (p - p_last) / dt if dt > 0 else 0.0
        a = (v - v_last) / dt if dt > 0 else 0.0
        p_last = p
        v_last = v

        outlet.push_sample([math.degrees(p), float(v), float(a)])

        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.monotonic()


# ── Event helpers ─────────────────────────────────────────────────────────────

def push_event(outlet: pylsl.StreamOutlet, payload: dict) -> None:
    outlet.push_sample([json.dumps(payload)])


# ── Main loop ─────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="Mock LSL robot controller")
    parser.add_argument(
        "--waveform",
        choices=("trapezoid", "sine", "step"),
        default=None,
        help="Override the waveform choice from params.yaml",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Path to params.yaml "
             "(default: auto-locate via ament share or repo).",
    )
    parser.add_argument(
        # "--pair-id",
        "--group-number",
        default=None,
        help="Group number N: LSL suffix (_N, none for 0). Overrides CV_GROUP_NUMBER env.",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)

    # Per-group LSL isolation: append "_<N>" to stream name + source_id so this
    # robot's streams resolve only on the matching verifier. CLI wins over env.
    # (Suffix behaviour UNCHANGED — only the identifier name is renamed.)
    # pair_id = args.pair_id if args.pair_id is not None else os.environ.get("CV_PAIR_ID", "")
    group_number = args.group_number if args.group_number is not None else os.environ.get("CV_GROUP_NUMBER", "")
    group_number = str(group_number).strip()
    suffix = f"_{group_number}" if group_number and group_number != "0" else ""
    if suffix:
        for _s in (cfg["lsl_params"]["actual_states_stream"],
                   cfg["lsl_params"]["event_trigger_stream"]):
            _s["name"]      = _s.get("name", "") + suffix
            _s["source_id"] = _s.get("source_id", "") + suffix
        print(f"[mock_robot_controller] LSL group suffix: {suffix}", file=sys.stderr)
    waveform_name = args.waveform or cfg.get("waveform_config", {}).get("type", "trapezoid")
    rate_hz = float(cfg.get("lsl_params", {}).get("actual_states_stream", {}).get("sampling_rate_hz", 100.0))
    waveform = build_waveform(waveform_name, cfg["waveform_config"])

    print("[mock_robot_controller] opening LSL outlets…", file=sys.stderr)

    tel_outlet = create_outlet(cfg["lsl_params"]["actual_states_stream"], pylsl.cf_float32)
    mrk_outlet = create_outlet(cfg["lsl_params"]["event_trigger_stream"], pylsl.cf_string)

    print("[mock_robot_controller] LSL ready: ActualStates, EventTrigger", file=sys.stderr)
    print(f"[mock_robot_controller] waveform={waveform_name}, "
          f"streaming telemetry at {rate_hz:.0f} Hz", file=sys.stderr)
    print(
        "[mock_robot_controller] commands:\n"
        "  ptp <value> <unit>                 — point_to_point\n"
        "  pp  <n> <seq> <dirs> <gripper>    — pick_place\n"
        "  perf <speed> <accel>              — performance\n"
        "  prec <init> <tar> <repeat> <unit> — precision\n"
        "  stop                              — STOP trigger\n"
        "  quit / q                          — exit",
        file=sys.stderr,
    )

    stop_event = threading.Event()
    tel_thread = threading.Thread(
        target=telemetry_loop,
        args=(tel_outlet, stop_event, waveform, rate_hz),
        daemon=True,
    )
    tel_thread.start()

    try:
        for line in sys.stdin:
            parts = line.strip().split()
            if not parts:
                continue
            cmd = parts[0].lower()

            if cmd == "ptp":
                if len(parts) != 3:
                    print("usage: ptp <value> <unit>", file=sys.stderr)
                    continue
                payload = {
                    "mode":   "Auto",
                    "action": "point_to_point",
                    "value":  float(parts[1]),
                    "unit":   parts[2],
                }
                push_event(mrk_outlet, payload)
                print(f"[mock_robot_controller] → {payload}", file=sys.stderr)

            elif cmd == "pp":
                if len(parts) != 5:
                    print("usage: pp <n> <seq> <dirs> <gripper>", file=sys.stderr)
                    continue
                n       = int(parts[1])
                seq     = [int(x) for x in parts[2].split(",")]
                dirs    = parts[3].split(",")
                gripper = parts[4].lower() in ("true", "1", "yes")
                payload = {
                    "mode":        "Auto",
                    "action":      "pick_place",
                    "num":         n,
                    "order_sequence": seq,
                    "directions":  dirs,
                    "use_gripper": gripper,
                }
                push_event(mrk_outlet, payload)
                print(f"[mock_robot_controller] → {payload}", file=sys.stderr)

            elif cmd == "perf":
                if len(parts) != 3:
                    print("usage: perf <speed> <accel>", file=sys.stderr)
                    continue
                payload = {
                    "mode":   "Test",
                    "action": "performance",
                    "speed":  float(parts[1]),
                    "accel":  float(parts[2]),
                }
                push_event(mrk_outlet, payload)
                print(f"[mock_robot_controller] → {payload}", file=sys.stderr)

            elif cmd == "prec":
                if len(parts) != 5:
                    print("usage: prec <init_pos> <tar_pos> <repeat> <unit>", file=sys.stderr)
                    continue
                payload = {
                    "mode":     "Test",
                    "action":   "precision",
                    "init_pos": int(parts[1]),
                    "tar_pos":  int(parts[2]),
                    "repeat":   int(parts[3]),
                    "unit":     parts[4],
                }
                push_event(mrk_outlet, payload)
                print(f"[mock_robot_controller] → {payload}", file=sys.stderr)

            elif cmd == "stop":
                payload = {"mode": "STOP", "action": "stop"}
                push_event(mrk_outlet, payload)
                print(f"[mock_robot_controller] → {payload}", file=sys.stderr)

            elif cmd in ("quit", "exit", "q"):
                break

            else:
                print(
                    f"[mock_robot_controller] unknown command: '{cmd}' "
                    "(use ptp|pp|perf|prec|stop|quit)",
                    file=sys.stderr,
                )
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        tel_thread.join(timeout=1.0)
        print("[mock_robot_controller] exit", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
