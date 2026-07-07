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

  "EventTrigger"  — 1 string channel, irregular rate, JSON payload:
      {"event": "START|STOP", "profile_id": "...",
       "label": "...", "expected_duration": 0.0}

Usage:
  python3 mock_robot_controller.py [--waveform trapezoid|sine|step]
  ros2 run claude_visualizer mock_robot_controller.py -- --waveform sine

  > start        # push EventTrigger START
  > stop         # push EventTrigger STOP
  > quit         # exit

The web_visualizer ROS node picks up both LSL streams and forwards them
to the JS frontend over WebSocket.
"""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import pylsl
# import yaml

from utils import create_outlet


# ── Config loading ────────────────────────────────────────────────────────────

# CONFIG_FILENAME = "params.yaml"


# def _find_config(explicit: str | None) -> Path:
#     """Locate params.yaml. Priority:
#         1. --config CLI flag
#         2. Installed share dir via ament_index_python (works under `ros2 run`)
#         3. Repo-relative fallback (works for direct `python3 mock_robot_controller.py`)
#     """
#     if explicit:
#         p = Path(explicit).expanduser().resolve()
#         if not p.is_file():
#             raise FileNotFoundError(f"--config not found: {p}")
#         return p

#     try:
#         from ament_index_python.packages import get_package_share_directory
#         share = Path(get_package_share_directory("claude_visualizer"))
#         candidate = share / "config" / CONFIG_FILENAME
#         if candidate.is_file():
#             return candidate
#     except Exception:
#         pass

#     here = Path(__file__).resolve().parent
#     candidate = here.parent / "config" / CONFIG_FILENAME
#     if candidate.is_file():
#         return candidate

#     raise FileNotFoundError(
#         f"Could not locate {CONFIG_FILENAME}. Pass --config <path> explicitly."
#     )


# def load_config(explicit_path: str | None) -> dict:
#     path = _find_config(explicit_path)
#     with open(path, "r") as f:
#         doc = yaml.safe_load(f) or {}
#     section = doc.get("mock_robot_controller", {}).get("ros__parameters", {})
#     if "lsl_params" not in section or "waveform_config" not in section:
#         raise KeyError(
#             f"Expected 'mock_robot_controller.ros__parameters.lsl_params' and "
#             f"'.waveform_config' in {path}"
#         )
#     print(f"[mock_robot_controller] config: {path}", file=sys.stderr)
#     return section


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

        outlet.push_sample([float(p), float(v), float(a)])

        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.monotonic()


# ── Marker helpers ────────────────────────────────────────────────────────────

def push_marker(outlet: pylsl.StreamOutlet, event: str, profile_id: str,
                label: str = "cli", expected_duration: float = 0.0) -> None:
    #The json (dict) is used because the data type of the marker msg is not a single type which requires more than one stream.
    #However, the single channel json has drawbacks on the high overhead in serialization and deserialization (using CPU) comparing to the multichannel stream.
    payload = {
        "event": event,
        "profile_id": profile_id,
        "label": label,
        "expected_duration": expected_duration,
    }   
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
    args = parser.parse_args()

    # cfg = load_config(args.config)
    # waveform_name = args.waveform or cfg.get("waveform_config", {}).get("type", "trapezoid")
    # rate_hz = float(cfg.get("lsl_params", {}).get("actual_states_stream", {}).get("sampling_rate_hz", 100.0))
    # waveform = build_waveform(waveform_name, cfg["waveform_config"])

    print("[mock_robot_controller] opening LSL outlets…", file=sys.stderr)

    cfg = {
        "lsl_params": {"event_trigger_stream": {
            "name": "EventTrigger",
            "type": "Trigger",
            "channel": ["json_payload"],
            "outlet_type": "IRREGULAR",
            "source_id": "mock_robot_controller-event_trigger",
        }},
    }

    # tel_outlet = create_outlet(cfg["lsl_params"]["actual_states_stream"], pylsl.cf_float32)
    mrk_outlet = create_outlet(cfg["lsl_params"]["event_trigger_stream"], pylsl.cf_string)

    print("[mock_robot_controller] LSL ready: ActualStates, EventTrigger", file=sys.stderr)
    # print(f"[mock_robot_controller] waveform={waveform_name}, "
    #       f"streaming telemetry at {rate_hz:.0f} Hz", file=sys.stderr)
    print("[mock_robot_controller] commands: start | stop | quit", file=sys.stderr)

    # stop_event = threading.Event()
    # tel_thread = threading.Thread(
    #     target=telemetry_loop,
    #     args=(tel_outlet, stop_event, waveform, rate_hz),
    #     daemon=True,
    # )
    # tel_thread.start()

    counter = 0
    active_profile_id = None

    try:
        for line in sys.stdin:
            cmd = line.strip().lower()
            if not cmd:
                continue

            if cmd == "start":
                counter += 1
                active_profile_id = f"cli_{counter:03d}"
                push_marker(mrk_outlet, "START", active_profile_id)
                print(f"[mock_robot_controller] START → {active_profile_id}", file=sys.stderr)

            elif cmd == "stop":
                if active_profile_id is None:
                    print("[mock_robot_controller] (no active profile)", file=sys.stderr)
                    continue
                push_marker(mrk_outlet, "STOP", active_profile_id)
                print(f"[mock_robot_controller] STOP  → {active_profile_id}", file=sys.stderr)
                active_profile_id = None

            elif cmd in ("quit", "exit", "q"):
                break

            else:
                print(f"[mock_robot_controller] unknown command: '{cmd}' "
                      f"(use start|stop|quit)", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        # stop_event.set()
        # tel_thread.join(timeout=1.0)
        print("[mock_robot_controller] exit", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
