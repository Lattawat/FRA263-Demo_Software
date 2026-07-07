# System Design — claude_visualizer

**Purpose:** Dense technical reference. Re-read this to orient in a new session before touching code.  
**Last updated:** 2026-06-26

---

## Development Phase Status

| Phase | Description | Status |
|---|---|---|
| 1 | ROS backbone: mock_encoder → encoder_reader → web_visualizer → browser | ✅ Complete |
| 1.5 | Hardware integration: real encoder + mock_robot_controller experiment CLI | ✅ Complete |
| 2 | UI integration: right panel activates on experiment; metrics/summary panels | ✅ Complete |
| 3 | Reference lines on uPlot charts (target_rad amber dashed lines) | ✅ Complete |
| 4 | Interactive criteria tab — view and edit pass/fail thresholds live from browser | ✅ Complete |
| 5 | UI polish: x-axis fix, plot zoom, Pos Sync/Zero server-side, unit toggle, scrollable panel | ✅ Complete |
| 6 | Multi-pair LAN scaling: single `pair_id` → ROS domain, LSL suffix, web ports | ✅ Complete |

**Current hardware state:** Real Teensy 4.1 encoder connected; mock_encoder commented out of bringup.launch.py. Use `mock_robot_controller.py` or `mock_ui.py` for sending experiment triggers.

---

## 1. Repository Layout

```
claude_visualizer_ws/
├── src/
│   ├── claude_visualizer/                  # Main ROS 2 package (ament_cmake)
│   │   ├── claude_visualizer/              # Installed Python module
│   │   │   ├── __init__.py
│   │   │   └── utils.py                    # create_outlet() — shared LSL outlet factory
│   │   ├── config/
│   │   │   ├── params.yaml                 # All node parameters (single source of truth)
│   │   │   └── criteria.yaml              # Per-robot evaluation criteria lookup table
│   │   ├── launch/
│   │   │   └── bringup.launch.py           # Launches encoder_reader + web_visualizer + experiment_evaluator
│   │   ├── mock_UI/
│   │   │   └── mock_ui.py                  # Dev GUI: encoder knob + command sender (Tkinter)
│   │   ├── scripts/
│   │   │   ├── mock_encoder.py             # Phase 1: synthetic /encoder_raw (commented out in launch)
│   │   │   ├── Kalman_filter.py            # encoder_reader node (constant-jerk KF)
│   │   │   ├── web_visualizer.py           # LSL↔ROS bridge + WS + HTTP server
│   │   │   ├── mock_robot_controller.py    # Non-ROS LSL publisher; experiment CLI
│   │   │   └── experiment_evaluator.py     # evaluates /estimated_states vs experiment config
│   │   ├── web/                            # Browser frontend (served on :http_port)
│   │   │   ├── index.html
│   │   │   ├── app.js
│   │   │   └── style.css
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── claude_visualizer_interface/        # Custom ROS 2 message + service definitions
│       ├── msg/
│       │   ├── EncoderRaw.msg
│       │   ├── EncoderState.msg
│       │   ├── ActualStates.msg
│       │   ├── EventTrigger.msg
│       │   └── ExperimentEval.msg          # live metrics + summary transport
│       ├── srv/
│       │   └── UpdateCriteria.srv          # Phase 4: live criteria editing
│       ├── CMakeLists.txt
│       └── package.xml
├── encoder_data_publisher/                 # Teensy 4.1 firmware (PlatformIO)
│   ├── src/main.cpp
│   ├── platformio.ini                      # Board: teensy41, framework: arduino
│   └── MICROROS_NOTES.md
├── pairs/                                  # Per-pair env files (Phase 6)
│   ├── pair<N>.env                         # source before running on any machine in the pair
│   ├── pair11.env                          # example: pair 11
│   └── pair12.env                          # example: pair 12
├── scripts/
│   └── make_pair_env.sh                    # Generator: ./scripts/make_pair_env.sh <N>
├── docs/
│   ├── System Design.md                   # ← this file (Claude agent reference)
│   └── System Overview.md                 # Human-readable narrative + reasoning
└── .venv/                                  # Python venv (--system-site-packages)
```

---

## 2. System Architecture

### Topology (single pair)

```
╔══════════════════════════════════════════════════════════════════════════╗
║  HARDWARE                                                                ║
║  Teensy 4.1 + quadrature encoder (4096 ticks/rev)                       ║
║  firmware: encoder_data_publisher/src/main.cpp                           ║
║  transport: micro-ROS over USB serial (115200 baud)                      ║
║  publishes: /encoder_raw  @ 100 Hz (10 ms timer)                         ║
╚══════════════════════════════╦═══════════════════════════════════════════╝
                                ║ (Phase 1: mock_encoder.py replaces this)
                    ┌───────────▼───────────┐
                    │   /encoder_raw        │  ROS 2 topic — EncoderRaw
                    └───────────┬───────────┘
                                │
                    ┌───────────▼───────────┐
                    │   encoder_reader      │  Kalman_filter.py
                    │   (constant-jerk KF)  │  state: [pos, vel, acc, jerk]
                    └─────────┬─────────────┘
                              │
                    ┌─────────▼─────────────┐
                    │  /estimated_states    │  ROS 2 topic — EncoderState
                    └──────┬────────────────┘
                           │                           ┌──────────────────────┐
                           │                           │ mock_robot_controller│ (non-ROS)
                           │                           │ OR mock_ui (tkinter) │
                           │                           │ LSL: ActualStates    │ pos in DEGREES
                           │                           │ LSL: EventTrigger    │ vel/acc in rad/s
                           │                           └──────────┬───────────┘
                           │                                      │ LSL (cross-machine transport)
               ┌───────────┼──────────────────────────────────────▼───────────┐
               │           │         web_visualizer (5 threads)                │
               │           ▼                                                   │
               │   sub /estimated_states → LSL EstimatedStates outlet         │
               │                         → WS broadcast estimated_states      │
               │                           (position adjusted by _est_zero_rad)│
               │   sub /actual_states    → WS broadcast actual_states         │
               │   sub /event_trigger    → WS broadcast event_trigger         │
               │   sub /eval_live        → WS broadcast eval_live             │
               │   sub /eval_summary     → WS broadcast eval_summary          │
               │   lsl-actual-states thread → converts deg→rad, applies       │
               │                             _actual_pos_offset_rad,          │
               │                             pub /actual_states               │
               │   lsl-event-trigger thread → pub /event_trigger              │
               │   ws-server (asyncio :9000+N) + http-server (:8000+N)        │
               │   GET /config.json → {"ws_port": N}                          │
               │   srv client /update_criteria → experiment_evaluator         │
               └─────────────────────────────┬────────────────────────────────┘
                                             │
                    ┌────────────────────────▼──────────────────────┐
                    │           experiment_evaluator                 │
                    │  sub /event_trigger → start/stop experiment   │
                    │  sub /estimated_states → evaluate per sample  │
                    │  pub /eval_live    (throttled @ 10 Hz)        │
                    │  pub /eval_summary (on experiment end)        │
                    │  srv server /update_criteria (Phase 4)        │
                    └────────────────────────────────────────────────┘
                                             │ WebSocket :9000+N
                             ┌───────────────▼───────────────┐
                             │       Browser (app.js)         │
                             │  fetches /config.json on load  │
                             │  uPlot charts @ 30 Hz          │
                             │  Criteria tab (Phase 4)        │
                             │  Pos Sync / Zero (Phase 5)     │
                             └───────────────────────────────┘
```

### Machine topology

**Verifier machine runs the ENTIRE ROS graph:** Teensy 4.1 + micro-ROS agent → `/encoder_raw` → `encoder_reader` → `/estimated_states` → `web_visualizer` + `experiment_evaluator`. Also runs: HTTP file server and WebSocket server.

**Robot machine** (when separate) runs only the robot controller, which emits **LSL** `ActualStates` + `EventTrigger`. No ROS on the robot machine in real deployment.

**Cross-machine transport = LSL only.** All ROS/DDS traffic is local to the verifier machine. `ROS_DOMAIN_ID` isolates each verifier's ROS graph from other verifiers on the LAN; it does NOT cross to the robot machine in a real deployment (harmless to source there if using a ROS-based mock).

**Cross-thread sync:** ROS callbacks → `asyncio.run_coroutine_threadsafe()` → WS loop thread.  
**Self-loop fanout:** `web_visualizer` publishes `/actual_states` + `/event_trigger` via LSL workers, then also subscribes to both — subscription callbacks do the WS broadcast.

---

## 3. ROS 2 Message & Service Interfaces

### `EncoderRaw`
```
std_msgs/Header header      # frame_id "encoder"
int32   ticks               # cumulative signed tick count
float64 raw_position        # ticks → rad (ticks / 4096 * 2π)
uint32  dt_us               # hardcoded 10000 (not measured — known issue)
```

### `EncoderState`
```
std_msgs/Header header
float64 position            # KF estimate [rad]
float64 velocity            # KF estimate [rad/s]
float64 acceleration        # KF estimate [rad/s²]
float64 pos_variance        # posterior P[0,0]
float64 vel_variance        # posterior P[1,1]
float64 acc_variance        # posterior P[2,2]
int32   raw_ticks
```

### `ActualStates`
```
std_msgs/Header header      # frame_id "robot_controller"
float64 actual_position     # [rad] — converted from degrees by web_visualizer; offset applied
float64 actual_velocity     # [rad/s]
float64 actual_acceleration # [rad/s²]
```

### `EventTrigger`
```
std_msgs/Header header      # frame_id "robot_controller"; stamp set by bridge on receipt
string  event               # Full JSON payload string. ALWAYS contains key "action".
                            # "mode" key indicates Auto/Test/STOP.
                            # All other keys are experiment-specific.
```
**IMPORTANT:** `msg.event` is the complete JSON string — not a short type tag.
Parse with `json.loads(msg.event)`. Dispatch on `payload["action"]`.
Stop condition: `payload["mode"] == "STOP"` or `payload["action"] == "stop"`.
`pick_place` payload uses key `"order_sequence"` (not `"sequence"`).

### `ExperimentEval`
```
std_msgs/Header header
string action    # experiment type: point_to_point | pick_place | performance | precision
string data      # JSON string — metrics dict (eval_live) or summary dict (eval_summary)
```
Used on both `/eval_live` (throttled live metrics) and `/eval_summary` (post-experiment).

### `UpdateCriteria.srv` (Phase 4)
```
string criteria_json        # JSON: { "max_overshoot_pct": 15.0 } — partial updates allowed
---
bool   success
string message              # "Updated: max_overshoot_pct" or error reason
string current_criteria_json  # full criteria dict after update (or current state on failure)
```
Empty `criteria_json` (`"{}"`) is a valid read-only snapshot request — returns current criteria without modifying anything.

---

## 4. ROS Topics & Services

### Topics

| Topic               | Type              | Producer                         | Consumer(s)                          |
|---------------------|-------------------|----------------------------------|--------------------------------------|
| `/encoder_raw`      | `EncoderRaw`      | Teensy firmware / mock_encoder / **mock_ui** | `encoder_reader`                     |
| `/estimated_states` | `EncoderState`    | `encoder_reader`                 | `web_visualizer`, `experiment_evaluator` |
| `/actual_states`    | `ActualStates`    | `web_visualizer` (LSL inlet)     | `web_visualizer` (self-loop → WS)    |
| `/event_trigger`    | `EventTrigger`    | `web_visualizer` (LSL inlet) / **mock_ui** (direct ROS2) | `web_visualizer` (self-loop → WS), `experiment_evaluator` |
| `/eval_live`        | `ExperimentEval`  | `experiment_evaluator`           | `web_visualizer` (→ WS broadcast)    |
| `/eval_summary`     | `ExperimentEval`  | `experiment_evaluator`           | `web_visualizer` (→ WS broadcast)    |

QoS on all topics: `RELIABLE`, `KEEP_LAST`, depth=10.

### Services

| Service            | Type              | Server                  | Client             |
|--------------------|-------------------|-------------------------|--------------------|
| `/update_criteria` | `UpdateCriteria`  | `experiment_evaluator`  | `web_visualizer`   |

---

## 5. LSL Streams

| Stream name       | Direction | Format    | Channels | Rate      | Producer                  |
|-------------------|-----------|-----------|----------|-----------|---------------------------|
| `EstimatedStates` | outlet    | float32   | 3        | 500 Hz    | `web_visualizer`          |
| `ActualStates`    | inlet     | float32   | 3        | 500 Hz    | `mock_robot_controller` / `mock_ui` |
| `EventTrigger`    | inlet     | string    | 1        | IRREGULAR | `mock_robot_controller` / `mock_ui` |

**Unit conventions for `ActualStates`:** position channel is in **degrees**; velocity in rad/s; acceleration in rad/s².  
`web_visualizer._actual_states_worker` converts position with `* _DEG_TO_RAD` and subtracts `_actual_pos_offset_rad` before publishing to `/actual_states`.

**Multi-pair suffix (Phase 6):** When `pair_id = N ≠ 0`, **stream name AND source_id** both get suffix `_N` (e.g. `ActualStates_11`, `EventTrigger_11`). This suffix must be consistent across ALL three producers for a given pair:
- verifier: `web_visualizer` `session` launch param → `_suf()` applied in code
- robot mock: `CV_PAIR_ID` env var / `--pair-id` CLI arg → suffix applied before `create_outlet()`
- `_resolve_stream` grabs `streams[0]` — the unique name+source_id suffix ensures it finds the correct pair's stream

`utils.py:create_outlet()` is the shared factory for all LSL outlets.

---

## 6. WebSocket Protocol

**Port:** `9000 + pair_id` (default `9090` when `pair_id = 0`)  
**HTTP port:** `8000 + pair_id` (default `8000` when `pair_id = 0`)  
**Format:** newline-delimited JSON text frames

**Port discovery (Phase 6):** Browser fetches `GET /config.json` from the HTTP server on page load. Response: `{"ws_port": N}`. `app.js` uses this port to open the WebSocket instead of a hardcoded value. This decouples the browser from knowing the port at build time.

### Server → Browser

| `topic`            | `data` keys |
|--------------------|-------------|
| `estimated_states` | `stamp`, `position` (adjusted by `_est_zero_rad`), `velocity`, `acceleration`, `pos_variance`, `vel_variance`, `acc_variance`, `raw_ticks` |
| `actual_states`    | `stamp`, `actual_position` (deg→rad converted + offset applied), `actual_velocity`, `actual_acceleration` |
| `event_trigger`    | `stamp` + **all keys spread from `msg.event` JSON** (action, mode, + experiment-specific) |
| `eval_live`        | `stamp`, `action` + **all keys spread from `ExperimentEval.data` JSON** |
| `eval_summary`     | `stamp`, `action` + **all keys spread from `ExperimentEval.data` JSON** |
| `time_sync`        | `ref_stamp` |
| `criteria_snapshot`| full criteria dict (all keys + values); sent once on WS connect |
| `criteria_ack`     | `success`, `message`, `criteria` (full dict, only on success) |
| `zero_ack`         | `est_zero_rad` — raw Kalman position at time Zero was pressed; browser uses to offset target line |

**Snapshot on connect:** last cached message for each topic + `criteria_snapshot` sent immediately.

### Browser → Server

| `command`          | `data` keys | Effect |
|--------------------|-------------|--------|
| `time_sync`        | —           | Server sets `ref_stamp = now`, broadcasts `time_sync` topic |
| `stop_experiment`  | —           | Server publishes stop `EventTrigger` |
| `criteria_update`  | `{ key: value }` | Server calls `/update_criteria` service, broadcasts `criteria_ack` |
| `pos_sync`         | `delta_rad` | Server adds `delta_rad` to `_actual_pos_offset_rad`; future actual positions shift |
| `zero_set`         | `act_rad`   | Server sets `_est_zero_rad = last_est_pos`, adds `act_rad` to `_actual_pos_offset_rad`, broadcasts `zero_ack` |

### HTTP Endpoints

| Path | Method | Response | Notes |
|------|--------|----------|-------|
| `/config.json` | GET | `{"ws_port": N}` | Served by `_SilentHTTPHandler.do_GET`; injected port from `self.server.ws_port` |
| `/*` | GET | static file | Falls through to `http.server.SimpleHTTPRequestHandler.do_GET` |

---

## 7. Node Reference

### `encoder_reader` (`scripts/Kalman_filter.py`)
- State vector: `[position, velocity, acceleration, jerk]`
- Transition: constant-jerk, variable dt from `EncoderRaw.dt_us`
- Measurement: position only (`H = [1, 0, 0, 0]`)

### `web_visualizer` (`scripts/web_visualizer.py`)
- **5 threads:** ROS spin, `ws-server` (asyncio), `http-server`, `lsl-actual-states`, `lsl-event-trigger`
- **Subscribes:** `/estimated_states`, `/actual_states`, `/event_trigger`, `/eval_live`, `/eval_summary`
- **Publishes:** `/actual_states`, `/event_trigger`
- **Service client:** `/update_criteria` (async call inside WS handler)
- **LSL outlet:** `EstimatedStates` (3 × float32) — name + source_id suffixed by `_suf()` when session set
- **LSL inlets:** `ActualStates` (float), `EventTrigger` (string/JSON) — resolved by suffixed name
- **Serializers:** `estimated_states_to_json`, `actual_states_to_json`, `event_trigger_to_json`, `_eval_to_json`

**Parameters (Phase 6 additions):**
- `session` (string, default `""`) — suffix token; `_suf = (lambda n: f"{n}_{session}") if session else (lambda n: n)` — applied to all LSL stream names + source_ids
- `ws_port` — set by launch to `9000+N` (or 9090)
- `http_port` — set by launch to `8000+N` (or 8000)
- `ws_host` — default `"0.0.0.0"` (listens on all interfaces)

**`/config.json` wiring:**
```python
# in _run_http_server:
self._http_server.ws_port = self._ws_port   # inject port onto server object

# in _SilentHTTPHandler.do_GET:
if self.path == "/config.json":
    body = json.dumps({"ws_port": self.server.ws_port}).encode()
    ...
```

**Position offset state (Phase 5):**
```python
self._actual_pos_offset_rad = 0.0   # cumulative; subtracted from every incoming LSL position
self._est_zero_rad          = 0.0   # subtracted from WS broadcast of estimated position
self._last_est_pos_rad      = 0.0   # updated each _estimated_states_cb; used by zero_set
```

**Key callbacks:**
- `_estimated_states_cb`: pushes to LSL outlet; subtracts `_est_zero_rad` from position before WS broadcast; updates `_last_est_pos_rad`
- `_actual_states_worker`: converts position deg→rad, subtracts `_actual_pos_offset_rad`, publishes to `/actual_states`
- `_handle_command`: dispatches `time_sync`, `stop_experiment`, `criteria_update`, `pos_sync`, `zero_set`
- `_get_criteria_snapshot`: called on each new WS connection; sends `{}` to `/update_criteria` to read current state
- `_broadcast` / `_broadcast_async`: thread-safe WS fanout; caches last message per topic

### `experiment_evaluator` (`scripts/experiment_evaluator.py`)
- **Parameters:** `pair_id` (default: `"0"`), `criteria_file_path` — `pair_id` selects the criteria row (replaces the old `robot_id`)
- **Subscribes:** `/event_trigger`, `/estimated_states`
- **Publishes:** `/eval_live` (throttled every 0.1 s), `/eval_summary` (on finish)
- **Service server:** `/update_criteria` — validates key names and non-negative values; updates `self._criteria` in-place; changes are in-memory only (lost on restart)
- **Criteria:** loaded from `criteria.yaml` at startup into `self._criteria` dict
- **Constants:** `SETTLING_THRESHOLD_rad = 0.01`, `SETTLING_WINDOW_s = 0.5`, `LIVE_PUB_INTERVAL_s = 0.1`
- **Settling detection:** time-based — requires continuous 0.5 s inside band. `cont_band_entry_s` resets on any band exit; `first_band_entry_s` records first entry (never resets) for `settling_time_s` reporting.
- **Variable naming:** all variables carry unit suffixes (`_rad`, `_rad_s`, `_rad_s2`, `_s`, `_pct`, `_count`)
- **ptp summary:** reports `final_error_rad` (error at finish) instead of `max_error`
- **performance:** `cmd_speed_rad_s` and `cmd_accel_rad_s2` taken from payload first, then criteria as fallback
- **`pick_place` payload key:** reads `payload["order_sequence"]` — senders must use this key (not `"sequence"`)
- See §8 for full experiment logic.

### `mock_ui` (`mock_UI/mock_ui.py`)
- **Tkinter GUI** combining mock encoder knobs + LSL/ROS2 event sender. Developer tool for testing the full pipeline without real hardware or a robot controller.
- **ROS2 publishers:** `/encoder_raw` (EncoderRaw), `/event_trigger` (EventTrigger)
- **LSL outlets:** `ActualStates` (3 × float32, 50 Hz), `EventTrigger` (1 × string, IRREGULAR)
- **Two knobs:** Encoder knob → drives `/encoder_raw`; ActualStates knob → drives LSL `ActualStates`
- **Sync mode:** when ON, the ActualStates knob follows the encoder knob's *rate of change* (not absolute position)
- **Fine / Rough mode:** toggles drag sensitivity between 0.1 deg/px (fine) and 1.0 deg/px (rough)
- **Control direction:** drag up = anticlockwise (negative), drag down = clockwise (positive)
- **Readout:** both knobs display rad and deg simultaneously
- **Command entry:** same ptp/pp/perf/prec/stop command syntax as `mock_robot_controller` — publishes to both LSL EventTrigger and ROS2 `/event_trigger`
- **`TICKS_PER_REV`:** 4096 (matches Teensy firmware)
- **Phase 6:** reads `CV_PAIR_ID` env / `--pair-id` CLI; `_pair_suffix()` helper appends `_N` to LSL name + source_id before `create_outlet()`

**`_pair_suffix()` pattern (shared by mock_ui and mock_robot_controller):**
```python
def _pair_suffix(pair_id=None) -> str:
    pid = str(pair_id if pair_id is not None else os.environ.get("CV_PAIR_ID", "")).strip()
    return f"_{pid}" if pid and pid != "0" else ""
```

### `mock_robot_controller` (`scripts/mock_robot_controller.py`)
- **NOT a ROS node.** Pure Python + pylsl. Run separately.
- **LSL outlets:** `ActualStates` (3 × float32, REGULAR), `EventTrigger` (1 × string, IRREGULAR)
- **Telemetry:** continuous waveform (trapezoid/sine/step) in background thread
- **Unit output:** position sent in **degrees** (`math.degrees(p)`); velocity and acceleration in rad/s and rad/s² (numerical derivatives of raw radian waveform — not converted)
- **Phase 6:** reads `CV_PAIR_ID` env / `--pair-id` CLI; appends suffix to YAML-loaded `name` and `source_id` before `create_outlet()`
- **CLI commands:**

| Command | Parameters | Example | JSON sent |
|---|---|---|---|
| `ptp <value> <unit>` | `value`: increment in `unit`; `unit`: `index` \| `degree` \| `rad` | `ptp 5 index` | `{mode:"Auto", action:"point_to_point", value:5.0, unit:"index"}` |
| `pp <n> <seq> <dirs> <gripper>` | `n`: waypoint count; `seq`: comma-sep integers [index]; `dirs`: comma-sep `CW`\|`CCW`; `gripper`: `true`\|`false` | `pp 3 0,36,72 CW,CCW,CW true` | `{mode:"Auto", action:"pick_place", num:3, order_sequence:[0,36,72], directions:["CW","CCW","CW"], use_gripper:true}` |
| `perf <speed> <accel>` | `speed` [rad/s]; `accel` [rad/s²] | `perf 1.0 2.0` | `{mode:"Test", action:"performance", speed:1.0, accel:2.0}` |
| `prec <init> <tar> <repeat> <unit>` | `init`, `tar`: positions in `unit`; `repeat`: trial count (integer); `unit`: `index` \| `degree` | `prec 0 36 10 index` | `{mode:"Test", action:"precision", init_pos:0, tar_pos:36, repeat:10, unit:"index"}` |
| `stop` | — | — | `{mode:"STOP", action:"stop"}` |
| `quit` / `q` | — | — | exit |

---

## 8. Experiment Evaluation Subsystem

### Unit mapping (N_HOLES = 72)
```
index → rad:  rad = index × 5 × π/180  ≈ index × 0.08727
degree → rad: rad = degree × π/180
_to_rad(value, unit) helper: "index" | "degree" | anything else (treated as rad)
```

### Settling detection algorithm (shared by ptp, pp, precision)
```
band = max(SETTLING_THRESHOLD, criteria["settling_band_pct"] / 100.0 × |travel|)
  • band_entry_time recorded on first sample where error < band
  • settled_count increments while error < band; resets to 0 otherwise
  • SETTLED when settled_count ≥ SETTLING_WINDOW (50 samples)
  • settling_time_s = time_at_settled - band_entry_time
```

### Experiment payloads → evaluator behaviour

#### `point_to_point`
```json
{"mode":"Auto","action":"point_to_point","value":5,"unit":"index"}
```
- `target_rad = _start_pos + _to_rad(value, unit)` — **increment from start position**
- Terminates: **auto** when settled at target, or explicit stop
- eval_live: `target_rad`, `current_pos`, `current_error`, `elapsed_s`
- eval_summary: `target_rad`, `final_error_rad`, `overshoot_pct`, `settling_time_s`, `pass_overshoot`, `pass_settling`

#### `pick_place`
```json
{"mode":"Auto","action":"pick_place","num":3,"order_sequence":[0,36,72],"directions":["CW","CCW","CW"],"use_gripper":true}
```
- **Key name:** `"order_sequence"` (NOT `"sequence"`)
- Waypoints: `order_sequence[idx] × RAD_PER_INDEX` (absolute, index units)
- Terminates: **auto** after settling at waypoint `num-1`, or explicit stop
- eval_summary: `total_waypoints`, `avg_error_rad`, `pass_avg_error`, `passed`, `failed`, `details[]`

#### `performance`
```json
{"mode":"Test","action":"performance","speed":1.0,"accel":2.0}
```
- Terminates: explicit stop only
- eval_summary: `commanded_speed_rad_s`, `peak_speed_rad_s`, `commanded_accel_rad_s2`, `peak_accel_rad_s2`, `pass_speed`, `pass_accel`

#### `precision`
```json
{"mode":"Test","action":"precision","init_pos":0,"tar_pos":36,"repeat":10,"unit":"index"}
```
- Two-state machine: alternates settling at `tar_rad` and `init_rad`; records position at each tar settle
- Terminates: **auto** after `repeat` trials at target, or explicit stop
- eval_summary: `target_rad`, `num_trials`, `mean_error_rad`, `std_error_rad`, `max_error_rad`, `pass_error`

### Criteria (`config/criteria.yaml`)
Keyed by `pair_id` (the launch arg / `CV_PAIR_ID`). Rows are pair numbers (e.g. `11`, `12`) plus a `default` fallback used for `pair_id` 0 or any unlisted pair. The loader normalizes keys to strings, so `11:` (int) and `"11":` (str) both match.

| Key | Used by | Meaning |
|---|---|---|
| `min_speed` | performance | peak_vel must ≥ this |
| `min_acceleration` | performance | peak_acc must ≥ this |
| `max_avg_error_rad` | pick_place, precision | error threshold |
| `max_overshoot_pct` | point_to_point, pick_place | % of travel |
| `max_settling_time_s` | point_to_point, pick_place | from band entry |
| `settling_band_pct` | all except performance | % of travel for band width |

**Live editing (Phase 4):** Criteria can be modified at runtime via the browser CRITERIA tab without restarting nodes. Changes are in-memory only — `criteria.yaml` is not written.

---

## 9. Configuration Files

### `config/params.yaml`
Single source for all ROS node params. Key entries:

| Node | Parameter | Default | Notes |
|---|---|---|---|
| `encoder_reader` | `kf_q_position` | 0.001 | Process noise — tune for lag vs noise |
| `encoder_reader` | `kf_r_position` | 0.5 | Measurement noise — higher = smoother |
| `web_visualizer` | `ws_port` | 9090 | WebSocket port; launch overrides to `9000+N` |
| `web_visualizer` | `http_port` | 8000 | HTTP file server port; launch overrides to `8000+N` |
| `web_visualizer` | `ws_host` | `"0.0.0.0"` | WS listen address |
| `web_visualizer` | `session` | `""` | LSL stream name suffix token; launch sets to `str(N)` |
| `web_visualizer` | `lsl_params.resolve_timeout_s` | 5.0 | LSL stream lookup timeout |
| `mock_robot_controller` | `lsl_params.actual_states_stream.sampling_rate_hz` | 500.0 | |
| `mock_robot_controller` | `waveform_config.type` | `"trapezoid"` | `sine` \| `trapezoid` \| `step` |
| `mock_robot_controller` | `waveform_config.trap_max_velocity` | π | [rad/s] — waveform is computed in rad |
| `mock_robot_controller` | `waveform_config.trap_acceleration` | π/2 | [rad/s²] |

### `config/criteria.yaml`
Per-pair evaluation criteria. Edit directly to change pass/fail thresholds.
Launch with `pair_id:=11` (or any pair number / key in the file) to select a row; `pair_id` 0 or an unlisted pair uses `default`.

---

## 10. Browser Frontend (`web/`)

| File | Role |
|---|---|
| `index.html` | Header (Time Sync, Pos Sync, Zero buttons); two-panel main (left: live, right: profile/zoom/criteria tabs); footer (Clear, Pause, Crop) |
| `style.css` | CSS grid (2 col); flex left panel (3 equal plots); scrollable right panel tab-content |
| `app.js` | WS client, data buffers, uPlot, redraw loop, crop/zoom, unit toggle, pos sync, zero |

### Layout
- **Left panel:** fixed height, 3 plots fill equally, no scroll
- **Right panel:** flex column; tab-bar is `flex-shrink: 0` (always visible); `.tab-content` is `overflow-y: auto; flex: 1` (scrolls independently); zoom-header is `position: sticky; top: 0` within the scroll area
- **Right panel plots:** 260 px each (taller than left panel for detail inspection)

### Data buffers
- `live`: rolling 10 s, max 5000 samples
- `profile`: START → STOP (or settled)
- `zoom`: drag-crop snapshot from left panel

### Redraw loop
`setInterval(redraw, 1000/30)` — 30 Hz, decoupled from WS rate. Updates live plots, profile plots, and zoom plots every tick using current `pScale` (rad or deg). This ensures unit-toggle changes are reflected in all panels without re-cropping.

### Phase 6: WS port discovery
`app.js` calls `resolveWsUrl()` on page load before `connect()`:
```js
async function resolveWsUrl() {
    try {
        const resp = await fetch("/config.json", { cache: "no-store" });
        const cfg  = await resp.json();
        if (cfg && cfg.ws_port) {
            WS_URL = `ws://${location.hostname || "localhost"}:${cfg.ws_port}`;
        }
    } catch (_) { /* keep fallback WS_URL = ws://...:9090 */ }
    document.getElementById("ws-url").textContent = WS_URL;
}
resolveWsUrl().then(connect);
```
`location.hostname` automatically resolves to the verifier machine's host (same host that served the page), so the WS connects to the right machine without hardcoding an IP.

### State object (key fields)
```javascript
state = {
    paused, profileActive, profileStartTs,
    latestActual,           // most recent actual_states msg
    timeRef,                // epoch-second reference (auto-set on first sample; reset by Time Sync)
    estZeroRad,             // server's _est_zero_rad at last zero_set; offsets target line + abs eval fields
    cropMode,
    rightTab,
    trackEvents,            // Auto button: whether to auto-start profile on event_trigger
    targetRef,              // current target_rad from eval_live [rad, server frame]
    livePosUnit,            // "rad" | "deg" — left panel
    profilePosUnit,         // "rad" | "deg" — right panel (shared by profile + zoom)
}
```

### Position pipeline (browser side)
All positions arrive from server already offset (`_est_zero_rad` / `_actual_pos_offset_rad` applied). Browser only applies unit scale:
```
display_value = position_rad * pScale    // pScale = 1.0 (rad) or 180/π (deg)
```
`applyPos(arr, 0, scale)` — offset is always 0 (server handles it).

### Pos Sync and Zero buttons
- **Pos Sync**: sends `{ command: "pos_sync", data: { delta_rad: act - est } }` — server shifts all future actual positions so they align with estimated at press time. Both browser and evaluator see corrected values.
- **Zero**: sends `{ command: "zero_set", data: { act_rad: act } }` — server zeroes both estimated (broadcast) and actual (LSL offset). Server replies with `zero_ack { est_zero_rad }`. Browser stores `state.estZeroRad` to offset target reference line and `target_rad`/`current_pos_rad` eval display fields.

### Unit toggle
Two independent buttons:
- `btn-unit-live` → `state.livePosUnit` (left panel only)
- `btn-unit-profile` → `state.profilePosUnit` (right panel: profile + zoom)

`posScale(which)` returns the multiplier; `updatePosLabels()` updates `[rad]`/`[deg]` labels.

### Plot zoom (right panel)
- **Drag**: x-only (`drag: { x: true, y: false }`) — y auto-scales to all data in visible x-range
- **Double-click**: resets x-scale to auto-fit (all data visible)
- Zoom data snapshot is re-rendered every redraw tick — unit toggle updates zoom plots automatically

### WS message routing
| Message | Handler |
|---|---|
| `estimated_states` | `pushLive(msg)` — auto-sets `state.timeRef` on first sample |
| `actual_states` | `state.latestActual = msg.data` |
| `event_trigger` | `onEventTrigger` — `"stop"` → `stopExperiment()`; else → `startExperiment()` |
| `eval_live` | `onEvalLive` — sets `state.targetRef`, renders live metrics |
| `eval_summary` | `onEvalSummary` — calls `stopProfile()`, renders summary, switches to PROFILE tab |
| `criteria_snapshot` | `onCriteriaSnapshot` — renders editable CRITERIA tab rows |
| `criteria_ack` | `onCriteriaAck` — green/red flash on edited field |
| `zero_ack` | `state.estZeroRad = msg.data.est_zero_rad` |
| `time_sync` | `state.timeRef = msg.data.ref_stamp` |

### Eval display — relative positions
`POSITION_ABS_FIELDS = { "target_rad", "current_pos_rad" }` — in `appendRow()`, these fields are displayed as `value - state.estZeroRad` so they show displacement from zero, matching what the operator sees on the plots.

### uPlot notes
- NEVER set `cursor.points.show: true` — causes crash. Use default or a function.
- Left panel plots: `cursor.drag: { x: false, y: false }` (crop overlay handles selection)
- Right panel plots: `cursor.drag: { x: true, y: false }` (x-zoom only, y auto-fits)
- `snapCursorToEst` cursor.move snaps the crosshair to the estimated series y-value

---

## 11. Firmware (Teensy 4.1)

- **Location:** `encoder_data_publisher/src/main.cpp`
- **Transport:** USB serial, 115200 baud, micro-ROS
- **Encoder pins:** CH_A=7, CH_B=6, 4096 ticks/rev
- **Timer:** 10 ms → publishes `EncoderRaw`
- **Timestamp:** `rmw_uros_epoch_nanos()` (synced via `rmw_uros_sync_session(1000)`)
- **Known:** `dt_us` hardcoded to 10000 — not measured from actual timer elapsed

---

## 12. Build & Run

### Standard (single pair)

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

# Build (interface first if .srv changed, then main package)
colcon build --packages-select claude_visualizer_interface claude_visualizer
source install/setup.bash

# Launch pipeline (real encoder)
ros2 launch claude_visualizer bringup.launch.py
# With a specific pair's criteria (also sets ports/LSL suffix):
ros2 launch claude_visualizer bringup.launch.py pair_id:=11

# Mock controller (separate terminal — not a ROS node, no sourcing needed)
ros2 run claude_visualizer mock_robot_controller
# Then at prompt: ptp 5 index | pp 3 0,36,72 CW,CCW,CW true | perf 1.0 2.0 | prec 0 36 10 index | stop | quit

# Browser (hard-refresh after any web/ file change — no build needed for CSS/JS/HTML)
# http://localhost:8000
```

### Multi-pair (Phase 6)

```bash
# Generate env file for pair N (run once per pair number)
./scripts/make_pair_env.sh 11    # creates pairs/pair11.env

# --- On VERIFIER machine (runs full ROS graph + web server) ---
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source pairs/pair11.env          # sets ROS_DOMAIN_ID=11, CV_PAIR_ID=11
source install/setup.bash
ros2 launch claude_visualizer bringup.launch.py
# WS on :9011, HTTP on :8011, LSL stream names suffixed _11

# --- On ROBOT machine (LSL source only) ---
source pairs/pair11.env          # sets CV_PAIR_ID=11 (ROS_DOMAIN_ID harmless if no ROS)
python3 src/claude_visualizer/scripts/mock_robot_controller.py
# OR if the mock_ui is on a ROS-enabled machine:
# source /opt/ros/jazzy/setup.bash && source install/setup.bash
# python3 src/claude_visualizer/mock_UI/mock_ui.py

# Browser: http://<verifier-ip>:8011   (app.js fetches /config.json to get ws_port=9011)

# --- Pair stream name verification ---
python3 -c "
pair=11
print('ws_port:', 9000+pair, 'http_port:', 8000+pair)
print('LSL names: ActualStates_11, EventTrigger_11, EstimatedStates_11')
"
```

### Port derivation table

| pair_id | ROS_DOMAIN_ID | ws_port | http_port | LSL suffix |
|---------|--------------|---------|-----------|------------|
| 0 (unset) | 0 | 9090 | 8000 | (none) |
| 1 | 1 | 9001 | 8001 | `_1` |
| 11 | 11 | 9011 | 8011 | `_11` |
| 12 | 12 | 9012 | 8012 | `_12` |
| 101 | 101 | 9101 | 8101 | `_101` |

`ROS_DOMAIN_ID` safe range: **0–101** (ports 7400–7680; outside this range DDS has port collisions).

**Launch args:**
- `pair_id` (default: from `CV_PAIR_ID` env, else `"0"`) — single setup identifier: derives ports + LSL suffix (Phase 6) **and** selects the criteria row in `criteria.yaml` (replaces the old `robot_id`)
- `waveform` (default: `"trapezoid"`) — for mock_encoder if re-enabled

**Build notes:**
- `colcon build` is required for any `.py` script change (scripts are copied to install, not symlinked — `--symlink-install` conflicts with micro_ros_package)
- Web files (`index.html`, `app.js`, `style.css`) are served from the installed share dir — also require `colcon build`
- `claude_visualizer_interface` must be rebuilt whenever `.srv` or `.msg` files change

---

## 13. Multi-Pair LAN Scaling (Phase 6)

### Problem
Running N robot+verifier pairs on one LAN causes three independent collision risks:
1. **ROS 2 / DDS:** All pairs on domain 0 receive each other's `/estimated_states`, `/event_trigger`, etc. — evaluator gets wrong data.
2. **LSL:** `_resolve_stream(name)` grabs `streams[0]` from any stream with that name on the LAN — verifier latches onto the wrong robot's telemetry.
3. **Web ports:** Two `web_visualizer` nodes on the same host cannot both bind to `:9090` / `:8000` — second launch fails.

### Solution: single `pair_id = N` → three-layer derivation

```python
# bringup.launch.py — _derive_pair()
def _derive_pair(pair_id: int):
    if pair_id:
        return 9000 + pair_id, 8000 + pair_id, str(pair_id)
    return 9090, 8000, ""
```

| Layer | Derivation | Scope |
|---|---|---|
| ROS 2 | `ROS_DOMAIN_ID = N` (exported before launch) | verifier machine only |
| LSL | stream name + source_id get suffix `_N` | both machines |
| Web | `ws_port = 9000+N`, `http_port = 8000+N` | verifier machine |

### Env file (`pairs/pair<N>.env`)
```bash
export ROS_DOMAIN_ID=11   # verifier: isolates local ROS graph; harmless on pure-LSL robot box
export CV_PAIR_ID=11      # both machines: LSL suffix _11 — the cross-machine isolator
```
Generated by `./scripts/make_pair_env.sh <N>`. **Source before any `ros2` command or mock script.**

### Component wiring

| Component | How it reads pair_id | What it does with it |
|---|---|---|
| `bringup.launch.py` | `CV_PAIR_ID` env → `pair_id` launch arg | Computes ws/http ports + session; passes as param overrides to `web_visualizer` |
| `web_visualizer.py` | `session` ROS param (set by launch) | `_suf()` appends `_{session}` to all LSL names + source_ids |
| `mock_robot_controller.py` | `CV_PAIR_ID` env / `--pair-id` CLI | Mutates YAML-loaded stream cfg before `create_outlet()` |
| `mock_ui.py` | `CV_PAIR_ID` env / `--pair-id` CLI | `_pair_suffix()` → mutates ACTUAL_STATES_CFG / EVENT_TRIGGER_CFG |
| `app.js` | fetches `/config.json` → `ws_port` | Opens WS to `ws://${location.hostname}:${ws_port}` |

### Backward compatibility
`pair_id = 0` or unset → no suffix, ports `9090`/`8000`, `ROS_DOMAIN_ID=0`. All existing single-pair workflows unchanged.

### Verification (expected stream names)
```
pair=0   ws=9090 http=8000 | ActualStates / EventTrigger / EstimatedStates
pair=11  ws=9011 http=8011 | ActualStates_11 / EventTrigger_11 / EstimatedStates_11
pair=101 ws=9101 http=8101 | ActualStates_101 / EventTrigger_101 / EstimatedStates_101
```

---

## 14. Known Issues

| Location | Issue | Severity |
|---|---|---|
| `encoder_data_publisher/src/main.cpp` | `dt_us` hardcoded to 10000 — not measured from actual timer elapsed | Known |
| `web_visualizer.py` `_actual_pos_offset_rad` | Cumulative offset; not reset on WS reconnect or node restart — operator must press Pos Sync again after restart | Known |
| `criteria_update` (Phase 4) | Changes are in-memory only — lost on `experiment_evaluator` restart | By design |
| `web_visualizer.py` `_resolve_stream` | If LSL stream not found within `resolve_timeout_s`, node prints error but continues — no retry loop | Known |

---

## 15. Dependencies

### Python (pip, venv)
| Package | Used by |
|---|---|
| `pylsl` | `web_visualizer.py`, `mock_robot_controller.py`, `mock_ui.py` |
| `websockets` | `web_visualizer.py` |
| `numpy` | `mock_encoder.py`, `Kalman_filter.py` |
| `PyYAML` | `mock_robot_controller.py`, `experiment_evaluator.py` |
| `catkin-pkg` | ROS 2 build tooling (required in venv) |
| `empy` | ROS 2 build tooling (required in venv) |
| `lark` | ROS 2 build tooling (required in venv) |

### ROS 2
`rclcpp`, `rclpy`, `std_msgs`, `builtin_interfaces`, `rosidl_default_generators`

### Frontend (CDN)
uPlot 1.6.30 via jsdelivr

### System
`liblsl`: `sudo apt install liblsl-dev`
