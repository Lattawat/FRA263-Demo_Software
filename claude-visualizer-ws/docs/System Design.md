# System Design — claude_visualizer

**Purpose:** Dense technical reference. Re-read this to orient in a new session before touching code.  
**Last updated:** 2026-07-08

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
| 6 | Multi-group isolation: single `group_number` → ROS **namespace** `/G<N>/` + LSL suffix (superseded the earlier `pair_id`/ROS_DOMAIN_ID/port scheme) | ✅ Complete |
| 7 | Homing at source (Zero re-zeros `/estimated_states` in the Kalman node), Skip Iteration, two-group precision, CSV export | ✅ Complete |

**Current hardware state:** Real Teensy 4.1 encoder connected; `mock_encoder` is launched only when `use_mock_encoder:=true` (default `false`, i.e. it waits for the Teensy). Use `mock_robot_controller.py` or `mock_ui.py` for sending experiment triggers.

**⚠ Firmware/host requirements (see §13/§14):** the Teensy firmware still uses `EN_RES 4096.0` while the host expects `ticks_per_rev: 8192` — reconcile before trusting real-hardware position. For real hardware also: the firmware's `GROUP_NUMBER` must equal the launch `group_number:=N`, and the verifier must run on `ROS_DOMAIN_ID=156` to match the firmware's domain (§13).

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
│   │   │   ├── mock_encoder.py             # synthetic /encoder_raw (launched only with use_mock_encoder:=true)
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
                    │   position = RAW-zeroed│  vel/acc = KF estimate
                    │   sub /zero_estimated_states (Empty) → re-zero at source
                    └─────────┬─────────────┘
                              │
                    ┌─────────▼─────────────┐
                    │  /estimated_states    │  ROS 2 topic — EncoderState
                    │  position already zeroed (shared by all consumers)
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
               │                           (position already zeroed upstream)  │
               │   sub /actual_states    → WS broadcast actual_states         │
               │   sub /event_trigger    → WS broadcast event_trigger         │
               │   sub /eval_live        → WS broadcast eval_live             │
               │   sub /eval_summary     → WS broadcast eval_summary          │
               │   lsl-actual-states thread → converts deg→rad, applies       │
               │                             _actual_pos_offset_rad,          │
               │                             pub /actual_states               │
               │   lsl-event-trigger thread → pub /event_trigger              │
               │   pub /zero_estimated_states (Empty) on Zero press           │
               │   ws-server (asyncio :9090) + http-server (:8000)  [fixed]   │
               │   GET /config.json → {"ws_port": 9090}                       │
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
                                             │ WebSocket :9090
                             ┌───────────────▼───────────────┐
                             │       Browser (app.js)         │
                             │  fetches /config.json on load  │
                             │  uPlot charts @ 30 Hz          │
                             │  Criteria tab (Phase 4)        │
                             │  Pos Sync / Zero (Phase 5/7)   │
                             │  Skip Iteration, Save CSV (P7) │
                             └───────────────────────────────┘
```

### Machine topology

**Verifier machine runs the ENTIRE ROS graph:** Teensy 4.1 + micro-ROS agent → `/encoder_raw` → `encoder_reader` → `/estimated_states` → `web_visualizer` + `experiment_evaluator`. Also runs: HTTP file server and WebSocket server.

**Robot machine** (when separate) runs only the robot controller, which emits **LSL** `ActualStates` + `EventTrigger`. No ROS on the robot machine in real deployment.

**Cross-machine transport = LSL only.** All ROS/DDS traffic is local to the verifier machine. Groups are isolated by ROS **namespace** (`/G<N>/`), so multiple verifiers can share one LAN/domain without cross-talk; the LSL suffix (`_N`) isolates the cross-machine robot→verifier streams.

**Cross-thread sync:** ROS callbacks → `asyncio.run_coroutine_threadsafe()` → WS loop thread.  
**Self-loop fanout:** `web_visualizer` publishes `/actual_states` + `/event_trigger` via LSL workers, then also subscribes to both — subscription callbacks do the WS broadcast.

**Zeroing (Phase 7 — "homing offset"):** The estimated-side zero lives in the **Kalman node**, not in `web_visualizer`. Pressing **Zero** in the browser makes `web_visualizer` publish an empty message on `/zero_estimated_states`; `encoder_reader._zero_cb` captures the current raw position as `_zero_offset_rad`, so `/estimated_states.position` is re-zeroed **at the source**. Every downstream consumer (WS broadcast, LSL `EstimatedStates` outlet, `experiment_evaluator`) then shares one zeroed frame with no per-consumer patching. Only the **actual**-side offset (`_actual_pos_offset_rad`) is still applied locally in `web_visualizer`, because `/actual_states` is produced there from the LSL inlet.

**Position semantics:** `/estimated_states.position` is now the **raw encoder position** (ticks→rad) minus `_zero_offset_rad` — *not* the Kalman position state (`x[0]`, which is commented out). `velocity` and `acceleration` are still the KF estimates (a constant zero offset has zero derivative, so they are unaffected).

---

## 3. ROS 2 Message & Service Interfaces

### `EncoderRaw`
```
std_msgs/Header header      # frame_id "encoder"
int32   ticks               # cumulative signed tick count
float64 raw_position        # ticks → rad. Firmware uses 4096; host KF uses 8192 (see §14 mismatch)
uint32  dt_us               # hardcoded 10000 (not measured — known issue)
```

### `EncoderState`
```
std_msgs/Header header
float64 position            # RAW encoder position minus _zero_offset_rad [rad] — NOT the KF x[0] estimate
float64 velocity            # KF estimate [rad/s]
float64 acceleration        # KF estimate [rad/s²]
float64 pos_variance        # posterior P[0,0]
float64 vel_variance        # posterior P[1,1]
float64 acc_variance        # posterior P[2,2]
int32   raw_ticks
```
**Position note:** `position` carries the source-zeroed raw encoder angle so that the browser, the LSL `EstimatedStates` outlet, and `experiment_evaluator` all share one zeroed frame. The Kalman position state is currently unused for output.

### `/zero_estimated_states` payload
```
std_msgs/Empty              # no fields — arrival = "capture current raw position as the new zero"
```
Published by `web_visualizer` when the browser presses **Zero**; consumed by `encoder_reader`.

### `ActualStates`
```
std_msgs/Header header      # frame_id "robot_controller"
float64 actual_position     # [rad] — converted from degrees by web_visualizer; offset applied
float64 actual_velocity     # [rad/s]
float64 actual_acceleration # [rad/s²]
```

### `EventTrigger`
```
std_msgs/Header header      # frame_id "robot_controller" (LSL bridge) / "browser" / "mock_ui"
string  event               # Full JSON payload string. ALWAYS contains key "action".
                            # "mode" key indicates Auto/Test/STOP.
                            # All other keys are experiment-specific.
```
**IMPORTANT:** `msg.event` is the complete JSON string — not a short type tag.
Parse with `json.loads(msg.event)`. Dispatch on `payload["action"]`.
Stop condition: `payload["mode"] == "STOP"` or `payload["action"] == "stop"`.
Skip condition: `payload["action"] == "skip_iteration"` (advances pick_place waypoint / precision trial).
`pick_place` payload uses key `"order_sequence"` (not `"sequence"`).
`precision` payload uses keys `"init_pos"` + `"target_pos"` (evaluator reads `target_pos`; see §14 — the CLI `mock_robot_controller` still emits `tar_pos` and is out of sync).

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
| `/zero_estimated_states` | `std_msgs/Empty` | `web_visualizer` (on Zero press) | `encoder_reader` (re-zero at source) |

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

**Multi-group suffix:** When `group_number = N ≠ 0`, **stream name AND source_id** both get suffix `_N` (e.g. `ActualStates_5`, `EventTrigger_5`). This suffix must be consistent across ALL producers for a group:
- verifier: `web_visualizer` `group_number` param (string, from launch) → `_suf()` applied in code (`0`/`""` → no suffix)
- robot mock: `CV_GROUP_NUMBER` env var / `--group-number` CLI arg → suffix applied before `create_outlet()`
- `_resolve_stream` grabs `streams[0]` — the unique name+source_id suffix ensures it finds the correct group's stream

`utils.py:create_outlet()` is the shared factory for all LSL outlets.

---

## 6. WebSocket Protocol

**Port:** fixed `9090` (WebSocket)  
**HTTP port:** fixed `8000`  
**Format:** newline-delimited JSON text frames

Ports are no longer group-derived (groups run on different machines / IPs). The removed per-group port scheme is archived in `docs/Per-Group Port Configuration (archived).md`.

**Port discovery:** Browser fetches `GET /config.json` from the HTTP server on page load. Response: `{"ws_port": N}` (now always `9090`). `app.js` uses this port to open the WebSocket instead of a hardcoded value — retained as indirection even though the port is fixed.

### Server → Browser

| `topic`            | `data` keys |
|--------------------|-------------|
| `estimated_states` | `stamp`, `position` (already zeroed at source by the Kalman node), `velocity`, `acceleration`, `pos_variance`, `vel_variance`, `acc_variance`, `raw_ticks` |
| `actual_states`    | `stamp`, `actual_position` (deg→rad converted + offset applied), `actual_velocity`, `actual_acceleration` |
| `event_trigger`    | `stamp` + **all keys spread from `msg.event` JSON** (action, mode, + experiment-specific) |
| `eval_live`        | `stamp`, `action` + **all keys spread from `ExperimentEval.data` JSON** |
| `eval_summary`     | `stamp`, `action` + **all keys spread from `ExperimentEval.data` JSON** |
| `time_sync`        | `ref_stamp` |
| `criteria_snapshot`| full criteria dict (all keys + values); sent once on WS connect |
| `criteria_ack`     | `success`, `message`, `criteria` (full dict, only on success) |

There is **no `zero_ack`** anymore (Phase 7). Zeroing is done at the source in the Kalman node, so subsequent `estimated_states` broadcasts already carry the zeroed position — the browser does not need an acknowledgement or a client-side offset.

**Snapshot on connect:** last cached message for each topic + `criteria_snapshot` sent immediately.

### Browser → Server

| `command`          | `data` keys | Effect |
|--------------------|-------------|--------|
| `time_sync`        | —           | Server sets `ref_stamp = now`, broadcasts `time_sync` topic |
| `stop_experiment`  | —           | Server publishes stop `EventTrigger` (`{"mode":"STOP","action":"stop"}`) |
| `skip_iteration`   | —           | Server publishes `EventTrigger` `{"action":"skip_iteration"}` → evaluator advances the current pick_place waypoint / precision trial |
| `criteria_update`  | `{ key: value }` | Server calls `/update_criteria` service, broadcasts `criteria_ack` |
| `pos_sync`         | `delta_rad` | Server adds `delta_rad` to `_actual_pos_offset_rad`; future actual positions shift |
| `zero_set`         | `act_rad`   | Server publishes `Empty` to `/zero_estimated_states` (Kalman re-zeros estimated at source) **and** adds `act_rad` to `_actual_pos_offset_rad` (actual side). No `zero_ack` reply. |

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
- **Parameter:** `ticks_per_rev` (default `8192`) → `_ticks_to_rad = 2π / ticks_per_rev`
- **Subscribes:** `/encoder_raw` (`EncoderRaw`), `/zero_estimated_states` (`std_msgs/Empty`)
- **Publishes:** `/estimated_states` (`EncoderState`)
- **Output position (Phase 7):** `out.position = raw_pos_rad - _zero_offset_rad`, where `raw_pos_rad = ticks × _ticks_to_rad`. The Kalman position estimate `x[0]` is **commented out** — only `velocity` (`x[1]`) and `acceleration` (`x[2]`) come from the filter.
- **`_zero_cb(Empty)`:** sets `_zero_offset_rad = _last_raw_pos_rad` (the most recent unzeroed position), re-zeroing `/estimated_states` at the source for all consumers.

### `web_visualizer` (`scripts/web_visualizer.py`)
- **5 threads:** ROS spin, `ws-server` (asyncio), `http-server`, `lsl-actual-states`, `lsl-event-trigger`
- **Subscribes:** `/estimated_states`, `/actual_states`, `/event_trigger`, `/eval_live`, `/eval_summary`
- **Publishes:** `/actual_states`, `/event_trigger`, `/zero_estimated_states` (`Empty`, on Zero press)
- **Service client:** `/update_criteria` (async call inside WS handler)
- **LSL outlet:** `EstimatedStates` (3 × float32) — name + source_id suffixed by `_suf()` when `group_number` ≥ 1
- **LSL inlets:** `ActualStates` (float), `EventTrigger` (string/JSON) — resolved by suffixed name
- **Serializers:** `estimated_states_to_json`, `actual_states_to_json`, `event_trigger_to_json`, `_eval_to_json`

**Parameters:**
- `group_number` (string, default `"0"`; was named `session`) — LSL suffix token, set by launch via `ParameterValue(…, value_type=str)`; `_suf = (lambda n: f"{n}_{group_number}") if group_number not in ("", "0") else (lambda n: n)` — applied to all LSL stream names + source_ids (`0`/`""` → no suffix)
- `ws_port` — fixed `9090` (from params.yaml; no longer group-derived)
- `http_port` — fixed `8000` (from params.yaml)
- `ws_host` — default `"0.0.0.0"` (listens on all interfaces)
- The node runs under the namespace `/G<N>/` (from `PushRosNamespace` in the launch), so its topics are `/G<N>/estimated_states`, etc.

**`/config.json` wiring:**
```python
# in _run_http_server:
self._http_server.ws_port = self._ws_port   # inject port onto server object

# in _SilentHTTPHandler.do_GET:
if self.path == "/config.json":
    body = json.dumps({"ws_port": self.server.ws_port}).encode()
    ...
```

**Position offset state (Phase 5/7):**
```python
self._actual_pos_offset_rad = 0.0   # cumulative; subtracted from every incoming LSL actual position
# NOTE (Phase 7): _est_zero_rad and _last_est_pos_rad are GONE — the estimated-side
# zero now lives in the Kalman node, applied upstream of every consumer.
self._zero_estimated_pub            # publisher: Empty → /zero_estimated_states
```

**Key callbacks:**
- `_estimated_states_cb`: pushes to LSL outlet and WS broadcasts the message **as-is** — position already arrives zeroed from the Kalman node, so the LSL outlet and WS carry the same frame (no patching).
- `_actual_states_worker`: converts position deg→rad, subtracts `_actual_pos_offset_rad`, publishes to `/actual_states`
- `_handle_command`: dispatches `time_sync`, `stop_experiment`, `skip_iteration`, `criteria_update`, `pos_sync`, `zero_set`
  - `skip_iteration` → publishes `EventTrigger {"action":"skip_iteration"}`
  - `zero_set` → publishes `Empty` to `/zero_estimated_states` (estimated side) + adds `act_rad` to `_actual_pos_offset_rad` (actual side)
- `_get_criteria_snapshot`: called on each new WS connection; sends `{}` to `/update_criteria` to read current state
- `_broadcast` / `_broadcast_async`: thread-safe WS fanout; caches last message per topic

### `experiment_evaluator` (`scripts/experiment_evaluator.py`)
- **Parameters:** `group_number` (default: `"0"`, declared with `dynamic_typing=True` so launch may pass it as int or str), `criteria_file_path` — `group_number` selects the criteria row (was `pair_id`)
- **Subscribes:** `event_trigger`, `estimated_states` (relative → `/G<N>/…`)
- **Publishes:** `eval_live` (throttled every 0.1 s), `eval_summary` (on finish) — relative → `/G<N>/…`
- **Service server:** `update_criteria` (relative → `/G<N>/update_criteria`) — validates key names and non-negative values; updates `self._criteria` in-place; changes are in-memory only (lost on restart)
- **Criteria:** loaded from `criteria.yaml` at startup into `self._criteria` dict (keys normalized to strings)
- **Constants:** `SETTLING_THRESHOLD_rad = 0.01`, `SETTLING_WINDOW_s = 0.5`, `LIVE_PUB_INTERVAL_s = 0.1`
- **Settling detection:** time-based — requires continuous 0.5 s inside band. `cont_band_entry_s` resets on any band exit; `first_band_entry_s` records first entry (never resets) for `settling_time_s` reporting.
- **Variable naming:** all variables carry unit suffixes (`_rad`, `_rad_s`, `_rad_s2`, `_s`, `_pct`, `_count`)
- **Skip handling (Phase 7):** `action=="skip_iteration"` → `_skip_iteration()` → `_skip_waypoint()` (pick_place) / `_skip_trial()` (precision); no-op for ptp/performance. `_skip_trial()` is the single funnel for both the manual skip and the two precision auto-skip timeouts (it owns the skip-counter selection and phase flip).
- **ptp summary:** reports `final_error_rad` + `pass_final_error` (not `max_error`); `pass_overshoot` only true when settled
- **pick_place:** per-waypoint results include `overshoot_pct`, `settling_time_s`, `pass_error/pass_overshoot/pass_settling`; summary adds a `skipped` count; skipped waypoints are excluded from `avg_error_rad`
- **precision (Phase 7, rewritten):** two phase-groups — `target_group` (init→target) and `return_group` (return→init) — each with `num_trials, num_skipped, mean_error_rad, std_error_rad, max_error_rad, pass_error`. Reads `payload["target_pos"]` + `payload["init_pos"]`. Auto-skips a stuck phase via a time-to-halfway timeout.
- **performance:** `cmd_speed_rad_s` and `cmd_accel_rad_s2` taken from payload first, then criteria as fallback
- **`pick_place` payload key:** reads `payload["order_sequence"]` — senders must use this key (not `"sequence"`)
- See §8 for full experiment logic.

### `mock_ui` (`mock_UI/mock_ui.py`)
- **Tkinter GUI** combining mock encoder knobs + LSL/ROS2 event sender. Developer tool for testing the full pipeline without real hardware or a robot controller.
- **ROS2 publishers:** `/encoder_raw` (EncoderRaw), `/event_trigger` (EventTrigger)
- **LSL outlets:** `ActualStates` (3 × float32, 50 Hz), `EventTrigger` (1 × string, IRREGULAR)
- **Two knobs:** Encoder knob → drives `/encoder_raw`; ActualStates knob → drives LSL `ActualStates`
- **Sync mode (bidirectional):** when ON, whichever knob the user moves this frame drives the other by the same delta (encoder wins ties). Mirrors *rate of change*, not absolute position.
- **Fine / Rough mode:** toggles drag sensitivity between 0.1 deg/px (fine) and 1.0 deg/px (rough)
- **Control direction:** drag up = anticlockwise (negative), drag down = clockwise (positive)
- **Readout:** both knobs display rad and deg simultaneously
- **Command entry:** same ptp/pp/perf/prec/stop command syntax as `mock_robot_controller` — publishes to both LSL EventTrigger and ROS2 `/event_trigger`. Its `prec` emits the **correct** `target_pos` key (unlike the CLI `mock_robot_controller`).
- **`ticks_per_rev`:** read from `params.yaml` (`mock_encoder.ticks_per_rev`, default `8192` via `_load_ticks_per_rev()`) so it always matches the KF/hardware
- **Multi-group:** reads `CV_GROUP_NUMBER` env / `--group-number` CLI; `_group_suffix()` appends `_N` to LSL name + source_id, and `_group_namespace()` sets the ROS node namespace `G<N>` (default `G0` — mock_ui always namespaces its `/G<N>/encoder_raw` + `/G<N>/event_trigger`)

**suffix + namespace helpers (mock_ui):**
```python
def _group_suffix(group_number=None) -> str:      # LSL: 0/empty → "", N → "_N"
    n = str(group_number if group_number is not None else os.environ.get("CV_GROUP_NUMBER", "")).strip()
    return f"_{n}" if n and n != "0" else ""

def _group_namespace(group_number=None) -> str:   # ROS ns: ALWAYS present, default "G0"
    n = str(group_number if group_number is not None else os.environ.get("CV_GROUP_NUMBER", "")).strip() or "0"
    return f"G{n}"
```

### `mock_robot_controller` (`scripts/mock_robot_controller.py`)
- **NOT a ROS node.** Pure Python + pylsl. Run separately.
- **LSL outlets:** `ActualStates` (3 × float32, REGULAR), `EventTrigger` (1 × string, IRREGULAR)
- **Telemetry:** continuous waveform (trapezoid/sine/step) in background thread
- **Unit output:** position sent in **degrees** (`math.degrees(p)`); velocity and acceleration in rad/s and rad/s² (numerical derivatives of raw radian waveform — not converted)
- **Multi-group:** reads `CV_GROUP_NUMBER` env / `--group-number` CLI; appends `_N` suffix to YAML-loaded `name` and `source_id` before `create_outlet()`. **No ROS namespace** (pure LSL). Suffix behaviour unchanged from the pair_id era.
- **CLI commands:**

| Command | Parameters | Example | JSON sent |
|---|---|---|---|
| `ptp <value> <unit>` | `value`: increment in `unit`; `unit`: `index` \| `degree` \| `rad` | `ptp 5 index` | `{mode:"Auto", action:"point_to_point", value:5.0, unit:"index"}` |
| `pp <n> <seq> <dirs> <gripper>` | `n`: waypoint count; `seq`: comma-sep integers [index]; `dirs`: comma-sep `CW`\|`CCW`; `gripper`: `true`\|`false` | `pp 3 0,36,72 CW,CCW,CW true` | `{mode:"Auto", action:"pick_place", num:3, order_sequence:[0,36,72], directions:["CW","CCW","CW"], use_gripper:true}` |
| `perf <speed> <accel>` | `speed` [rad/s]; `accel` [rad/s²] | `perf 1.0 2.0` | `{mode:"Test", action:"performance", speed:1.0, accel:2.0}` |
| `prec <init> <tar> <repeat> <unit>` | `init`, `tar`: positions in `unit`; `repeat`: trial count (integer); `unit`: `index` \| `degree` | `prec 0 36 10 index` | `{mode:"Test", action:"precision", init_pos:0, tar_pos:36, repeat:10, unit:"index"}` |
| `stop` | — | — | `{mode:"STOP", action:"stop"}` |
| `quit` / `q` | — | — | exit |

> **⚠ `prec` is out of sync (see §14):** the CLI emits `tar_pos`, but the evaluator reads `target_pos` → precision via `mock_robot_controller` raises `KeyError`. Use `mock_ui` (which sends `target_pos`) until the CLI key is fixed.

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
- eval_live: `target_rad`, `current_pos_rad`, `current_error_rad`, `elapsed_s`
- eval_summary: `target_rad`, `final_error_rad`, `overshoot_pct`, `settling_time_s`, `pass_final_error`, `pass_overshoot` (only true when settled), `pass_settling`

#### `pick_place`
```json
{"mode":"Auto","action":"pick_place","num":3,"order_sequence":[0,36,72],"directions":["CW","CCW","CW"],"use_gripper":true}
```
- **Key name:** `"order_sequence"` (NOT `"sequence"`)
- Waypoints: `order_sequence[idx] × RAD_PER_INDEX` (absolute, index units)
- Terminates: **auto** after settling at the last waypoint, or explicit stop
- Supports **skip** (button or `skip_iteration`): the current waypoint is recorded as `skipped:true` (excluded from `avg_error_rad`) and the next waypoint's travel baseline becomes the robot's *actual* current position
- eval_summary: `total_waypoints`, `avg_error_rad`, `pass_avg_error`, `passed`, `failed`, `skipped`, `details[]`
- each `details[]` entry: `waypoint`, `target_rad`, `final_error_rad`, `overshoot_pct`, `settling_time_s`, `pass_error`, `pass_overshoot`, `pass_settling` (+ `skipped:true` on skipped ones)

#### `performance`
```json
{"mode":"Test","action":"performance","speed":1.0,"accel":2.0}
```
- Terminates: explicit stop only
- eval_summary: `commanded_speed_rad_s`, `peak_speed_rad_s`, `commanded_accel_rad_s2`, `peak_accel_rad_s2`, `pass_speed`, `pass_accel`

#### `precision` (Phase 7 — two phase-groups)
```json
{"mode":"Test","action":"precision","init_pos":0,"target_pos":36,"repeat":10,"unit":"index"}
```
- **Payload keys:** `init_pos` + `target_pos` (evaluator reads `payload["target_pos"]`; the CLI mock's `tar_pos` is a bug — §14).
- **Two-phase state machine:** each cycle is init→target (**target_group**, `_prec_at_target=False`) then return→init (**return_group**, `_prec_at_target=True`). Both phases are now *measured*: the settled position at each phase is recorded into `_trial_positions_rad` / `_return_positions_rad`. Termination lives in the **return** phase — the run finishes once `len(return_positions) + return_skipped ≥ repeat`.
- **Auto-skip timeout:** if the robot crosses the 50% "counting band" toward the goal but never settles within `2 × (time-to-halfway) + buffer_reach_t_s` (`buffer_reach_t_s = 1.0 s`), that phase is force-skipped via `_skip_trial()`.
- eval_live: phase-aware `target_rad` (target while approaching, init while returning), `current_pos_rad`, `current_error_rad`, `trials_done`, `returns_done`, `trials_skipped`, `trials_total`, `elapsed_s`
- eval_summary: `target_rad`, `init_rad`, `target_group{...}`, `return_group{...}` where each group = `num_trials`, `num_skipped`, `mean_error_rad`, `std_error_rad`, `max_error_rad`, `pass_error`

### Manual & auto skip (Phase 7)
- **Manual:** the browser **Skip Iteration** button (or a `skip_iteration` WS command) → `EventTrigger {"action":"skip_iteration"}` → `_skip_iteration()`.
  - `pick_place` → `_skip_waypoint()` (advance waypoint, baseline = actual current position)
  - `precision` → `_skip_trial()` (drop current phase measurement, flip phase, count a skip)
  - ptp / performance → no-op (single-shot)
- **Auto (precision only):** the two timeouts in `_update_prec` also call `_skip_trial()`. `_skip_trial()` is the single owner of skip-counter selection (chosen from `_prec_at_target` **before** flipping) and the phase flip — callers must never pre-flip.

### Criteria (`config/criteria.yaml`)
Keyed by `group_number` (the launch arg). Rows are group numbers (e.g. `11`, `12`) plus a `default` fallback used for group 0 or any unlisted group. The loader normalizes keys to strings, so `11:` (int) and `"11":` (str) both match.

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
| `encoder_reader` | `ticks_per_rev` | 8192 | ticks→rad = 2π/ticks_per_rev. **Firmware uses 4096 — see §14** |
| `encoder_reader` | `kf_q_position` | 1e-8 | Process noise (position) |
| `encoder_reader` | `kf_q_velocity` | 1e-8 | Process noise (velocity) |
| `encoder_reader` | `kf_q_acceleration` | 0.001 | Process noise (acceleration) |
| `encoder_reader` | `kf_q_jerk` | 5.0 | Process noise (jerk) |
| `encoder_reader` | `kf_r_position` | 4.65e-6 | Measurement noise — larger = smoother, trusts encoder less |
| `encoder_reader` | `kf_p0` | 1.0 | Initial state uncertainty |
| `mock_encoder` | `ticks_per_rev` | 8192 | must match `encoder_reader`; also read by `mock_ui` |
| `web_visualizer` | `ws_port` | 9090 | WebSocket port (fixed — no longer group-derived) |
| `web_visualizer` | `http_port` | 8000 | HTTP file server port (fixed) |
| `web_visualizer` | `ws_host` | `"0.0.0.0"` | WS listen address |
| `web_visualizer` | `group_number` | `"0"` | LSL stream name suffix token (was `session`); launch passes it as a string; `"0"`/`""` → no suffix |
| `web_visualizer` | `lsl_params.resolve_timeout_s` | 5.0 | LSL stream lookup timeout |
| `web_visualizer` | `lsl_params.estimated_states_stream.sampling_rate_hz` | 500.0 | EstimatedStates outlet rate (code default is 100.0; params.yaml/launch wins → 500 Hz) |
| `mock_robot_controller` | `lsl_params.actual_states_stream.sampling_rate_hz` | 500.0 | |
| `mock_robot_controller` | `waveform_config.type` | `"trapezoid"` | `sine` \| `trapezoid` \| `step` |
| `mock_robot_controller` | `waveform_config.trap_max_velocity` | π (3.1416) | [rad/s] — waveform is computed in rad |
| `mock_robot_controller` | `waveform_config.trap_acceleration` | π/2 (1.5708) | [rad/s²] |

### `config/criteria.yaml`
Per-group evaluation criteria. Edit directly to change pass/fail thresholds.
Launch with `group_number:=11` (or any group number / key in the file) to select a row; group 0 or an unlisted group uses `default`.

---

## 10. Browser Frontend (`web/`)

| File | Role |
|---|---|
| `index.html` | Header (Time Sync, Pos Sync, Zero buttons); two-panel main (left: live, right: profile/zoom/criteria tabs); right tab-bar actions (Save CSV, unit toggle, Auto, Stop); profile header (Skip Iteration, Reset); footer (Clear, Pause, Crop) |
| `style.css` | CSS grid (2 col); flex left panel (3 equal plots); scrollable right panel tab-content |
| `app.js` | WS client, data buffers, uPlot, redraw loop, crop/zoom, unit toggle, pos sync, zero, skip iteration, CSV export |

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

### WS port discovery
`app.js` calls `resolveWsUrl()` on page load before `connect()` (retained even though the port is now fixed at 9090):
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
    paused, profileActive,
    profileAction,          // current experiment action (drives Skip button visibility)
    profileStartTs,
    latestActual,           // most recent actual_states msg
    timeRef,                // epoch-second reference (auto-set on first sample; reset by Time Sync)
    cropMode,
    rightTab,
    trackEvents,            // Auto button: whether to auto-start profile on event_trigger
    targetRef,              // current target_rad from eval_live [rad, server frame]
    livePosUnit,            // "rad" | "deg" — left panel
    profilePosUnit,         // "rad" | "deg" — right panel (shared by profile + zoom)
}
// NOTE (Phase 7): no estZeroRad. Zeroing is done server-side in the Kalman node, so
// every position the browser receives is already in the zeroed frame.
```

### Position pipeline (browser side)
All positions arrive from server already offset (Kalman `_zero_offset_rad` / `_actual_pos_offset_rad` applied). Browser only applies unit scale, and **wraps the live estimated position** into `[0, 2π)`:
```
live position:    display = wrapAngle(position_rad) * pScale   // pScale = 1.0 (rad) or 180/π (deg)
profile/zoom pos: display = position_rad * pScale              // not wrapped
```
`applyPos(arr, 0, scale)` — offset is always 0 (server handles it). Velocity and acceleration are never wrapped.

### Pos Sync and Zero buttons
- **Pos Sync**: sends `{ command: "pos_sync", data: { delta_rad: act - est } }` — server shifts all future actual positions so they align with estimated at press time. Both browser and evaluator see corrected values.
- **Zero**: sends `{ command: "zero_set", data: { act_rad: act } }` — server publishes `Empty` to `/zero_estimated_states` (Kalman re-zeros estimated at source) and adds `act_rad` to `_actual_pos_offset_rad` (actual side). **No `zero_ack`** — the next `estimated_states` frame already arrives zeroed, so the browser needs no client-side offset.

### Skip Iteration and Save CSV
- **Skip Iteration** (`#btn-skip-iteration`): visible only while a `precision` or `pick_place` profile is active (`updateSkipBtn()`); sends `{ command: "skip_iteration" }`.
- **Save CSV** (`#btn-save-csv`): exports the active tab's buffer (profile or zoom) via `buildCSV()`/`saveCSV()` — columns `time_s, est_pos_<unit>, act_pos_<unit>, est_vel_rad_s, act_vel_rad_s, est_acc_rad_s2, act_acc_rad_s2`. Uses the File System Access API when available, else an anchor download.
- **Reset** buttons: `#btn-reset-profile` and `#btn-reset-zoom` clear the per-plot zoom lock (`zoomedPlots`) and re-fit; double-clicking a right-panel plot does the same for that plot.

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
| `actual_states` | `onActualStates` — `state.latestActual = msg.data` |
| `event_trigger` | `onEventTrigger` — `"stop"` → `stopExperiment()`; a genuine experiment action → `startExperiment()`; `skip_iteration` loops back here but does **not** restart the profile |
| `eval_live` | `onEvalLive` — sets `state.targetRef`/`profileAction`, renders live metrics |
| `eval_summary` | `onEvalSummary` — calls `stopProfile()`, renders summary (incl. precision two-group + waypoint details), switches to PROFILE tab |
| `criteria_snapshot` | `onCriteriaSnapshot` — renders editable CRITERIA tab rows |
| `criteria_ack` | `onCriteriaAck` — green/red flash on edited field |
| `time_sync` | `state.timeRef = msg.data.ref_stamp` |

(There is no `zero_ack` handler anymore — Phase 7.)

### Eval display — positions already zeroed
Position fields (`target_rad`, `current_pos_rad`, …) are rendered as-is by `appendRow()` — they already arrive in the zeroed frame from the evaluator, so there is no client-side subtraction (the old `POSITION_ABS_FIELDS` / `estZeroRad` logic is gone).

### Precision & pick_place summary rendering
- Precision: `renderSummary()` renders `target_group` as "Target reaching performance" and `return_group` as "Initial position returning performance" via `renderPrecisionGroup()` (a PASS/FAIL header + metric rows).
- Pick & place: each `details[]` waypoint renders a SKIPPED / PASS / FAIL header (`pass_error && pass_overshoot && pass_settling`) plus its metric rows.

### uPlot notes
- NEVER set `cursor.points.show: true` — causes crash. Use default or a function.
- Left panel plots: `cursor.drag: { x: false, y: false }` (crop overlay handles selection)
- Right panel plots: `cursor.drag: { x: true, y: false }` (x-zoom only, y auto-fits)
- `snapCursorToEst` cursor.move snaps the crosshair to the estimated series y-value

---

## 11. Firmware (Teensy 4.1)

- **Location:** `encoder_data_publisher/src/main.cpp`
- **Transport:** USB serial, 115200 baud, micro-ROS
- **Encoder pins:** CH_A=7, CH_B=6; `EN_RES 4096.0`
- **ROS domain:** `rcl_init_options_set_domain_id(..., 156)` — **hardcoded to 156** (real Teensy only visible on `ROS_DOMAIN_ID=156`; see §13/§14)
- **Timer:** 10 ms → publishes `EncoderRaw`
- **Timestamp:** `rmw_uros_epoch_nanos()` (synced via `rmw_uros_sync_session(1000)`)
- **Known:** `dt_us` hardcoded to 10000 — not measured from actual timer elapsed
- **Known:** firmware `EN_RES 4096.0` vs host `ticks_per_rev: 8192` — the host halves the true angle unless reconciled (§14)

---

## 12. Build & Run

### Standard (single pair)

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

# Build (interface first if .srv changed, then main package)
colcon build --packages-select claude_visualizer_interface claude_visualizer
source install/setup.bash

# Launch pipeline (real encoder — waits for Teensy)
ros2 launch claude_visualizer bringup.launch.py
# Hardware-free mode (start the synthetic mock_encoder instead of the Teensy):
ros2 launch claude_visualizer bringup.launch.py use_mock_encoder:=true
# With a specific group (namespaces the whole ROS graph under /G<N>/, sets the
# LSL suffix _N, and selects the criteria row):
ros2 launch claude_visualizer bringup.launch.py group_number:=5

# Mock controller (separate terminal — not a ROS node, no sourcing needed)
ros2 run claude_visualizer mock_robot_controller
# Then at prompt: ptp 5 index | pp 3 0,36,72 CW,CCW,CW true | perf 1.0 2.0 | prec 0 36 10 index | stop | quit

# Browser (hard-refresh after any web/ file change — no build needed for CSS/JS/HTML)
# http://localhost:8000
```

### Multi-group (namespace isolation)

```bash
# --- Group 5 verifier (runs full ROS graph + web server) ---
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch claude_visualizer bringup.launch.py group_number:=5
# ROS graph under /G5/…, LSL streams suffixed _5, web on fixed :9090/:8000

# --- Group 5 robot / mock (LSL source; namespaced if it publishes ROS) ---
python3 src/claude_visualizer/scripts/mock_robot_controller.py --group-number 5   # pure LSL → ActualStates_5
# OR the GUI mock (also namespaces its ROS topics under /G5/):
# python3 src/claude_visualizer/mock_UI/mock_ui.py --group-number 5

# Browser: http://<verifier-ip>:8000   (each group's verifier is a different machine/IP)

# --- Verify the group's ROS graph is namespaced ---
ros2 topic list        # → /G5/encoder_raw, /G5/estimated_states, /G5/eval_live, …
```

### Group → isolation table

| group_number | ROS namespace | LSL suffix | ws_port | http_port |
|---|---|---|---|---|
| 0 (default) | `/G0/` | (none) | 9090 | 8000 |
| 5 | `/G5/` | `_5` | 9090 | 8000 |
| 11 | `/G11/` | `_11` | 9090 | 8000 |

**There is always a namespace** (default group 0 → `/G0/`). Ports are **fixed** — different groups run on different machines (distinct IPs), so per-group ports are unnecessary. The removed per-group port scheme is archived in `docs/Per-Group Port Configuration (archived).md`.

**Launch args:**
- `group_number` (default: `"0"`) — CLI-only. Puts the whole ROS graph under `/G<N>/`, sets the LSL suffix `_N` (none for 0), and selects the criteria row in `criteria.yaml`. Replaces `pair_id` (and the old `ROS_DOMAIN_ID` / per-group port scheme).
- `use_mock_encoder` (default: `"false"`) — when `true`, launches `mock_encoder` under an `IfCondition`; when `false`, the graph waits for the real Teensy on `/G<N>/encoder_raw`
- `waveform` (default: `"trapezoid"`) — mock_encoder waveform (`sine`|`trapezoid`|`step`)
- All nodes are wrapped in `GroupAction([PushRosNamespace(["G", group_number]), …])` (no OpaqueFunction — the namespace is a substitution and `group_number` is passed as a `ParameterValue(str)`); `web_visualizer` gets `group_number` (LSL suffix); `experiment_evaluator` gets `group_number` for criteria. The GroupAction is preferred over per-node `namespace=` so a newly added node can't silently miss the namespace.

**Build notes:**
- `colcon build` is required for any `.py` script change (scripts are copied to install, not symlinked — `--symlink-install` conflicts with micro_ros_package)
- Web files (`index.html`, `app.js`, `style.css`) are served from the installed share dir — also require `colcon build`
- `claude_visualizer_interface` must be rebuilt whenever `.srv` or `.msg` files change

---

## 13. Multi-Group Isolation (ROS namespaces)

Isolation is by **ROS namespace**, keyed on a single `group_number = N`. (This replaces the earlier `pair_id` model that used `ROS_DOMAIN_ID` + per-group ports; that port scheme is archived in `docs/Per-Group Port Configuration (archived).md`.)

### Problem
Running multiple robot+verifier groups on one LAN causes collision risks:
1. **ROS 2 topics:** groups on the same graph receive each other's `/estimated_states`, `/event_trigger`, etc. — evaluator gets the wrong data.
2. **LSL:** `_resolve_stream(name)` grabs `streams[0]` from any stream with that name on the LAN — verifier latches onto the wrong robot's telemetry.

### Solution: `group_number = N` → namespace + LSL suffix

Fully declarative in the launch (no OpaqueFunction) — the namespace is a substitution and the group number is forced to a string param:
```python
# bringup.launch.py
group_number = LaunchConfiguration("group_number")
namespace    = ["G", group_number]                 # "G0", "G5", …  (always present)

# web_visualizer param (string; ParameterValue avoids "5" → int inference):
{"group_number": ParameterValue(group_number, value_type=str)}

# all nodes namespaced by the group boundary:
GroupAction([PushRosNamespace(namespace), *nodes])
```
The "group 0 → no LSL suffix" rule lives in `web_visualizer._suf` (`"0"`/`""` → no suffix), not the launch.

| Layer | Mechanism | Notes |
|---|---|---|
| ROS 2 | node **namespace** `/G<N>/` | All nodes wrapped in `GroupAction([PushRosNamespace("G<N>"), …])`. Requires **relative** topic names (no leading `/`) so the namespace attaches. Always present (default `/G0/`). |
| LSL | stream name + source_id suffix `_N` | Cross-machine isolator; `0 → none`, `N → _N` (behaviour unchanged from the pair_id era). |
| Web | fixed `9090 / 8000` | Not derived — groups live on different machines (distinct IPs). |

### How it reaches each component
| Component | Reads group | Applies |
|---|---|---|
| `bringup.launch.py` | `group_number:=N` (CLI) | `PushRosNamespace(["G", group_number])` on all nodes (via GroupAction); `group_number` → web_visualizer + experiment_evaluator |
| `web_visualizer.py` | `group_number` ROS param | `_suf()` appends `_N` to LSL names + source_ids (`0`/`""` → none) |
| `mock_ui.py` | `--group-number` / `CV_GROUP_NUMBER` | ROS node `namespace="G<N>"` **and** LSL suffix `_N` |
| `mock_robot_controller.py` | `--group-number` / `CV_GROUP_NUMBER` | LSL suffix `_N` only (pure LSL, no namespace) |

### Why namespaces instead of `ROS_DOMAIN_ID`
`ROS_DOMAIN_ID` must be exported **before** the ROS daemon/DDS starts (it can't be set by a launch arg), which forced the old env-file workflow. A **namespace is a plain launch argument** (`group_number:=N`) — no pre-sourcing, no daemon-timing constraint, and it shows up cleanly in `ros2 topic list` as a `/G<N>/` tree. The env file + `make_pair_env.sh` are removed; group is passed on the CLI (`group_number:=N` for the launch, `--group-number N` for the mocks; `CV_GROUP_NUMBER` env still works as a fallback for the mocks).

### Real Teensy + namespaces (implemented in firmware)
`encoder_data_publisher/src/main.cpp` now namespaces the node from a compile-time define, so the real Teensy publishes `/G<N>/encoder_raw`:
```c
#define GROUP_NUMBER 0
#define STR2(x) #x
#define STR(x)  STR2(x)
#define ROS_NAMESPACE "G" STR(GROUP_NUMBER)     // → "G0"
// node init: rclc_node_init_default(&node, "encoder_data_publisher", ROS_NAMESPACE, &support);
// publisher: rclc_publisher_init_default(..., "encoder_raw");   // relative → /G<N>/encoder_raw
```
Two hard requirements for real hardware:
1. **`GROUP_NUMBER` (firmware) must equal `group_number:=N` (launch)** — the Teensy's `/G<N>/encoder_raw` must match what `encoder_reader` subscribes to. Each group flashes its own number.
2. **Domain must match.** The firmware stays on `ROS_DOMAIN_ID = 156` (`rcl_init_options_set_domain_id(&init_options, 156)`), so the verifier must run on domain 156 too: `export ROS_DOMAIN_ID=156` before `ros2 launch`. Groups are still isolated by **namespace** even though they share domain 156. The mock path (mock_ui/mock_encoder) is unaffected — it namespaces itself and runs on whatever domain the host uses.

(If a particular micro-ROS build doesn't expand the relative topic under the node namespace, switch to an absolute `#define ENCODER_TOPIC "/G" STR(GROUP_NUMBER) "/encoder_raw"` in the publisher init — a fallback noted in `main.cpp`.)

### Backward compatibility
`group_number = 0` (default) → namespace `/G0/`, **no** LSL suffix, criteria `default`, ports `9090`/`8000`. There is always a namespace, but group 0 keeps unsuffixed LSL streams.

### Verification (expected topics + stream names)
```
group=0  → /G0/…   | ActualStates / EventTrigger / EstimatedStates          (no suffix)
group=5  → /G5/…   | ActualStates_5 / EventTrigger_5 / EstimatedStates_5
group=11 → /G11/…  | ActualStates_11 / EventTrigger_11 / EstimatedStates_11
```
`ros2 topic list` shows the `/G<N>/` tree; `ros2 topic echo /G5/estimated_states` vs `/G6/…` confirm no cross-talk.

---

## 14. Known Issues

| Location | Issue | Severity |
|---|---|---|
| `encoder_data_publisher/src/main.cpp` | `dt_us` hardcoded to 10000 — not measured from actual timer elapsed | Known |
| `encoder_data_publisher/src/main.cpp` | `EN_RES 4096.0` vs host `ticks_per_rev: 8192` — host computes half the true angle with real hardware unless reconciled | **Bug** |
| `encoder_data_publisher/src/main.cpp` | node now namespaced via `GROUP_NUMBER` → publishes `/G<N>/encoder_raw` (§13). Requirement: firmware `GROUP_NUMBER` must equal launch `group_number:=N`, and host must run `ROS_DOMAIN_ID=156` | Resolved / note |
| `mock_robot_controller.py` `prec` | Emits `tar_pos`, but `experiment_evaluator` reads `target_pos` → precision via the CLI mock raises `KeyError`. Use `mock_ui` (sends `target_pos`) | **Bug** |
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
