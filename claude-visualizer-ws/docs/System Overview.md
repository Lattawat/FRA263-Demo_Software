# claude_visualizer — System Overview

A ROS 2 pipeline that streams encoder kinematics and robot-controller telemetry into a browser dashboard via **LSL (Lab Streaming Layer)** and a built-in **WebSocket** server. The encoder reader runs a constant-jerk Kalman filter for **velocity and acceleration**; the **position** it publishes is the raw encoder angle re-zeroed at the source (see *Homing / Zero* below). All development phases (1–7) are complete and running with a real Teensy 4.1 encoder. `mock_ui.py` and `mock_robot_controller.py` stand in for the robot controller during testing. Phase 6 adds multi-pair LAN scaling so that multiple robot-verifier pairs can run simultaneously on the same network; Phase 7 adds source-side homing (Zero), a Skip-Iteration control, a two-group precision experiment, and CSV export.

---

## File Structure

```
claude_visualizer_ws/
├── src/
│   ├── claude_visualizer/                  # Main ROS 2 package (ament_cmake)
│   │   ├── claude_visualizer/              # Python package (ament-installed)
│   │   │   ├── __init__.py
│   │   │   └── utils.py                    # create_outlet() — shared LSL outlet factory
│   │   ├── config/
│   │   │   ├── params.yaml                 # All node parameters (single source of truth)
│   │   │   └── criteria.yaml              # Per-robot evaluation criteria lookup table
│   │   ├── launch/
│   │   │   └── bringup.launch.py           # Launches encoder_reader + web_visualizer + experiment_evaluator
│   │   ├── mock_UI/
│   │   │   └── mock_ui.py                  # Tkinter GUI: knob encoder + command sender (dev tool)
│   │   ├── scripts/
│   │   │   ├── mock_encoder.py             # synthetic /encoder_raw (launched only with use_mock_encoder:=true)
│   │   │   ├── Kalman_filter.py            # encoder_reader node (constant-jerk KF)
│   │   │   ├── web_visualizer.py           # LSL↔ROS bridge + WS + HTTP server (5 threads)
│   │   │   ├── mock_robot_controller.py    # Non-ROS LSL publisher; experiment CLI
│   │   │   └── experiment_evaluator.py     # Evaluates /estimated_states vs experiment config
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
│       │   └── ExperimentEval.msg
│       ├── srv/
│       │   └── UpdateCriteria.srv
│       ├── CMakeLists.txt
│       └── package.xml
├── encoder_data_publisher/                 # Teensy 4.1 firmware (PlatformIO)
│   ├── src/main.cpp
│   └── platformio.ini
├── pairs/                                  # Per-pair environment files (Phase 6)
│   ├── pair11.env                          # example: source this for pair 11
│   └── pair12.env                          # example: source this for pair 12
├── scripts/
│   └── make_pair_env.sh                    # Generates pairs/pair<N>.env from N
├── docs/
│   ├── System Overview.md                  # ← this file (human guide)
│   └── System Design.md                   # Dense technical reference for developers/AI
└── .venv/                                  # Python venv (--system-site-packages)
```

---

## System Diagram

```
╔═══════════════════════════════════════════════════╗
║  HARDWARE (on Verifier machine)                   ║
║  Teensy 4.1 + quadrature encoder (4096 ticks/rev) ║
║  micro-ROS over USB serial → /encoder_raw @ 100Hz ║
╚══════════════════════╦════════════════════════════╝
                       ║   (mock_encoder.py [use_mock_encoder:=true] or mock_ui replaces this during dev)
           ┌───────────▼───────────┐
           │     /encoder_raw      │  EncoderRaw
           └───────────┬───────────┘
                       │
           ┌───────────▼───────────┐
           │    encoder_reader     │  Kalman_filter.py
           │  vel/acc = KF         │  position = raw − zero
           │  sub /zero_estimated_states (Empty) → re-zero
           └───────────┬───────────┘
                       │
           ┌───────────▼───────────┐
           │  /estimated_states    │  EncoderState (position already zeroed)
           └──────┬────────────────┘
                  │                   ┌─────────────────────────────────────┐
                  │                   │  Robot machine (may be same or      │
                  │                   │  different physical machine)        │
                  │                   │                                     │
                  │                   │  mock_robot_controller (non-ROS)    │
                  │                   │  OR mock_ui (Tkinter GUI)           │
                  │                   │                                     │
                  │                   │  LSL: ActualStates[_N]  (pos=deg)   │
                  │                   │  LSL: EventTrigger[_N]  (JSON)      │
                  │                   │  ROS2: /event_trigger   (mock_ui)   │
                  │                   └──────────────┬──────────────────────┘
                  │                                  │ LSL over LAN (only cross-machine link)
      ┌───────────┼──────────────────────────────────▼──────────────┐
      │           │        web_visualizer (5 threads)                │
      │           │        [All on Verifier machine]                 │
      │           ▼                                                  │
      │   sub /estimated_states → LSL EstimatedStates[_N] outlet    │
      │                         → WS broadcast estimated_states      │
      │   sub /actual_states    → WS broadcast actual_states         │
      │   sub /event_trigger    → WS broadcast event_trigger         │
      │   sub /eval_live        → WS broadcast eval_live             │
      │   sub /eval_summary     → WS broadcast eval_summary          │
      │   lsl-actual-states thread → deg→rad, offset, pub /actual_states │
      │   lsl-event-trigger thread → pub /event_trigger              │
      │   pub /zero_estimated_states (Empty) on Zero press           │
      │   ws-server (asyncio :9000+N)                                │
      │   http-server (:8000+N) serves web/ + /config.json           │
      │   srv client /update_criteria                                │
      └────────────────────────┬─────────────────────────────────────┘
                               │
           ┌───────────────────▼───────────────┐
           │       experiment_evaluator         │
           │  sub /event_trigger → start/stop  │
           │  sub /estimated_states → evaluate │
           │  pub /eval_live    (throttled)     │
           │  pub /eval_summary (on finish)     │
           │  srv server /update_criteria       │
           └───────────────────────────────────┘
                               │ WebSocket :9000+N
               ┌───────────────▼───────────────┐
               │        Browser (app.js)        │
               │  fetches /config.json → WS port│
               │  uPlot charts @ 30 Hz          │
               │  Criteria tab (live editing)   │
               │  Pos Sync / Zero / Skip / CSV  │
               └───────────────────────────────┘
```

---

## ROS 2 Topics

| Topic               | Type             | Producer                                  | Consumer(s)                                      |
|---------------------|------------------|-------------------------------------------|--------------------------------------------------|
| `/encoder_raw`      | `EncoderRaw`     | Teensy / mock_encoder / mock_ui           | `encoder_reader`                                 |
| `/estimated_states` | `EncoderState`   | `encoder_reader`                          | `web_visualizer`, `experiment_evaluator`         |
| `/actual_states`    | `ActualStates`   | `web_visualizer` (from LSL inlet)         | `web_visualizer` (self-loop → WS)                |
| `/event_trigger`    | `EventTrigger`   | `web_visualizer` (LSL inlet) / `mock_ui`  | `web_visualizer` (self-loop → WS), `experiment_evaluator` |
| `/eval_live`        | `ExperimentEval` | `experiment_evaluator`                    | `web_visualizer` (→ WS)                          |
| `/eval_summary`     | `ExperimentEval` | `experiment_evaluator`                    | `web_visualizer` (→ WS)                          |
| `/zero_estimated_states` | `std_msgs/Empty` | `web_visualizer` (on Zero press)     | `encoder_reader` (re-zero position at source)    |

## LSL Streams

| Stream name | Suffix (multi-pair) | Direction | Channels | Rate | Producer |
|---|---|---|---|---|---|
| `ActualStates` | `ActualStates_N` | inlet on verifier | 3 float | 500 Hz | robot machine |
| `EventTrigger` | `EventTrigger_N` | inlet on verifier | 1 string | IRREGULAR | robot machine |
| `EstimatedStates` | `EstimatedStates_N` | outlet on verifier | 3 float | 500 Hz | `web_visualizer` |

Where `N` is the `pair_id` (e.g. `11`). No suffix when running a single pair (pair_id = 0 or unset).

---

## Components

### `encoder_reader` (`scripts/Kalman_filter.py`)
Constant-jerk Kalman filter (state `[position, velocity, acceleration, jerk]`, variable-dt transition). Subscribes `/encoder_raw`, publishes `/estimated_states`. **Velocity and acceleration** come from the filter; the published **position** is the raw encoder angle (`ticks × 2π/ticks_per_rev`) minus a zero offset. It also subscribes `/zero_estimated_states` (`std_msgs/Empty`): each message captures the current raw position as the new zero, re-homing `/estimated_states` at the source for every downstream consumer. `ticks_per_rev` defaults to 8192.

### `web_visualizer` (`scripts/web_visualizer.py`)
The bridge node. Runs 5 threads: ROS executor, asyncio WebSocket server, HTTP file server, LSL-ActualStates worker, LSL-EventTrigger worker. Bridges LSL streams into ROS topics and broadcasts everything to the browser as JSON over WebSocket. Also serves `/config.json` so the browser can discover the correct WebSocket port dynamically.

### `experiment_evaluator` (`scripts/experiment_evaluator.py`)
Evaluates encoder motion against configurable pass/fail criteria. Supports four experiment types: `point_to_point`, `pick_place`, `performance`, `precision`. Publishes live metrics and a final summary. Selects its criteria row from `criteria.yaml` by `pair_id` (the single setup identifier — `pair_id` 0 or any unlisted pair uses `default`). Criteria can be edited live from the browser CRITERIA tab via the `/update_criteria` service.

It also handles **Skip Iteration** (`action:"skip_iteration"`): drop the current pick_place waypoint or precision trial and advance. **Precision is a two-group experiment** — it measures both the init→target reach (*target group*) and the return→init move (*return group*), each reporting mean/std/max error and a pass flag; a stuck phase is auto-skipped after a time-to-halfway timeout. Its payload uses `init_pos` + `target_pos` (note: the CLI `mock_robot_controller` still sends `tar_pos` and is out of sync — use `mock_ui`).

### `mock_robot_controller` (`scripts/mock_robot_controller.py`)
Non-ROS CLI tool. Streams continuous waveform telemetry (trapezoid/sine/step) on LSL `ActualStates` and sends experiment commands on LSL `EventTrigger` from stdin. Use when the real robot controller is not available. Supports `--pair-id` or `CV_PAIR_ID` env for multi-pair LSL suffix.

### `mock_ui` (`mock_UI/mock_ui.py`)
Tkinter GUI combining a mock encoder knob and a mock robot controller knob. Publishes `/encoder_raw` and `/event_trigger` (ROS2) and both LSL streams. Use for full end-to-end testing without any hardware. Features: **bidirectional** Sync mode (either knob drives the other by rate of change), Fine/Rough sensitivity toggle, dual rad/deg readout, command entry field. Reads `ticks_per_rev` from `params.yaml` (default 8192) so it matches the KF/hardware, and its `prec` command sends the correct `target_pos` key. Supports `--pair-id` or `CV_PAIR_ID` env for multi-pair LSL suffix.

---

## How to Run

### Single pair (basic)

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Activate venv (created with --system-site-packages)
source .venv/bin/activate

# 3. Install Python deps (first time only)
pip install pylsl websockets numpy pyyaml catkin-pkg empy lark

# 4. Build (interface package first if .msg/.srv changed)
colcon build --packages-select claude_visualizer_interface claude_visualizer
source install/setup.bash

# 5. Launch the pipeline (real Teensy — waits for hardware on /encoder_raw)
ros2 launch claude_visualizer bringup.launch.py
# Hardware-free (start the synthetic mock_encoder instead of the Teensy):
ros2 launch claude_visualizer bringup.launch.py use_mock_encoder:=true
# With a specific pair's criteria (pair_id also sets ports + LSL suffix):
ros2 launch claude_visualizer bringup.launch.py pair_id:=11

# 6a. Send experiment commands — CLI style (separate terminal)
ros2 run claude_visualizer mock_robot_controller
# prompt: ptp 5 index | pp 3 0,36,72 CW,CCW,CW true | perf 1.0 2.0 | stop
# NOTE: the CLI `prec` command is currently broken (sends tar_pos; evaluator expects
# target_pos). For precision experiments use the GUI mock (6b) instead.

# 6b. OR use the GUI mock (separate terminal)
python3 src/claude_visualizer/mock_UI/mock_ui.py

# 7. Open browser
# http://localhost:8000
```

### Multiple pairs on one LAN

```bash
# Generate env files (run once, keeps in pairs/ dir)
./scripts/make_pair_env.sh 11   # → pairs/pair11.env
./scripts/make_pair_env.sh 12   # → pairs/pair12.env

# --- On the VERIFIER machine for pair 11 ---
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source pairs/pair11.env          # exports ROS_DOMAIN_ID=11, CV_PAIR_ID=11
source install/setup.bash
ros2 launch claude_visualizer bringup.launch.py
# Listens on WS :9011 and HTTP :8011

# --- On the ROBOT machine for pair 11 ---
source pairs/pair11.env          # exports CV_PAIR_ID=11
python3 src/claude_visualizer/scripts/mock_robot_controller.py
# Streams ActualStates_11 and EventTrigger_11 on LSL

# Open browser: http://<verifier-ip>:8011
```

> **Build note:** `colcon build` is required after any `.py` script change — scripts are copied to install, not symlinked (`--symlink-install` conflicts with micro_ros_package). Web files also require a rebuild.

---

## Multi-Pair LAN Scaling — Concept and Reasoning (Phase 6)

This section explains **why** multi-pair scaling was designed the way it was, so the rationale is clear when you revisit or extend it.

### Why is isolation needed at all?

In FRA263, multiple robot-verifier pairs may run on the same lab network simultaneously — for example, one per group or one per station. Without isolation, the system breaks in three different ways that are not immediately obvious:

1. **ROS 2 / DDS bleeds across the network.**  
   ROS 2 uses a protocol called DDS (Data Distribution Service) to automatically discover other ROS nodes on the same network. By default, all ROS nodes using the same "domain ID" can see each other's topics. If Verifier A and Verifier B both run `/estimated_states` on the default domain (ID 0), the `experiment_evaluator` on Verifier A may evaluate encoder data coming from Verifier B's robot — producing completely wrong results.

2. **LSL does not distinguish whose data is whose by default.**  
   Lab Streaming Layer (LSL) is used to send robot telemetry across the LAN (the robot machine → verifier machine). When `web_visualizer` calls `_resolve_stream("ActualStates")`, it finds the first LSL stream with that name on the network — which might be from a completely different robot pair. This is the most dangerous cross-pair contamination because it is silent: the system will run normally but show the wrong robot's data.

3. **Port collisions when two pairs run on the same machine.**  
   If two verifier setups run on the same physical machine (for example, one machine simulating two pairs during development), both would try to bind to port 9090 (WebSocket) and port 8000 (HTTP). The second one fails to start. Even on separate machines, a consistent port scheme makes it easier to know which dashboard belongs to which pair.

### Why a single `pair_id` number?

Rather than asking the operator to configure three separate settings (ROS domain, LSL suffix, web port) separately for each pair, all three are derived from a single integer: `pair_id = N`. This minimizes human error — you only have one thing to set per pair, and everything else follows automatically:

- `ROS_DOMAIN_ID = N` → isolates ROS within each verifier machine
- LSL stream names get suffix `_N` (e.g. `ActualStates_11`) → isolates cross-machine data
- WebSocket port = `9000 + N`, HTTP port = `8000 + N` → allows co-location without collision

### Why an env file (`pairs/pair<N>.env`)?

**What the file is and how it is used.** `scripts/make_pair_env.sh <N>` (N validated to 0–101) writes `pairs/pair<N>.env` containing exactly two lines: `export ROS_DOMAIN_ID=N` and `export CV_PAIR_ID=N`. You **source it on every terminal of every machine the pair spans** before running anything. The verifier machine uses both variables (ROS graph isolation + LSL suffix); the robot machine needs only `CV_PAIR_ID`.

**How each component reads the pair id (implementation method).** From that single number, three isolation layers are derived:
- `bringup.launch.py` declares `pair_id` with **default `os.environ.get("CV_PAIR_ID","0")`**, then an `OpaqueFunction` computes `ws_port = 9000+N`, `http_port = 8000+N`, and the LSL `session` suffix, passing them to `web_visualizer`; `pair_id` is also handed to `experiment_evaluator` to pick the criteria row.
- `web_visualizer` applies the `session` suffix to every LSL stream name + source_id.
- `mock_robot_controller.py` / `mock_ui.py` read a `--pair-id` flag **or** `CV_PAIR_ID` (flag wins).
- `app.js` reads nothing from the env — it discovers the WebSocket port from `/config.json`.

**Why an env file rather than just an `argparse --pair-id` flag everywhere?** `argparse` *is* still available on the mock tools as an override, but it cannot be the primary mechanism:
1. **`ROS_DOMAIN_ID` has to exist in the environment *before* any ROS process / DDS starts** — DDS reads it at initialization. A launch argument or a Python flag applied after the process starts is too late; only a pre-sourced shell variable guarantees the ordering.
2. **One `source` sets it for the whole session, instead of repeating a flag on every command.** A pair spans several separately launched programs (the `ros2 launch`, the mock controller, maybe `mock_ui`, across one or two machines). If each needed `--pair-id N`, a single forgotten flag would silently mis-pair the LSL/ROS data. Sourcing one file makes the value consistent everywhere — the same "set one number, get all three layers" philosophy behind using a single `pair_id`.
3. **The launch file already consumes the env var as its default**, so once you have sourced the file, `ros2 launch …` needs no extra argument (a CLI `pair_id:=N` override still works).
4. So the design is **env-primary, argparse-override**: the env file for the normal one-setting-per-machine case, `--pair-id` only for ad-hoc overrides on the non-ROS mock tools (which cannot receive a ROS launch argument at all).

### ⚠ Caveat: real firmware runs on ROS domain 156

The Teensy firmware hardcodes `ROS_DOMAIN_ID = 156`. A **real** encoder is therefore only visible on domain 156, while `pair<N>.env` sets the domain to N (0–101). With real hardware the verifier will not receive `/encoder_raw` unless the two domains are reconciled (re-flash the firmware to domain N, or run that pair on domain 156). The mock encoder / mock_ui are unaffected because they publish on whatever domain the host environment sets.

### Why LSL and not ROS for cross-machine communication?

The **Teensy micro-ROS agent and the entire ROS graph run on the verifier machine**, not on the robot machine. In real deployment, the robot controller does not speak ROS — it only emits LSL streams. So LSL is the only cross-machine transport in this system. DDS / ROS 2 traffic never crosses between the robot machine and verifier machine; it stays local to the verifier.

This has an important implication: `ROS_DOMAIN_ID` isolation applies only within the verifier machine (preventing one verifier's ROS graph from interfering with another verifier's ROS graph on the same LAN). It does nothing for the robot ↔ verifier link. The LSL suffix is the only thing that isolates cross-machine data.

### Why add `_N` to both the stream name AND source_id?

LSL uses `name` + `source_id` together to uniquely identify a stream. When the verifier searches for its robot's stream with `_resolve_stream("ActualStates_11")`, adding `_11` to the name ensures the search returns only pair 11's stream. Adding `_11` to the `source_id` as well provides a secondary guarantee: if somehow two streams had the same suffixed name, the source_id would still distinguish them.

### Why `/config.json` and not hardcode the WebSocket port in the browser?

The HTML/JavaScript files are static — they are built once and served from the install directory. If the WebSocket port were hardcoded in `app.js` as `:9090`, every pair would need a different build of the frontend. That is impractical.

Instead, the HTTP server (which already runs on the correct port for this pair) serves a tiny JSON file at `/config.json`:
```json
{"ws_port": 9011}
```
The browser fetches this file when the page loads, discovers the correct WebSocket port, and connects. Because the browser uses `location.hostname` (the verifier's IP — the same host that served the page), it automatically connects to the right machine without needing any hardcoded IP addresses either.

### Backward compatibility

All of these changes are backward compatible. If `pair_id` is not set (or is set to 0), the system behaves exactly as it did before Phase 6:
- No suffix on LSL stream names
- Ports: WebSocket `:9090`, HTTP `:8000`
- ROS domain: 0

So existing single-pair setups require no changes.

---

## Homing, Zeroing, and Skip — Concept and Reasoning (Phase 7)

### Why zero at the source instead of in the browser?

Earlier the browser and `web_visualizer` each kept their own zero offset, which meant the number the operator saw on the plot could differ from the number the `experiment_evaluator` scored against. That is dangerous: a pass/fail verdict must be measured in the same frame the operator is looking at.

Phase 7 moves the estimated-side zero **upstream into the Kalman node**. Pressing **Zero** in the browser sends a `zero_set` command; `web_visualizer` publishes an empty message on `/zero_estimated_states`; the Kalman node captures the current raw position as its zero offset and subtracts it from every `/estimated_states` message it publishes thereafter. Because the re-zero happens *before* the data fans out, the browser plot, the LSL `EstimatedStates` outlet, and the evaluator all read exactly the same zeroed position — there is no per-consumer patching and no `zero_ack` round-trip. (The **actual**-side offset still lives in `web_visualizer`, since `/actual_states` is produced locally from the LSL inlet.)

A consequence worth remembering: the published `position` is now the *raw* encoder angle minus the zero, not the Kalman position estimate. Velocity and acceleration are still Kalman-filtered, and are unaffected by the constant offset (its derivative is zero).

### Why a Skip Iteration control?

The `pick_place` and `precision` experiments run several sub-steps (waypoints / trials). If the robot stalls or overshoots badly on one sub-step, the whole run would otherwise hang waiting for a settle that never comes. **Skip Iteration** lets the operator (or an automatic timeout, in precision) abandon the current sub-step and move on: the skipped step is recorded as a skip (excluded from the averaged error), and the next step's baseline is taken from where the robot actually is. The button only appears for the two multi-step experiments; single-shot experiments ignore it.

---

## Dependencies

### Python (pip, venv)
| Package      | Used by                                              |
|--------------|------------------------------------------------------|
| `pylsl`      | `web_visualizer.py`, `mock_robot_controller.py`, `mock_ui.py` |
| `websockets` | `web_visualizer.py`                                  |
| `numpy`      | `mock_encoder.py`, `Kalman_filter.py`                |
| `PyYAML`     | `mock_robot_controller.py`, `experiment_evaluator.py` |
| `catkin-pkg` | ROS 2 build tooling (required in venv)               |
| `empy`       | ROS 2 build tooling (required in venv)               |
| `lark`       | ROS 2 build tooling (required in venv)               |

### ROS 2
`rclcpp`, `rclpy`, `std_msgs`, `builtin_interfaces`, `rosidl_default_generators`

### Frontend (CDN)
uPlot 1.6.30 via jsdelivr

### System
`liblsl`: `sudo apt install liblsl-dev`
