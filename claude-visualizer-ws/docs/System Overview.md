# claude_visualizer — System Overview

A ROS 2 pipeline that streams encoder kinematics and robot-controller telemetry into a browser dashboard via **LSL (Lab Streaming Layer)** and a built-in **WebSocket** server. The encoder reader runs a constant-jerk Kalman filter for **velocity and acceleration**; the **position** it publishes is the raw encoder angle re-zeroed at the source (see *Homing / Zero* below). All development phases (1–7) are complete and running with a real Teensy 4.1 encoder. `mock_ui.py` and `mock_robot_controller.py` stand in for the robot controller during testing. Phase 6 adds multi-group isolation by ROS **namespace** (`group_number:=N` → `/G<N>/…`) so multiple robot-verifier groups can run on the same network; Phase 7 adds source-side homing (Zero), a Skip-Iteration control, a two-group precision experiment, and CSV export.

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
      │   ws-server (asyncio :9090)  [fixed]                         │
      │   http-server (:8000) serves web/ + /config.json  [fixed]    │
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
                               │ WebSocket :9090
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

| Stream name | Suffix (multi-group) | Direction | Channels | Rate | Producer |
|---|---|---|---|---|---|
| `ActualStates` | `ActualStates_N` | inlet on verifier | 3 float | 500 Hz | robot machine |
| `EventTrigger` | `EventTrigger_N` | inlet on verifier | 1 string | IRREGULAR | robot machine |
| `EstimatedStates` | `EstimatedStates_N` | outlet on verifier | 3 float | 500 Hz | `web_visualizer` |

Where `N` is the `group_number` (e.g. `5`). No suffix for group 0 / default. (ROS topics are separately namespaced under `/G<N>/`.)

---

## Components

### `encoder_reader` (`scripts/Kalman_filter.py`)
Constant-jerk Kalman filter (state `[position, velocity, acceleration, jerk]`, variable-dt transition). Subscribes `/encoder_raw`, publishes `/estimated_states`. **Velocity and acceleration** come from the filter; the published **position** is the raw encoder angle (`ticks × 2π/ticks_per_rev`) minus a zero offset. It also subscribes `/zero_estimated_states` (`std_msgs/Empty`): each message captures the current raw position as the new zero, re-homing `/estimated_states` at the source for every downstream consumer. `ticks_per_rev` defaults to 8192.

### `web_visualizer` (`scripts/web_visualizer.py`)
The bridge node. Runs 5 threads: ROS executor, asyncio WebSocket server, HTTP file server, LSL-ActualStates worker, LSL-EventTrigger worker. Bridges LSL streams into ROS topics and broadcasts everything to the browser as JSON over WebSocket. Also serves `/config.json` so the browser can discover the correct WebSocket port dynamically.

### `experiment_evaluator` (`scripts/experiment_evaluator.py`)
Evaluates encoder motion against configurable pass/fail criteria. Supports four experiment types: `point_to_point`, `pick_place`, `performance`, `precision`. Publishes live metrics and a final summary. Selects its criteria row from `criteria.yaml` by `group_number` (group 0 or any unlisted group uses `default`). Criteria can be edited live from the browser CRITERIA tab via the `update_criteria` service. Runs under the group namespace `/G<N>/`.

It also handles **Skip Iteration** (`action:"skip_iteration"`): drop the current pick_place waypoint or precision trial and advance. **Precision is a two-group experiment** — it measures both the init→target reach (*target group*) and the return→init move (*return group*), each reporting mean/std/max error and a pass flag; a stuck phase is auto-skipped after a time-to-halfway timeout. Its payload uses `init_pos` + `target_pos` (note: the CLI `mock_robot_controller` still sends `tar_pos` and is out of sync — use `mock_ui`).

### `mock_robot_controller` (`scripts/mock_robot_controller.py`)
Non-ROS CLI tool. Streams continuous waveform telemetry (trapezoid/sine/step) on LSL `ActualStates` and sends experiment commands on LSL `EventTrigger` from stdin. Use when the real robot controller is not available. Supports `--group-number` or `CV_GROUP_NUMBER` env for the multi-group LSL suffix (pure LSL — no ROS namespace).

### `mock_ui` (`mock_UI/mock_ui.py`)
Tkinter GUI combining a mock encoder knob and a mock robot controller knob. Publishes `/encoder_raw` and `/event_trigger` (ROS2) and both LSL streams. Use for full end-to-end testing without any hardware. Features: **bidirectional** Sync mode (either knob drives the other by rate of change), Fine/Rough sensitivity toggle, dual rad/deg readout, command entry field. Reads `ticks_per_rev` from `params.yaml` (default 8192) so it matches the KF/hardware, and its `prec` command sends the correct `target_pos` key. Supports `--group-number` or `CV_GROUP_NUMBER`: applies both the ROS node namespace `/G<N>/` (its `encoder_raw`/`event_trigger` topics) and the LSL suffix `_N`.

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
# With a specific group (namespaces the ROS graph under /G<N>/, LSL suffix _N, selects criteria):
ros2 launch claude_visualizer bringup.launch.py group_number:=5

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

### Multiple groups on one LAN

```bash
# --- Group 5 verifier ---
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch claude_visualizer bringup.launch.py group_number:=5
# ROS graph under /G5/…, LSL streams suffixed _5, web on fixed :9090/:8000

# --- Group 5 robot / mock (LSL source) ---
python3 src/claude_visualizer/scripts/mock_robot_controller.py --group-number 5
# Streams ActualStates_5 and EventTrigger_5 on LSL
# (or the GUI mock, which also namespaces its ROS topics under /G5/):
# python3 src/claude_visualizer/mock_UI/mock_ui.py --group-number 5

# Open browser: http://<verifier-ip>:8000   (each group is a different machine/IP)
# Verify: ros2 topic list  → /G5/encoder_raw, /G5/estimated_states, …
```

> **Build note:** `colcon build` is required after any `.py` script change — scripts are copied to install, not symlinked (`--symlink-install` conflicts with micro_ros_package). Web files also require a rebuild.

---

## Multi-Group Isolation — Concept and Reasoning

Isolation is by **ROS namespace**, keyed on a single `group_number = N`. (An earlier design used `pair_id` → `ROS_DOMAIN_ID` + per-group web ports; that port scheme is archived in `docs/Per-Group Port Configuration (archived).md`.) This section explains **why** the current design looks the way it does.

### Why is isolation needed at all?

In FRA263, multiple robot-verifier groups may run on the same lab network simultaneously — one per group. Without isolation, the system breaks in two silent ways:

1. **ROS 2 topics collide.**  
   ROS 2 auto-discovers other nodes on the same domain. If Group A and Group B both publish `/estimated_states`, the `experiment_evaluator` on A may evaluate B's encoder data — completely wrong results, no error message.

2. **LSL does not distinguish whose data is whose by default.**  
   When `web_visualizer` calls `_resolve_stream("ActualStates")`, it grabs the first LSL stream with that name on the LAN — possibly a different group's robot. This is the most dangerous contamination because it is silent.

### Why ROS namespaces (and not `ROS_DOMAIN_ID`)?

Each group's ROS graph lives under a **namespace** `/G<N>/`, so Group 5's topics are `/G5/encoder_raw`, `/G5/estimated_states`, … and Group 6's are `/G6/…`. Different names → no cross-talk, even on the same DDS domain.

Why not the old `ROS_DOMAIN_ID` approach? Because `ROS_DOMAIN_ID` must be set in the environment **before** the ROS daemon/DDS starts — it can't be a launch argument, which forced an awkward "source this env file in every terminal first" workflow. A **namespace is a plain launch argument** (`group_number:=N`): no pre-sourcing, no daemon-timing trap, and `ros2 topic list` shows a clean `/G<N>/` tree. So the env file and its generator were removed.

**How the namespace is applied (implementation).** ROS namespaces only prefix **relative** topic names, but the code originally used absolute names (`/encoder_raw`). So every topic string was made relative (`encoder_raw`), and the launch wraps all nodes in `GroupAction([PushRosNamespace("G<N>"), …])`. A relative `encoder_raw` under namespace `G5` resolves to `/G5/encoder_raw`; with the default group 0 it becomes `/G0/encoder_raw`. **There is always a namespace.**

### Why a single `group_number`?

One number sets everything, minimizing operator error:
- **ROS namespace** `/G<N>/` → isolates the ROS graph
- **LSL suffix** `_N` (e.g. `ActualStates_5`) → isolates cross-machine robot telemetry
- **criteria row** → picks the group's pass/fail thresholds from `criteria.yaml`

It is passed on the CLI: `group_number:=N` to the launch, `--group-number N` to the mocks (`CV_GROUP_NUMBER` env still works as a fallback for the mocks). No env file needed.

### Why LSL and not ROS for cross-machine communication?

The **entire ROS graph (incl. the Teensy micro-ROS agent) runs on the verifier machine.** The robot controller only emits LSL. So LSL is the only cross-machine transport; ROS/DDS never crosses between machines. That means the **namespace isolates the verifier's local graph**, and the **LSL suffix isolates the cross-machine link** — two different jobs, both driven by `group_number`.

### Why add `_N` to both the LSL stream name AND source_id?

LSL identifies a stream by `name` + `source_id`. Suffixing the **name** (`ActualStates_5`) makes `_resolve_stream("ActualStates_5")` return only Group 5's stream; suffixing the **source_id** too is a second guarantee if two streams ever shared a name. (This suffix behaviour is unchanged from the earlier `pair_id` design.)

### Why is group 0 special?

Default `group_number = 0` → namespace `/G0/` (there is always a namespace), but LSL streams stay **unsuffixed** (`ActualStates`, not `ActualStates_0`) and criteria falls back to `default`. Group 0 is the "single / default" setup; groups `1..N` get the `_N` suffix and their own criteria row.

### Why keep `/config.json` if the port is fixed?

The port is now fixed at 9090, so `/config.json` always answers `9090` — the same value `app.js` would hardcode. It is kept purely as indirection: the browser still discovers the port at runtime (via `fetch("/config.json")`) and connects to `ws://${location.hostname}:9090`, so it always targets whatever host served the page without any hardcoded IP.

### Backward compatibility

Running with no argument (`group_number = 0`) gives namespace `/G0/`, no LSL suffix, `default` criteria, and fixed ports 9090/8000 — the simplest single-setup case.

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
