Experiment Evaluator Node 
=========================

.. Node description (the design idea, detail, and other crucial info)

**Automatic Robot's Performance Evaluation**

This node is the judge of the Verification System. Before, a person had to watch the
logged data by eye and decide whether the robot performed well; the node
``experiment_evaluator`` does it automatically. It is **event-driven** (it does nothing
until a message tells it to act): it sits idle until a start event arrives on
``/event_trigger``, then measures every ``/estimated_states`` sample, publishes live
metrics while the run is going, and ends with one final **pass/fail summary**.

The node can evaluate the robot in **four test aspects**, and each aspect has its own
evaluation algorithm (explained one by one in the Node Workflow section):

1. ``point_to_point`` — move by a commanded distance; scored on settling time,
   overshoot, and final error.
2. ``pick_place`` — visit a sequence of hole positions; every waypoint is scored
   separately.
3. ``performance`` — reach a commanded speed and acceleration; scored on the peaks.
4. ``precision`` — repeat the same move many times; scored on accuracy (mean error)
   and precision (spread of the results).

.. code-block:: text

   [ from web_visualizer ]         ┌──────────────────────────┐       [ to web_visualizer ]
   /event_trigger  ──────────────▶ │                          │ ────▶ /eval_live      (pub, 10 Hz)
   (start / stop / skip events)    │   experiment_evaluator   │       live metrics during a run
                                   │                          │
   [ from State Estimator ]        │   4 test aspects,        │ ────▶ /eval_summary   (pub, once)
   /estimated_states ────────────▶ │   1 measuring loop       │       final pass/fail verdict
   (position, velocity, accel)     │                          │
                                   │                          │ ◀──▶ update_criteria (service server)
                                   └──────────────────────────┘       read / change limits live

**Parameters.**

.. list-table::
   :header-rows: 1
   :widths: 28 16 56

   * - Parameter
     - Value
     - Meaning
   * - ``group_number``
     - ``"0"``
     - Selects the group's row in ``criteria.yaml``. Declared with *dynamic typing*
       (accepts both int and string) because the launch file may pass ``5`` as an int
       while the standalone default is the string ``"0"``.
   * - ``criteria_file_path``
     - ``""``
     - Path to ``criteria.yaml``. Empty or missing → the node falls back to a
       hardcoded default set and warns.

**Criteria — the pass/fail limits.** All scoring compares the measured values against
six limits. The values below are the ``default`` row of ``criteria.yaml``; each group can
have its own row (keyed by ``group_number``):

.. list-table::
   :header-rows: 1
   :widths: 26 12 10 22 30

   * - Criterion
     - Default
     - Unit
     - Used by
     - Meaning
   * - ``min_speed``
     - ``3.0``
     - rad/s
     - performance
     - The peak speed must reach at least this (fallback when the payload gives no
       commanded speed).
   * - ``min_acceleration``
     - ``4.0``
     - rad/s²
     - performance
     - Same idea for the peak acceleration.
   * - ``max_avg_error_rad``
     - ``0.1``
     - rad
     - point_to_point, pick_place, precision
     - The largest allowed (average) position error.
   * - ``max_overshoot_pct``
     - ``10.0``
     - \% of travel
     - point_to_point, pick_place
     - How far past the target the robot may go.
   * - ``max_settling_time_s``
     - ``5.0``
     - s
     - point_to_point, pick_place
     - How long the robot may take to settle, timed from the first entry into the
       settling band.
   * - ``settling_band_pct``
     - ``3.0``
     - \% of travel
     - point_to_point, pick_place, precision
     - The width of the settling band (see the shared building block below).

There are **two ways to change the criteria**: (1) edit the group's row in
``criteria.yaml`` before launch, or (2) change them **live** from the browser's Criteria
tab — the web visualizer calls this node's ``update_criteria`` service, and the change
applies immediately without a restart.

**Units.** Test payloads carry a ``unit`` field. The helper ``_to_rad`` converts
``index`` (one hole on the 72-hole plate: 360/72 = **5°** ≈ 0.08727 rad) or ``degree``
into radians; anything else is assumed to be radians already.

**Interfaces.** Two subscriptions, two publishers, and one service — and each one has a
clear reason to exist:

- **Subscribes** ``/event_trigger`` (``EventTrigger``) — the node's *ears*. Every start,
  stop, and skip arrives here (from the Base System through the LSL bridge, or from the
  browser buttons). Without it the evaluator would never know a test began — it would sit
  idle forever.
- **Subscribes** ``/estimated_states`` (``EncoderState``) — the node's *measurement
  source*. Every metric it produces (error, overshoot, settling time, peaks, precision)
  is computed from these position/velocity/acceleration samples coming from the State
  Estimator.
- **Publishes** ``/eval_live`` (``ExperimentEval``, throttled to 10 Hz) — feeds the live
  evaluation panel in the browser, so the lecturer can watch the run progressing
  (current error, waypoint counter, phase reference). It is throttled (deliberately
  rate-limited) so the UI is not flooded with hundreds of messages per second.
- **Publishes** ``/eval_summary`` (``ExperimentEval``, once per run) — the final verdict.
  This single message with the pass/fail result is the product the whole Verification
  System exists to deliver; the UI displays it as the run's result.
- **Serves** ``update_criteria`` (``UpdateCriteria``) — the service **server** (the
  answering side; the web visualizer holds the matching client). It lets the lecturer
  read and change the pass/fail limits live from the browser, without restarting the
  node or editing the file mid-demo.

Both ``ExperimentEval`` messages carry the same shape: an ``action`` string naming the
test aspect, and a ``data`` string holding the metrics as JSON. As in the other nodes,
all names are **relative** (the launch namespace ``/G<N>/`` is added in front) and the
QoS profile is **RELIABLE** with **KEEP_LAST depth 10**.

Node Workflow
-------------

.. The flow chart of this whole node

Unlike the web visualizer, this node has no extra threads — everything runs in the two
subscription callbacks. What plays the role of the "workers" here are the **four
evaluation algorithms** that share one measuring loop. This section starts with the big
picture, then one shared building block, then each test aspect with its own algorithm.

The big picture
^^^^^^^^^^^^^^^

.. code-block:: text

   [callback: _event_trigger_cb — the router]
   /event_trigger ──▶ parse the JSON payload
        ├─ mode "STOP" / action "stop"      ──▶ finish the run now (publish the summary)
        ├─ action "skip_iteration"          ──▶ skip the current waypoint / trial
        ├─ action is one of the 4 aspects   ──▶ reset all state, start measuring
        └─ anything else                    ──▶ warn and ignore

   ─────────────────────────────────────────────────────────────────────────────────

   [callback: _estimated_states_cb — the measuring loop, runs once per sample]
   /estimated_states ──▶ no active run? ──▶ ignore the sample
        │ a run is active
        ▼
   first sample of this run? ──▶ store the start position + start time
        ▼
   append (position, velocity, acceleration) to the sample list
        ▼
   dispatch to the active aspect's update function:
        point_to_point → _update_ptp        pick_place → _update_pp
        performance    → _update_perf       precision  → _update_prec
        ▼
   ≥ 0.1 s since the last live message? ──▶ publish /eval_live   (throttle: 10 Hz)

The whole life of one experiment is then:

.. code-block:: text

   idle ──start event──▶ measuring ──(settled / last waypoint / repeat reached / STOP)──▶
                          publish /eval_summary ──▶ back to idle

A run can end **by itself** (point-to-point settles, the last waypoint settles, the
precision cycles are complete) or **by the user** (the STOP button). Either way the
summary is computed from whatever was collected up to that moment.

Shared building block: the settling band
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Three of the four aspects (all except ``performance``) decide "the robot has arrived"
the same way, so the rule is explained once here. The **settling band** is a small error
window around the target:

.. code-block:: text

   band = max(0.01 rad, settling_band_pct % × |travel|)

   every sample:  error = |position − target|

   error < band ? ──no──▶ reset the 0.5 s counter (left the band → not settled)
        │ yes
        ▼
   first time ever inside the band? ──▶ remember this moment (the settling clock starts)
        ▼
   inside the band continuously for ≥ 0.5 s ?
        │ yes
        ▼
   SETTLED — settling time = now − first band entry

Two details to notice. The band scales with the **travel distance** (3 % of a long move
is wider than 3 % of a short one), but never gets thinner than the **0.01 rad floor** —
otherwise a tiny move would create an impossibly thin band. And the settling time is
counted from the **first** band entry, not the last: if the robot enters the band,
bounces out, and comes back, that bouncing time counts against it. The 0.5 s continuous
stay (the *settling window*) is what separates "passing through the target" from
"actually stopped there".

Point-to-Point test
^^^^^^^^^^^^^^^^^^^

**Objective:** move by a commanded distance and stop there — the basic positioning test.
The payload gives the distance: ``{"action": "point_to_point", "value": <number>,
"unit": "index" | "degree" | ...}``.

.. code-block:: text

   start event ──▶ target = start position + travel      (travel = value → rad)
                   band   = max(0.01 rad, settling_band_pct % × |travel|)

   every sample:
        │
        ▼
   error = |position − target|  <  band ?  ──no──▶ reset the continuous counter
        │ yes
        ▼
   settled? (continuously in band ≥ 0.5 s)  ──no──▶ keep counting
        │ yes
        ▼
   settling time = now − first band entry
        │
        ▼
   AUTO-FINISH ──▶ /eval_summary

The run finishes **by itself** the moment the robot settles. If the robot never settles,
only the STOP button ends it — the summary then reports no settling time and fails that
criterion.

**Summary and pass rules.** The final error is taken from the last sample; the overshoot
is found by scanning all samples for the farthest point **past the target in the travel
direction**, given as a percentage of the travel:

- ``pass_final_error`` — final error ≤ ``max_avg_error_rad``
- ``pass_overshoot`` — settled **and** overshoot ≤ ``max_overshoot_pct``
- ``pass_settling`` — settling time ≤ ``max_settling_time_s``

The live message (10 Hz) carries the target, current position, current error, and the
elapsed time.

Pick-and-Place test
^^^^^^^^^^^^^^^^^^^

**Objective:** visit a sequence of holes in order — the multi-target test. The payload
gives the plan: ``{"action": "pick_place", "order_sequence": [i₁, i₂, ...]}`` where each
entry is a **hole index** (target = index × 5°). Every waypoint (one target in the
sequence) is scored on its own:

.. code-block:: text

   for one waypoint at a time:
        target = index × RAD_PER_INDEX
        travel = target − previous target     (first waypoint: from the start position)
        band   = max(0.01 rad, settling_band_pct % × |travel|)
        │
        ▼
   robot enters the band for the FIRST time
        ├─ the settling clock starts
        └─ peak tracking starts               (for the overshoot, direction-aware)
        │
        ▼
   settled? (continuously in band ≥ 0.5 s)
        │ yes
        ▼
   score the waypoint:  final error │ overshoot % (from the tracked peak) │ settling time
                        + 3 pass flags against the criteria
        │
        ├─ more waypoints left ──▶ baseline = this target ; next waypoint
        └─ last waypoint       ──▶ AUTO-FINISH ──▶ /eval_summary

   ─────────────────────────────────────────────────────────────────────────────────

   [Skip Iteration] pressed while a waypoint is stuck:
        record it as skipped  (all pass flags = false, excluded from the average error)
        baseline for the next waypoint = the robot's ACTUAL current position
                                         (not the unreached target)

Two design details. The **travel baseline** of each waypoint is the *previous target*,
so the band and overshoot are measured over the intended move. But after a **skip**, the
baseline becomes the robot's actual current position — the robot never reached the old
target, so measuring the next move from a position the robot never held would distort
its band and overshoot. And the peak tracking is **direction-aware**: for a positive
travel the peak is the maximum position, for a negative travel the minimum, so
"overshoot" always means "past the target", whichever way the robot moves.

**Summary.** Total waypoints, the average error over the *scored* (non-skipped)
waypoints with its pass flag, the passed / failed / skipped counts, and the full
per-waypoint details list. The live message carries the current waypoint number, total,
target, current error, and elapsed time.

Performance test
^^^^^^^^^^^^^^^^

**Objective:** show that the robot can actually reach the speed and acceleration it was
commanded — the raw capability test. This is the simplest algorithm of the four:

.. code-block:: text

   every sample:   peak_speed = max(peak_speed, |velocity|)
                   peak_accel = max(peak_accel, |acceleration|)

   ...runs until the user presses STOP ──▶ /eval_summary
        pass_speed = peak_speed ≥ commanded speed
        pass_accel = peak_accel ≥ commanded acceleration

There is no settling and no auto-finish — the node just remembers the highest absolute
speed and acceleration it has seen, and the user decides when the demonstration is over.
The commanded values come from the payload (``speed``, ``accel``); if the payload does
not give them, the criteria minimums (``min_speed``, ``min_acceleration``) are used as
the fallback. Note the comparison direction: unlike the error limits, these are
**minimums** — the peak must be *at least* the commanded value.

The live message carries both peaks, the current speed, and the elapsed time.

Precision test
^^^^^^^^^^^^^^

**Objective:** repeat the same move many times and measure how *repeatable* the robot is.
One run = ``repeat`` cycles of **init → target → init**. The payload:
``{"action": "precision", "init_pos": ..., "target_pos": ..., "unit": ..., "repeat": N}``.

Each cycle has **two phases**, and each phase collects one settled position:

.. code-block:: text

   band          = max(0.01 rad, settling_band_pct % × travel)
   counting band = 50 % × travel        (used only by the auto-skip timeout)

   ┌──────────────────────────── one cycle ────────────────────────────┐

   PHASE 1 — approach (init → target)
        settle at the target (0.5 s in band)
             ──▶ record the settled position into the TARGET group
             ──▶ flip to phase 2
        auto-skip: the robot crossed the 50 % counting band but never settles,
        and the phase has run longer than
             2 × (time it took to reach 50 %) + 1 s buffer
             ──▶ count a target skip, flip to phase 2 anyway
        │
        ▼
   PHASE 2 — return (target → init)
        settle at init (0.5 s in band)
             ──▶ record the settled position into the RETURN group
             ──▶ one full cycle done, flip back to phase 1
        auto-skip: same timeout rule, mirrored toward init
             ──▶ count a return skip, flip back to phase 1
        │
        ▼
   returns done + returns skipped ≥ repeat ?  ──no──▶ next cycle
        │ yes
        ▼
   FINISH ──▶ /eval_summary

Three design details:

- **The auto-skip timeout** protects the run from a stuck robot without any fixed magic
  number. When the robot crosses the halfway point (the 50 % *counting band*), the node
  stamps that time. A healthy move should finish in roughly twice the time it took to
  cover the first half — so if the phase runs past *2 × time-to-halfway + 1 s buffer*
  without settling, the node force-skips the phase by itself. The manual **Skip
  Iteration** button funnels into the same function (``_skip_trial``), which picks the
  right skip counter from the *current* phase before flipping it.
- **Termination lives in the return phase**: the run ends when *returns done + returns
  skipped* reaches ``repeat``. Ending on the return (not the approach) means the final
  trial still drives back to init — the robot always finishes the run at home.
- **The live reference follows the active phase**: while approaching, the live message's
  ``target_rad`` is the target; while returning, it is init. The amber reference line in
  the browser therefore tracks what the robot is *currently* trying to reach, with no
  extra logic on the browser side.

**Summary.** Two groups, scored with the same rule: the **target group** (all settled
positions at the target — the init→target reaching performance) and the **return group**
(all settled positions at init — the returning performance). Each group reports the
number of trials and skips, the **mean error** (accuracy — how close on average), the
**standard deviation** (precision — how repeatable), the max error, and a pass flag
(all ``repeat`` trials done and mean error ≤ ``max_avg_error_rad``).

Examine the code
----------------

.. referencing the section 2.1 of the mentioned link

The full node lives in ``scripts/experiment_evaluator.py`` (about 775 lines). As on the
web-visualizer page, the walkthrough is complete but condensed: commented-out legacy
lines are trimmed from the excerpts, while the meaningful comments and docstrings stay.

**Imports and constants.**

.. code-block:: python

   import json
   import math
   import os

   import rclpy
   from rclpy.node import Node
   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
   from rcl_interfaces.msg import ParameterDescriptor
   import yaml

   from claude_visualizer_interface.msg import EventTrigger, EncoderState, ExperimentEval
   from claude_visualizer_interface.srv import UpdateCriteria


   # ── constants ────────────────────────────────────────────────────────────────
   N_HOLES              = 72
   DEG_PER_INDEX        = 360.0 / N_HOLES                     # [deg/index]
   RAD_PER_INDEX        = DEG_PER_INDEX * math.pi / 180.0     # [rad/index] ≈ 0.08727

   SETTLING_THRESHOLD_rad = 0.01   # hard floor for settling band  [rad]
   SETTLING_WINDOW_s      = 0.5    # continuous in-band time to declare settled  [s]
   LIVE_PUB_INTERVAL_s    = 0.1    # throttle period for /eval_live  [s]


   def _to_rad(value, unit: str) -> float:
       unit = unit.lower()
       if unit == "index":
           return float(value) * RAD_PER_INDEX
       if unit == "degree":
           return float(value) * math.pi / 180.0
       return float(value)   # assume already radians

``yaml`` reads the criteria file and ``json`` parses the event payloads and packs the
result messages. The constants are the fixed rules of the evaluation: the plate geometry
(72 holes → 5° per index), the settling floor and window from the shared building block,
and the 0.1 s live-publish throttle. ``_to_rad`` is the single place where payload units
become radians — every algorithm below works in radians only.

**Constructor.**

.. code-block:: python

   class ExperimentEvaluator(Node):
       def __init__(self):
           super().__init__("experiment_evaluator")

           # group_number: launch passes it via LaunchConfiguration; "5" is
           # type-inferred to INT, while the default/standalone case is STR — dynamic_typing
           # accepts either (the criteria loader normalizes with str()).
           self.declare_parameter(
               "group_number", "0", ParameterDescriptor(dynamic_typing=True)
           )
           self.declare_parameter("criteria_file_path", "")

           group_number  = self.get_parameter("group_number").value
           criteria_file = self.get_parameter("criteria_file_path").value

           self._criteria = self._load_criteria(criteria_file, group_number)
           self.get_logger().info(
               f"[ExperimentEvaluator] group_number={group_number!r}  criteria={self._criteria}"
           )

           reliable_qos = QoSProfile(
               reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST,
               depth=10,
           )

           # Relative topic/service names so the node's namespace (/G<N>) is prepended.
           self._event_sub = self.create_subscription(
               EventTrigger, "event_trigger", self._event_trigger_cb, reliable_qos
           )
           self._states_sub = self.create_subscription(
               EncoderState, "estimated_states", self._estimated_states_cb, reliable_qos
           )

           self._live_pub    = self.create_publisher(ExperimentEval, "eval_live",    reliable_qos)
           self._summary_pub = self.create_publisher(ExperimentEval, "eval_summary", reliable_qos)

           self._update_criteria_srv = self.create_service(
               UpdateCriteria, "update_criteria", self._update_criteria_cb
           )

           self._reset_state()

The interesting parameter detail is ``dynamic_typing=True``: a ROS parameter normally has
one fixed type, but ``group_number`` can arrive as an **int** (the launch file passes
``5``) or as a **string** (the standalone default ``"0"``). Dynamic typing accepts both,
and the criteria loader normalizes with ``str()``. The rest is the usual wiring — two
subscriptions, two publishers, and the service server — and one call to
``_reset_state()`` so all experiment variables exist before the first event.

**Loading the criteria —** ``_load_criteria``.

.. code-block:: python

   def _load_criteria(self, path: str, group_number: str) -> dict:
       default = {
           "min_speed":           0.5,
           "min_acceleration":    1.0,
           "max_avg_error_rad":   0.1,
           "max_overshoot_pct":   20.0,
           "max_settling_time_s": 5.0,
           "settling_band_pct":   3.0,
       }
       if not path or not os.path.isfile(path):
           self.get_logger().warn(
               f"criteria_file_path not found ({path!r}); using defaults"
           )
           return default
       with open(path, "r") as f:
           data = yaml.safe_load(f)
       # Normalize keys to strings: YAML parses bare `11:` as an int, but group_number
       # arrives from the ROS param. group_number 0 / unlisted → 'default'.
       table = {str(k): v for k, v in data.get("criteria", {}).items()}
       key = str(group_number)
       if key in table:
           return table[key]
       self.get_logger().warn(
           f"group_number {group_number!r} not in criteria table; using 'default'"
       )
       return table.get("default", default)

Three layers of fallback, so the node always has usable limits: the group's own row → the
file's ``default`` row → the hardcoded ``default`` dict (when the file is missing). The
key normalization comment explains a real YAML trap: a bare ``11:`` in YAML is parsed as
an *integer* key, but the ROS parameter may deliver the string ``"11"`` — converting both
sides with ``str()`` makes them match.

**Changing the criteria live —** ``_update_criteria_cb``.

.. code-block:: python

   def _update_criteria_cb(self, request, response):
       try:
           updates = json.loads(request.criteria_json)
       except json.JSONDecodeError as e:
           response.success = False
           response.message = f"Invalid JSON: {e}"
           return response

       valid_criteria = {}
       for key, val in updates.items():
           if key not in self._criteria:
               response.success = False
               response.message = f"Unknown Key: {key}"
               return response

           if not isinstance(val, (int, float)) or val < 0:
               response.success = False
               response.message = f"Invalid value for {key}: {val}"
               return response

           valid_criteria[key] = float(val)

       self._criteria.update(valid_criteria)
       updated_keys = ", ".join(valid_criteria.keys())
       updated_values = ", ".join(str(v) for v in valid_criteria.values())
       self.get_logger().info(f"[criteria] Updated: {updated_keys} with value: {updated_values}")
       response.success = True
       response.message = f"Updated: {updated_keys} with value: {updated_values}"
       response.current_criteria_json = json.dumps(self._criteria)

       return response

This is the server side of the service the web visualizer calls. The validation is
**all-or-nothing**: every key must already exist in the criteria and every value must be
a non-negative number; the checked values are collected into ``valid_criteria`` first and
applied only after *everything* passed. One bad entry rejects the whole request, so the
criteria can never end up half-updated. The response always carries the full current
criteria as JSON — that is what the browser's Criteria tab displays. A request with an
empty ``{}`` changes nothing and just returns the current criteria (the web visualizer
uses exactly this trick for its snapshot).

**Fresh state for every run —** ``_reset_state``.

.. code-block:: python

   def _reset_state(self):
       # core
       self._active_action: str | None       = None
       self._payload: dict                   = {}
       self._start_pos_rad: float | None     = None
       self._start_time_s: float | None      = None
       self._last_live_time_s: float         = 0.0
       self._last_pos_rad: float | None      = None  # latest /estimated_states position
       self._samples: list[tuple[float, float, float]] = []  # (pos_rad, vel_rad_s, accel_rad_s2)
       self._t_start: float                  = 0.0

       # ptp settling
       self._settling_time_s: float | None   = None
       self._first_band_entry_s: float | None = None
       self._cont_band_entry_s: float | None  = None

       # pick_place
       self._wp_idx: int                          = 0
       self._wp_results: list[dict]               = []
       self._wp_first_band_entry_s: float | None  = None
       self._wp_cont_band_entry_s: float | None   = None
       self._wp_reached_target: bool              = False
       self._wp_peak_pos_rad: float | None        = None
       self._wp_prev_target_rad: float | None     = None

       # precision
       self._trial_positions_rad: list[float]     = []
       self._prec_est_terminating_t_s: float      = None
       self._buffer_reach_t_s: float              = 1.0
       self._prec_at_target: bool                 = False
       self._prec_cont_band_entry_s: float | None = None
       self._prec_skipped: int                    = 0
       #start time of the return-to-init phase, used for its own force-skip timeout
       self._prec_return_t_start_s: float | None  = None
       #return-to-init (group 2) settled positions, its skip count, and half-band cross timestamp
       self._return_positions_rad: list[float]    = []
       self._prec_return_skipped: int             = 0
       self._prec_return_est_t_s: float | None    = None

       # performance
       self._peak_speed_rad_s: float  = 0.0
       self._peak_accel_rad_s2: float = 0.0

This is the node's memory for one experiment, grouped by aspect — you can read the four
algorithms' needs straight from it: point-to-point needs the two band-entry timestamps,
pick-place needs a waypoint cursor and per-waypoint results, precision needs both phase
groups with their skip counters and timeout stamps, performance needs only the two peaks.
``_start_experiment`` calls this first, so **no value from the previous run can leak into
the next one** — that is why a fresh reset per run matters.

**The router —** ``_event_trigger_cb``.

.. code-block:: python

   def _event_trigger_cb(self, msg: EventTrigger):
       try:
           payload = json.loads(msg.event)
       except (TypeError, ValueError):
           self.get_logger().error(f"[EventTrigger] invalid JSON: {msg.event!r}")
           return

       action = payload.get("action", "")
       mode   = payload.get("mode",   "")

       if mode == "STOP" or action == "stop":
           if self._active_action is not None:
               self.get_logger().info("[Evaluator] STOP received — finishing experiment")
               self._finish_experiment()
           return

       if action == "skip_iteration":
           if self._active_action is not None:
               self._skip_iteration()
           return

       if action in ("point_to_point", "pick_place", "performance", "precision"):
           self._start_experiment(action, payload)
       else:
           self.get_logger().warn(f"[Evaluator] unknown action {action!r}, ignoring")

Every event goes through this one router. The order matters: STOP and skip are checked
first and only act when a run is active; a recognized test aspect starts a new run
(starting while another run is active simply replaces it — ``_start_experiment`` resets
everything); anything unknown is logged and ignored, so a typo in an event can never
crash the judge.

**The lifecycle pair —** ``_start_experiment`` **and** ``_finish_experiment``.

.. code-block:: python

   def _start_experiment(self, action: str, payload: dict):
       self._reset_state()
       self._active_action = action
       self._payload       = payload
       self._t_start       = self.get_clock().now().nanoseconds * 1e-9
       self.get_logger().info(f"[Evaluator] started experiment: {action}")

   def _finish_experiment(self):
       action = self._active_action
       if action is None:
           return
       summary = self._compute_summary(action)
       self._active_action = None
       msg = ExperimentEval()
       msg.header.stamp = self.get_clock().now().to_msg()
       msg.action       = action
       msg.data         = json.dumps(summary)
       self._summary_pub.publish(msg)
       self.get_logger().info(f"[Evaluator] summary: {msg.data}")

Start = wipe the state, remember which aspect is active and its payload, stamp the start
time. Finish = the mirror: note that the summary is computed **before**
``_active_action`` is cleared — the summary functions read the collected state, so the
order matters. After the summary is published the node is idle again, back to waiting for
the next event.

**The measuring loop —** ``_estimated_states_cb``.

.. code-block:: python

   def _estimated_states_cb(self, msg: EncoderState):
       if self._active_action is None:
           return

       pos_rad      = msg.position
       vel_rad_s    = msg.velocity
       accel_rad_s2 = msg.acceleration
       now_s        = self.get_clock().now().nanoseconds * 1e-9
       self._last_pos_rad = pos_rad

       if self._start_pos_rad is None:
           self._start_pos_rad = pos_rad
           self._start_time_s  = now_s

       elapsed_s = now_s - self._start_time_s
       self._samples.append((pos_rad, vel_rad_s, accel_rad_s2))

       action = self._active_action
       if action == "point_to_point":
           self._update_ptp(pos_rad, elapsed_s)
       elif action == "pick_place":
           self._update_pp(pos_rad, now_s)
       elif action == "performance":
           self._update_perf(vel_rad_s, accel_rad_s2)
       elif action == "precision":
           self._update_prec(pos_rad, now_s)

       if self._active_action is None:
           return  # experiment auto-finished during update

       # throttled live publish
       if now_s - self._last_live_time_s >= LIVE_PUB_INTERVAL_s:
           self._last_live_time_s = now_s
           live = self._compute_live(action, pos_rad, vel_rad_s, accel_rad_s2, elapsed_s)
           m = ExperimentEval()
           m.header.stamp = self.get_clock().now().to_msg()
           m.action       = action
           m.data         = json.dumps(live)
           self._live_pub.publish(m)

The shared skeleton from the big-picture diagram. Two guards are worth a close look. The
first line makes the node cheap when idle — samples arrive all the time, but without an
active run they are dropped immediately. The second guard, *after* the dispatch, handles
a subtle case: an update function may finish the experiment mid-callback (for example
point-to-point settling on this very sample). ``_finish_experiment`` clears
``_active_action``, so without this check the node would publish one more live message
for a run that is already over. The live publish itself is throttled: it runs only when
0.1 s has passed since the last one, turning the 500 Hz sample stream into a 10 Hz UI
feed.

**Point-to-point —** ``_update_ptp``.

.. code-block:: python

   def _update_ptp(self, pos_rad: float, elapsed_s: float):
       target_rad = self._start_pos_rad + _to_rad(self._payload["value"], self._payload["unit"])
       travel_rad = target_rad - self._start_pos_rad
       band_rad   = max(SETTLING_THRESHOLD_rad,
                        self._criteria["settling_band_pct"] / 100.0 * abs(travel_rad))
       error_rad  = abs(pos_rad - target_rad)

       if error_rad < band_rad:
           if self._first_band_entry_s is None:
               self._first_band_entry_s = elapsed_s
           if self._cont_band_entry_s is None:
               self._cont_band_entry_s = elapsed_s
           if (self._settling_time_s is None
                   and elapsed_s - self._cont_band_entry_s >= SETTLING_WINDOW_s):
               self._settling_time_s = elapsed_s - self._first_band_entry_s
               self._finish_experiment()
       else:
           self._cont_band_entry_s = None

The settling rule from the shared building block, in code. The two timestamps do
different jobs: ``_first_band_entry_s`` is set **once** (the settling clock) while
``_cont_band_entry_s`` is reset every time the robot leaves the band (the 0.5 s
continuity counter). When the continuous stay reaches the window, the settling time is
computed from the *first* entry — bounce-outs are included — and the run auto-finishes.

**Pick-and-place —** ``_update_pp`` **and** ``_settle_waypoint``.

.. code-block:: python

   def _update_pp(self, pos_rad: float, now_s: float):
       payload    = self._payload
       sequence   = payload["order_sequence"]
       target_rad = sequence[self._wp_idx] * RAD_PER_INDEX

       if self._wp_prev_target_rad is None:
           self._wp_prev_target_rad = self._start_pos_rad

       travel_rad = target_rad - self._wp_prev_target_rad
       band_rad   = max(SETTLING_THRESHOLD_rad,
                        self._criteria["settling_band_pct"] / 100.0 * abs(travel_rad))
       error_rad  = abs(pos_rad - target_rad)

       # First time entering the settling band → start approach tracking
       if error_rad < band_rad and not self._wp_reached_target:
           self._wp_reached_target     = True
           self._wp_peak_pos_rad       = pos_rad
           self._wp_first_band_entry_s = now_s
           self._wp_cont_band_entry_s  = now_s

       # Peak tracking and settling (only after robot first reaches target)
       if self._wp_reached_target:
           if travel_rad >= 0:
               self._wp_peak_pos_rad = max(self._wp_peak_pos_rad, pos_rad)
           else:
               self._wp_peak_pos_rad = min(self._wp_peak_pos_rad, pos_rad)

           if error_rad < band_rad:
               if self._wp_cont_band_entry_s is None:
                   self._wp_cont_band_entry_s = now_s
               if now_s - self._wp_cont_band_entry_s >= SETTLING_WINDOW_s:
                   self._settle_waypoint(target_rad, travel_rad, error_rad, now_s)
           else:
               self._wp_cont_band_entry_s = None

The same settling machinery as point-to-point, plus two waypoint-specific pieces: the
travel baseline is the **previous target** (``_wp_prev_target_rad``), and the **peak
tracking** starts at the first band entry. The peak is direction-aware — ``max`` for a
forward move, ``min`` for a backward one — so the overshoot is always measured *past* the
target regardless of direction.

.. code-block:: python

   def _settle_waypoint(
       self, target_rad: float, travel_rad: float, final_error_rad: float, now_s: float
   ):
       wp_settling_time_s = now_s - (self._wp_first_band_entry_s or now_s)

       if self._wp_peak_pos_rad is not None and abs(travel_rad) > 1e-9:
           if travel_rad >= 0:
               overshoot_rad = max(0.0, self._wp_peak_pos_rad - target_rad)
           else:
               overshoot_rad = max(0.0, target_rad - self._wp_peak_pos_rad)
           overshoot_pct = overshoot_rad / abs(travel_rad) * 100.0
       else:
           overshoot_pct = 0.0

       c = self._criteria
       self._wp_results.append({
           "waypoint":          self._wp_idx + 1,
           "target_rad":        round(target_rad, 5),
           "final_error_rad":   round(final_error_rad, 5),
           "overshoot_pct":     round(overshoot_pct, 3),
           "settling_time_s":   round(wp_settling_time_s, 4),
           "pass_error":        final_error_rad    <= c["max_avg_error_rad"],
           "pass_overshoot":    overshoot_pct      <= c["max_overshoot_pct"],
           "pass_settling":     wp_settling_time_s <= c["max_settling_time_s"],
       })
       self.get_logger().info(
           f"[Evaluator] pick_place waypoint {self._wp_idx + 1} settled"
       )

       if self._wp_idx == len(self._payload["order_sequence"]) - 1:
           self._finish_experiment()
           return

       # Advance to next waypoint and reset per-waypoint state
       self._wp_prev_target_rad    = target_rad
       self._wp_idx               += 1
       self._wp_first_band_entry_s = None
       self._wp_cont_band_entry_s  = None
       self._wp_reached_target     = False
       self._wp_peak_pos_rad       = None

One settled waypoint becomes one result dictionary with its three pass flags — the
browser's details table is exactly this list. After the last waypoint the run
auto-finishes; otherwise the baseline moves to the settled target and all per-waypoint
tracking is reset for the next one.

**Manual skip —** ``_skip_iteration`` **and** ``_skip_waypoint``.

.. code-block:: python

   def _skip_iteration(self):
       """Manually skip the current iteration and advance to the next one.

       Only pick_place and precision have multiple iterations; other actions are
       single-shot, so skipping is a no-op there.
       """
       if self._active_action == "pick_place":
           self._skip_waypoint()
       elif self._active_action == "precision":
           self._skip_trial()
       else:
           self.get_logger().warn(
               f"[Evaluator] skip_iteration ignored for action {self._active_action!r}"
           )

.. code-block:: python

   def _skip_waypoint(self):
       """Drop the current (unsettled) waypoint and advance to the next one. ..."""
       sequence   = self._payload["order_sequence"]
       target_rad = sequence[self._wp_idx] * RAD_PER_INDEX
       cur_pos    = self._last_pos_rad
       if cur_pos is None:
           cur_pos = (self._wp_prev_target_rad if self._wp_prev_target_rad is not None
                      else (self._start_pos_rad or 0.0))

       self._wp_results.append({
           "waypoint":        self._wp_idx + 1,
           "target_rad":      round(target_rad, 5),
           "final_error_rad": round(abs(cur_pos - target_rad), 5),
           "overshoot_pct":   0.0,
           "settling_time_s": None,
           "skipped":         True,
           "pass_error":      False,
           "pass_overshoot":  False,
           "pass_settling":   False,
       })
       self.get_logger().info(
           f"[Evaluator] pick_place waypoint {self._wp_idx + 1} SKIPPED"
       )

       if self._wp_idx == len(sequence) - 1:
           self._finish_experiment()
           return

       # Advance; baseline = actual current position (NOT the unreached target).
       self._wp_prev_target_rad    = cur_pos
       self._wp_idx               += 1
       self._wp_first_band_entry_s = None
       self._wp_cont_band_entry_s  = None
       self._wp_reached_target     = False
       self._wp_peak_pos_rad       = None

A skipped waypoint is not silently dropped — it is recorded as a failed result marked
``"skipped": True`` so the details table shows what happened, but the summary excludes it
from the average error (a stuck waypoint should not poison the score of the others). The
last comment carries the key idea: the next waypoint's baseline is the robot's **actual**
position, because the robot never reached the target it just skipped.

**Performance —** ``_update_perf``.

.. code-block:: python

   def _update_perf(self, vel_rad_s: float, accel_rad_s2: float):
       self._peak_speed_rad_s  = max(self._peak_speed_rad_s,  abs(vel_rad_s))
       self._peak_accel_rad_s2 = max(self._peak_accel_rad_s2, abs(accel_rad_s2))

Two lines — the whole algorithm. A running maximum of the absolute speed and
acceleration. Everything else about this aspect (the STOP-only finish, the commanded
values, the ≥ comparison) lives in the summary function below.

**Precision —** ``_update_prec``, phase 1 (approach).

.. code-block:: python

   def _update_prec(self, pos_rad: float, now_s: float):
       payload  = self._payload
       tar_rad  = _to_rad(payload["target_pos"],  payload["unit"])
       init_rad = _to_rad(payload["init_pos"], payload["unit"])
       travel_rad = abs(tar_rad - init_rad)
       band_rad   = max(SETTLING_THRESHOLD_rad,
                        self._criteria["settling_band_pct"] / 100.0 * travel_rad)
       counting_band_rad = 50.0/100.0 * travel_rad

       if not self._prec_at_target:
           # ── init→target approach (group 1) ── waiting to settle at target position
           if abs(pos_rad - tar_rad) < band_rad:
               if self._prec_cont_band_entry_s is None:
                   self._prec_cont_band_entry_s = now_s
               if now_s - self._prec_cont_band_entry_s >= SETTLING_WINDOW_s:
                   self._trial_positions_rad.append(pos_rad)
                   #flip into the return phase; termination no longer happens here so the
                   #final trial also drives back to init before the run finishes
                   self._prec_at_target           = True
                   self._prec_cont_band_entry_s   = None
                   self._prec_est_terminating_t_s = None
                   #fresh timers for the upcoming return phase (lazily re-stamped below)
                   self._prec_return_t_start_s    = None
                   self._prec_return_est_t_s      = None

           #if the robot pass the trial counting_band and still not reach the target fall in this case
           elif (abs(pos_rad - tar_rad) < counting_band_rad) and self._prec_cont_band_entry_s is None:
               #if it is the first entry to this condition => collect the timestamp
               if self._prec_est_terminating_t_s is None:
                   self._prec_est_terminating_t_s = now_s
               #if the time used is more than expected reach time we consider to force skip the trial
               #NOTE: The reach time is calculated by double the time that use for reaching the 50% of the total distance and adding some buffer
               if now_s - self._t_start >= ((self._prec_est_terminating_t_s - self._t_start)*2.0 + self._buffer_reach_t_s):
                   #force skip the trial; _skip_trial() counts the target skip and flips the phase
                   self._prec_est_terminating_t_s = None
                   self._skip_trial()

           else:
               self._prec_cont_band_entry_s = None

The approach phase has three branches, checked in order. Inside the settling band: the
familiar 0.5 s rule — on success the settled position joins the **target group** and the
phase flips to the return. Inside the 50 % *counting band* but not the settling band:
this is the auto-skip watch — the first crossing stamps ``_prec_est_terminating_t_s``,
and when the elapsed time exceeds *2 × time-to-halfway + buffer*, the phase is
force-skipped. Outside both: the continuity counter resets, same as every other aspect.

**Precision —** ``_update_prec``, phase 2 (return).

.. code-block:: python

       else:
           # ── return→init (group 2) ── must settle at init, and it is now MEASURED
           #lazily stamp the return-phase start on its first frame (its own skip timeout ref)
           if self._prec_return_t_start_s is None:
               self._prec_return_t_start_s = now_s

           if abs(pos_rad - init_rad) < band_rad:
               if self._prec_cont_band_entry_s is None:
                   self._prec_cont_band_entry_s = now_s
               if now_s - self._prec_cont_band_entry_s >= SETTLING_WINDOW_s:
                   #record the return-to-init settled position, then flip back to approach
                   self._return_positions_rad.append(pos_rad)
                   self._prec_at_target         = False
                   self._prec_cont_band_entry_s = None
                   self._prec_return_est_t_s    = None
                   self._prec_return_t_start_s  = None
                   #termination lives here: finish once `repeat` full cycles are done
                   if (len(self._return_positions_rad) + self._prec_return_skipped
                           >= payload["repeat"]):
                       self._finish_experiment()
                       return

           #gap 3: robot crossed the half-band back toward init but never settled => auto-skip
           elif (abs(pos_rad - init_rad) < counting_band_rad) and self._prec_cont_band_entry_s is None:
               #first entry to this condition => collect the half-band-crossing timestamp
               if self._prec_return_est_t_s is None:
                   self._prec_return_est_t_s = now_s
               #mirror of the approach timeout: 2× the time-to-halfway plus a buffer
               if now_s - self._prec_return_t_start_s >= ((self._prec_return_est_t_s - self._prec_return_t_start_s)*2.0 + self._buffer_reach_t_s):
                   #force skip the return; _skip_trial() counts the return skip and flips the phase
                   self._prec_return_est_t_s = None
                   self._skip_trial()

           else:
               self._prec_cont_band_entry_s = None

The mirror image of the approach, aimed at ``init`` instead of the target, with two extra
details. The phase start time is stamped **lazily** on the phase's first sample, because
the return's timeout must be measured from when the return actually began — not from the
start of the whole run. And the **termination check lives here**: the run finishes when
*returns done + returns skipped* reaches ``repeat``, so the robot always completes its
drive back to init before the summary is published.

**The phase-aware skip —** ``_skip_trial``.

.. code-block:: python

   def _skip_trial(self):
       """Advance the precision run past a stuck phase.

       Single funnel for both the manual skip button (``_skip_iteration``) and the two
       auto-skip timeouts in ``_update_prec``. It OWNS both the skip-counter selection
       and the phase flip: the counter is chosen by reading ``_prec_at_target`` BEFORE
       the phase is flipped, so callers must never pre-flip the phase.
       """

       self._prec_cont_band_entry_s = None
       #NOTE: pick the skip counter from the CURRENT phase BEFORE flipping it below.
       if self._prec_at_target:
           # return-to-init phase skipped => count a RETURN skip, then go back to approach
           self._prec_return_skipped += 1
           self._prec_at_target       = False
           self._prec_return_est_t_s  = None
           self.get_logger().info(
               f"[Evaluator] precision return-to-init SKIPPED "
               f"({len(self._return_positions_rad)} done, {self._prec_return_skipped} skipped)"
           )
           #termination lives in the return phase: this full cycle is done (return dropped)
           if (len(self._return_positions_rad) + self._prec_return_skipped
                   >= self._payload["repeat"]):
               self._finish_experiment()
           return

       # approach phase skipped => count a TARGET skip, then flip into the return phase
       self._prec_skipped             += 1
       self._prec_at_target            = True
       self._prec_est_terminating_t_s  = None
       self._prec_return_t_start_s     = None   # lazily re-stamped in the return branch
       self.get_logger().info(
           f"[Evaluator] precision trial SKIPPED "
           f"({len(self._trial_positions_rad)} done, {self._prec_skipped} skipped)"
       )
       #NOTE: no termination check here anymore — termination lives in the return phase

One function is the *single funnel* for all three skip sources (the manual button and
the two auto-skip timeouts), so the bookkeeping can never diverge. Its docstring states
the one rule callers must respect: the function itself reads the current phase to choose
the right skip counter **before** flipping the phase — a caller that pre-flipped the
phase would make the skip land on the wrong counter. A skipped return still counts as a
completed cycle for the termination check; a skipped approach only flips into the return
phase, so the return half of that cycle is still observed.

**The outputs —** ``_compute_live`` **and** ``_compute_summary``.

Both functions are one ``if`` branch per aspect building a plain dictionary; the fields
were already listed in each aspect's workflow section. Two branches are worth reading in
full. First the precision **live** branch, because of its phase-tracking reference:

.. code-block:: python

       if action == "precision":
           tar_rad  = _to_rad(self._payload["target_pos"], self._payload["unit"])
           init_rad = _to_rad(self._payload["init_pos"],   self._payload["unit"])
           #the live reference tracks the ACTIVE phase: target while approaching, init
           #while returning. app.js feeds target_rad into the amber reference line, so it
           #follows the phase with no change on the JS drawing side.
           ref_rad = init_rad if self._prec_at_target else tar_rad
           return {
               "target_rad":        round(ref_rad, 5),
               "current_pos_rad":   round(pos_rad, 5),
               "current_error_rad": round(abs(pos_rad - ref_rad), 5),
               "trials_done":       len(self._trial_positions_rad),
               "returns_done":      len(self._return_positions_rad),
               "trials_skipped":    self._prec_skipped,
               "trials_total":      self._payload["repeat"],
               "elapsed_s":         round(elapsed_s, 3),
           }

The reported "target" is whatever the robot is *currently* trying to reach — the target
while approaching, init while returning. The browser draws its amber reference line from
this field, so the line follows the phase with zero extra logic on the JavaScript side.

And the precision **summary** branch, with its little scoring helper:

.. code-block:: python

       if action == "precision":
           tar_rad    = _to_rad(self._payload["target_pos"], self._payload["unit"])
           init_rad   = _to_rad(self._payload["init_pos"],   self._payload["unit"])
           n_required = self._payload["repeat"]

           def _grp(positions, ref_rad, n_skipped):
               """Accuracy (mean/max error) + precision (std) of one phase group."""
               errs = [abs(p - ref_rad) for p in positions]
               n    = len(errs)
               mean = sum(errs) / n if n > 0 else 0.0
               std  = (math.sqrt(sum((e - mean) ** 2 for e in errs) / n)
                       if n > 1 else 0.0)
               return {
                   "num_trials":     n,
                   "num_skipped":    n_skipped,
                   "mean_error_rad": round(mean, 5),
                   "std_error_rad":  round(std, 5),
                   "max_error_rad":  round(max(errs) if errs else 0.0, 5),
                   #return group reuses the same criteria as the target group
                   "pass_error":     n >= n_required and mean <= c["max_avg_error_rad"],
               }

           return {
               "target_rad":   round(tar_rad, 5),
               "init_rad":     round(init_rad, 5),
               #group 1: init→target reaching performance
               "target_group": _grp(self._trial_positions_rad, tar_rad, self._prec_skipped),
               #group 2: return→init returning performance
               "return_group": _grp(self._return_positions_rad, init_rad, self._prec_return_skipped),
           }

``_grp`` scores one phase group: the **mean** error is the accuracy (how close on
average), the **standard deviation** is the precision (how repeatable — the spread of
the settled positions), plus the worst case and the skip count. The same helper is
applied twice — once per phase — so the summary tells reaching and returning performance
apart. The pass rule demands both a complete run (``n ≥ repeat``) and a small enough mean
error. The other three summary branches follow the pass rules already given in their
workflow sections (point-to-point additionally requires *settled* for the overshoot to
pass, and performance compares its peaks with **≥** against the commanded values).
``main`` is the same standard entry point as on the other node pages.

Notation
--------

Words used on this page.

event-driven
    The node does nothing on its own clock — it only reacts when a message arrives.
    Between events it is idle and costs almost nothing.

payload
    The JSON data attached to an event. It carries the test settings: ``value`` and
    ``unit``, the ``order_sequence``, ``repeat``, and so on.

settling band / settled / settling window
    The band is a small error window around the target,
    ``max(0.01 rad, settling_band_pct % × |travel|)``. The robot is **settled** when it
    stays inside the band continuously for the settling window (0.5 s). Same words as in
    the system-overview Notation.

overshoot
    How far the robot passes the target before coming back, as a percentage of the
    travel distance. Measured direction-aware, from the tracked peak position.

waypoint
    One target in the pick-and-place sequence. Each waypoint is scored on its own.

index (hole)
    One hole on the 72-hole plate: 1 index = 360/72 = **5°** ≈ 0.08727 rad.

accuracy vs precision
    Two different qualities. **Accuracy** = how close to the target on average (the mean
    error). **Precision** = how repeatable the stops are (the spread). A robot can be
    precise but inaccurate — always stopping at the same wrong place.

standard deviation (std)
    A single number for spread: small std = the settled positions are clustered tightly
    together. This is the "precision" score of the precision test.

throttle
    Deliberately limiting a rate. The live metrics are throttled to one message per
    0.1 s (10 Hz) so the browser is not flooded by the 500 Hz sample stream.

service server
    The answering side of a ROS 2 service (the request/response pattern). This node
    *serves* ``update_criteria``; the web visualizer holds the matching *client*.

phase (approach / return)
    The two halves of one precision cycle: driving init → target (approach, group 1) and
    driving target → init (return, group 2). Each phase collects its own settled
    position and has its own skip counter.

auto-skip / force-skip
    The node skips a stuck phase by itself, using the timeout rule
    *2 × time-to-halfway + 1 s buffer* — no fixed magic number, the limit adapts to how
    fast the robot actually moves.

all-or-nothing validation
    A criteria update is applied only if **every** key and value in the request is
    valid; one bad entry rejects the whole request, so the limits can never end up
    half-updated.
