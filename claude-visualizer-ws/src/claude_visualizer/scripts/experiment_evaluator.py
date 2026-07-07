#!/usr/bin/env python3
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


class ExperimentEvaluator(Node):
    def __init__(self):
        super().__init__("experiment_evaluator")

        # ── parameters ───────────────────────────────────────────────────────
        # self.declare_parameter("pair_id", "0")
        # Launch passes pair_id via LaunchConfiguration; "11" is type-inferred to INT,
        # while the default/standalone case is STR — dynamic_typing accepts either
        # (the criteria loader normalizes with str()). Fixes InvalidParameterTypeException.
        self.declare_parameter(
            "pair_id", "0", ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter("criteria_file_path", "")

        pair_id       = self.get_parameter("pair_id").value
        criteria_file = self.get_parameter("criteria_file_path").value

        # ── load criteria ────────────────────────────────────────────────────
        self._criteria = self._load_criteria(criteria_file, pair_id)
        self.get_logger().info(
            f"[ExperimentEvaluator] pair_id={pair_id!r}  criteria={self._criteria}"
        )

        # ── QoS ──────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── subscriptions ────────────────────────────────────────────────────
        self._event_sub = self.create_subscription(
            EventTrigger, "/event_trigger", self._event_trigger_cb, reliable_qos
        )
        self._states_sub = self.create_subscription(
            EncoderState, "/estimated_states", self._estimated_states_cb, reliable_qos
        )

        # ── publishers ───────────────────────────────────────────────────────
        self._live_pub    = self.create_publisher(ExperimentEval, "/eval_live",    reliable_qos)
        self._summary_pub = self.create_publisher(ExperimentEval, "/eval_summary", reliable_qos)

        # ── service ──────────────────────────────────────────────────────────
        self._update_criteria_srv = self.create_service(
            UpdateCriteria, "/update_criteria", self._update_criteria_cb
        )

        self._reset_state()

    # ── criteria loader ───────────────────────────────────────────────────────
    def _load_criteria(self, path: str, pair_id: str) -> dict:
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
        # table = data.get("criteria", {})
        # if robot_id in table:
        #     return table[robot_id]
        # self.get_logger().warn(
        #     f"robot_id {robot_id!r} not in criteria table; using 'default'"
        # )
        # return table.get("default", default)
        # Normalize keys to strings: YAML parses bare `11:` as an int, but pair_id
        # arrives as a string from the ROS param. pair_id 0 / unlisted → 'default'.
        table = {str(k): v for k, v in data.get("criteria", {}).items()}
        key = str(pair_id)
        if key in table:
            return table[key]
        self.get_logger().warn(
            f"pair_id {pair_id!r} not in criteria table; using 'default'"
        )
        return table.get("default", default)
    
    # ── criteria update ───────────────────────────────────────────────────────
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


    # ── experiment state reset ────────────────────────────────────────────────
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

    # ── event trigger callback ────────────────────────────────────────────────
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

    # ── experiment lifecycle ──────────────────────────────────────────────────
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

    # ── estimated_states callback ─────────────────────────────────────────────
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

    # ── per-experiment update logic ───────────────────────────────────────────
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

    # ── manual skip ───────────────────────────────────────────────────────────
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

    def _skip_waypoint(self):
        """Drop the current (unsettled) waypoint and advance to the next one.

        The skipped waypoint is recorded as a failed/skipped result (excluded from the
        average-error summary). The next waypoint's travel baseline is set to the robot's
        ACTUAL current position (latest /estimated_states) instead of the unreached
        target, so its settling band and overshoot are measured from where the robot is.
        """
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

    def _skip_trial(self):
        """Advance the precision run past a stuck phase.

        Single funnel for both the manual skip button (``_skip_iteration``) and the two
        auto-skip timeouts in ``_update_prec``. It OWNS both the skip-counter selection
        and the phase flip: the counter is chosen by reading ``_prec_at_target`` BEFORE
        the phase is flipped, so callers must never pre-flip the phase.

        Phase = approaching the target (``_prec_at_target`` False): the init→target
        measurement is dropped — count a TARGET skip (``_prec_skipped``), then flip into
        the return phase so the return-to-init half is still observed.

        Phase = returning to the init position (``_prec_at_target`` True): the return→init
        measurement is dropped — count a RETURN skip (``_prec_return_skipped``), flip back
        to the approach phase, then run the termination check (termination lives here).
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

    def _update_perf(self, vel_rad_s: float, accel_rad_s2: float):
        self._peak_speed_rad_s  = max(self._peak_speed_rad_s,  abs(vel_rad_s))
        self._peak_accel_rad_s2 = max(self._peak_accel_rad_s2, abs(accel_rad_s2))

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

    # ── live metrics ──────────────────────────────────────────────────────────
    def _compute_live(
        self, action: str,
        pos_rad: float, vel_rad_s: float, accel_rad_s2: float, elapsed_s: float
    ) -> dict:
        if action == "point_to_point":
            target_rad = self._start_pos_rad + _to_rad(
                self._payload["value"], self._payload["unit"]
            )
            return {
                "target_rad":        round(target_rad, 5),
                "current_pos_rad":   round(pos_rad, 5),
                "current_error_rad": round(abs(pos_rad - target_rad), 5),
                "elapsed_s":         round(elapsed_s, 3),
            }
        if action == "pick_place":
            seq     = self._payload["order_sequence"]
            tar_rad = seq[self._wp_idx] * RAD_PER_INDEX if self._wp_idx < len(seq) else 0.0
            return {
                "current_waypoint":  self._wp_idx + 1,
                "total_waypoints":   len(seq),
                "target_rad":        round(tar_rad, 5),
                "current_pos_rad":   round(pos_rad, 5),
                "current_error_rad": round(abs(pos_rad - tar_rad), 5),
                "elapsed_s":         round(elapsed_s, 3),
            }
        if action == "performance":
            return {
                "peak_speed_rad_s":    round(self._peak_speed_rad_s,  5),
                "peak_accel_rad_s2":   round(self._peak_accel_rad_s2, 5),
                "current_speed_rad_s": round(abs(vel_rad_s), 5),
                "elapsed_s":           round(elapsed_s, 3),
            }
        if action == "precision":
            # tar_rad = _to_rad(self._payload["target_pos"], self._payload["unit"])
            # return {
            #     "target_rad":        round(tar_rad, 5),
            #     "current_pos_rad":   round(pos_rad, 5),
            #     "current_error_rad": round(abs(pos_rad - tar_rad), 5),
            #     "trials_done":       len(self._trial_positions_rad),
            #     "trials_skipped":    self._prec_skipped,
            #     "trials_total":      self._payload["repeat"],
            #     "elapsed_s":         round(elapsed_s, 3),
            # }
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
        return {}

    # ── summary computation ───────────────────────────────────────────────────
    def _compute_summary(self, action: str) -> dict:
        c = self._criteria

        if action == "point_to_point":
            target_rad = self._start_pos_rad + _to_rad(
                self._payload["value"], self._payload["unit"]
            )
            travel_rad = target_rad - self._start_pos_rad
            settled    = self._settling_time_s is not None

            if self._samples:
                final_error_rad = abs(self._samples[-1][0] - target_rad)
                if travel_rad > 0:
                    overshoot_rad = max(0.0, max(s[0] for s in self._samples) - target_rad)
                elif travel_rad < 0:
                    overshoot_rad = max(0.0, target_rad - min(s[0] for s in self._samples))
                else:
                    overshoot_rad = 0.0
            else:
                final_error_rad = abs(travel_rad)
                overshoot_rad   = 0.0

            overshoot_pct   = (overshoot_rad / abs(travel_rad) * 100.0) if abs(travel_rad) > 1e-9 else 0.0
            settling_time_s = self._settling_time_s if settled else float("inf")

            return {
                "target_rad":       round(target_rad, 5),
                "final_error_rad":  round(final_error_rad, 5),
                "overshoot_pct":    round(overshoot_pct, 3),
                "settling_time_s":  round(settling_time_s, 4) if settled else None,
                "pass_final_error": final_error_rad  <= c["max_avg_error_rad"],
                "pass_overshoot":   settled and overshoot_pct <= c["max_overshoot_pct"],
                "pass_settling":    settling_time_s  <= c["max_settling_time_s"],
            }

        if action == "pick_place":
            scored = [r for r in self._wp_results if not r.get("skipped")]
            avg_error_rad = (
                sum(r["final_error_rad"] for r in scored) / len(scored)
                if scored else 0.0
            )
            passed_count = sum(
                1 for r in self._wp_results
                if r["pass_error"] and r["pass_overshoot"] and r["pass_settling"]
            )
            return {
                "total_waypoints": len(self._payload["order_sequence"]),
                "avg_error_rad":   round(avg_error_rad, 5),
                "pass_avg_error":  len(scored) > 0 and avg_error_rad <= c["max_avg_error_rad"],
                "passed":          passed_count,
                "failed":          len(self._wp_results) - passed_count,
                "skipped":         len(self._wp_results) - len(scored),
                "details":         self._wp_results,
            }

        if action == "performance":
            # Payload speed/accel take priority over criteria minimums
            cmd_speed_rad_s  = self._payload.get("speed", c["min_speed"])
            cmd_accel_rad_s2 = self._payload.get("accel", c["min_acceleration"])
            return {
                "commanded_speed_rad_s":  cmd_speed_rad_s,
                "peak_speed_rad_s":       round(self._peak_speed_rad_s, 5),
                "commanded_accel_rad_s2": cmd_accel_rad_s2,
                "peak_accel_rad_s2":      round(self._peak_accel_rad_s2, 5),
                "pass_speed":             self._peak_speed_rad_s  >= cmd_speed_rad_s,
                "pass_accel":             self._peak_accel_rad_s2 >= cmd_accel_rad_s2,
            }

        if action == "precision":
            # ── previous single-group (target-only) summary ──
            # tar_rad    = _to_rad(self._payload["target_pos"], self._payload["unit"])
            # n_required = self._payload["repeat"]
            # errors_rad = [abs(t - tar_rad) for t in self._trial_positions_rad]
            # n_done     = len(errors_rad)
            # mean_error_rad = sum(errors_rad) / n_done if n_done > 0 else 0.0
            # std_error_rad  = (
            #     math.sqrt(sum((e - mean_error_rad) ** 2 for e in errors_rad) / n_done)
            #     if n_done > 1 else 0.0
            # )
            # max_error_rad  = max(errors_rad) if errors_rad else 0.0
            # return {
            #     "target_rad":     round(tar_rad, 5),
            #     "num_trials":     n_done,
            #     "num_skipped":    self._prec_skipped,
            #     "mean_error_rad": round(mean_error_rad, 5),
            #     "std_error_rad":  round(std_error_rad, 5),
            #     "max_error_rad":  round(max_error_rad, 5),
            #     "pass_error":     n_done >= n_required and mean_error_rad <= c["max_avg_error_rad"],
            # }

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

        return {}


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
