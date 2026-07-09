#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Empty

from claude_visualizer_interface.msg import EncoderRaw, EncoderState


class KalmanFilterNode(Node):
    """
    Constant-jerk Kalman filter node (encoder_reader).

    Subscribes : /encoder_raw   (EncoderRaw)
    Publishes  : /estimated_states (EstimatedStates)

    State vector  : x = [position, velocity, acceleration, jerk]  (rad, rad/s, rad/s², rad/s³)
    Measurement   : position in radians converted from cumulative ticks

    State-transition model (constant jerk, variable dt):
        pos_{k+1} = pos + vel·dt + ½·acc·dt² + ⅙·jerk·dt³
        vel_{k+1} = vel + acc·dt + ½·jerk·dt²
        acc_{k+1} = acc + jerk·dt
        jerk_{k+1} = jerk
    """

    def __init__(self):
        super().__init__("encoder_reader")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("ticks_per_rev", 8192)
        self.declare_parameter("kf_q_position", 0.001)
        self.declare_parameter("kf_q_velocity", 0.01)
        self.declare_parameter("kf_q_acceleration", 0.1)
        self.declare_parameter("kf_q_jerk", 1.0)
        self.declare_parameter("kf_r_position", 0.5)
        self.declare_parameter("kf_p0", 1.0)

        ticks_per_rev = self.get_parameter("ticks_per_rev").value
        self._ticks_to_rad = 2.0 * np.pi / ticks_per_rev

        q_pos  = self.get_parameter("kf_q_position").value
        q_vel  = self.get_parameter("kf_q_velocity").value
        q_acc  = self.get_parameter("kf_q_acceleration").value
        q_jerk = self.get_parameter("kf_q_jerk").value
        r_pos  = self.get_parameter("kf_r_position").value
        p0     = self.get_parameter("kf_p0").value

        # ── Kalman Filter state ─────────────────────────────────────────────────────
        self._x = np.zeros(4)               # [pos, vel, acc, jerk]
        self._P = np.eye(4) * p0            # state covariance
        self._Q = np.diag([q_pos, q_vel, q_acc, q_jerk])
        self._R = np.array([[r_pos]])
        self._H = np.array([[1.0, 0.0, 0.0, 0.0]])
        self._initialized = False

        # ── Position zero reference ──────────────────────────────────────────
        # Subtracted from the published position so every downstream consumer
        # (web_visualizer WS, experiment_evaluator, LSL outlet) shares one
        # zeroed frame. _last_raw_pos_rad holds the most recent *unzeroed*
        # position so a (re-)zero captures the true current location.
        self._zero_offset_rad = 0.0
        self._last_raw_pos_rad = 0.0

        # ── ROS I/O ──────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Relative topic names so the node's namespace (/G<N>) is prepended.
        # self._pub = self.create_publisher(EncoderState, "/estimated_states", qos)
        # self.create_subscription(EncoderRaw, "/encoder_raw", self._cb, qos)
        # self.create_subscription(Empty, "/zero_estimated_states", self._zero_cb, qos)
        self._pub = self.create_publisher(EncoderState, "estimated_states", qos)
        self.create_subscription(EncoderRaw, "encoder_raw", self._cb, qos)
        self.create_subscription(Empty, "zero_estimated_states", self._zero_cb, qos)

        self.get_logger().info(
            f"encoder_reader started — ticks_per_rev={ticks_per_rev}, "
            f"Q_diag={[q_pos, q_vel, q_acc, q_jerk]}, R={r_pos}"
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _build_F(self, dt: float) -> np.ndarray:
        """State-transition matrix for constant-jerk model."""
        return np.array([
            [1.0, dt, 0.5 * dt ** 2, dt ** 3 / 6.0],
            [0.0, 1.0,          dt,  0.5 * dt ** 2],
            [0.0, 0.0,         1.0,             dt],
            [0.0, 0.0,         0.0,            1.0],
        ])

    # ── Callback ──────────────────────────────────────────────────────────────

    def _cb(self, msg: EncoderRaw) -> None:
        dt = float(msg.dt_us) * 1e-6
        z = np.array([msg.ticks * self._ticks_to_rad])

        # Seed state with first measurement; skip until dt is valid
        if not self._initialized:
            self._x[0] = z[0]
            self._initialized = True
            return

        if dt <= 0.0:
            self.get_logger().warn(
                f"dt_us={msg.dt_us} — skipping sample", throttle_duration_sec=1.0
            )
            return

        F = self._build_F(dt)

        # ── Predict ──────────────────────────────────────────────────────────
        x_pred = F @ self._x
        P_pred = F @ self._P @ F.T + self._Q

        # ── Update ───────────────────────────────────────────────────────────
        y = z - self._H @ x_pred
        S = self._H @ P_pred @ self._H.T + self._R
        K = P_pred @ self._H.T @ np.linalg.inv(S)
        self._x = x_pred + (K @ y)
        self._P = (np.eye(4) - K @ self._H) @ P_pred

        # ── Publish ──────────────────────────────────────────────────────────
        # Position is zeroed against the last commanded reference; velocity and
        # acceleration are unaffected (a constant offset has zero derivative).
        self._last_raw_pos_rad = float(msg.ticks * self._ticks_to_rad)

        out = EncoderState()
        out.header       = msg.header
        # out.position     = float(self._x[0])
        out.position     = self._last_raw_pos_rad - self._zero_offset_rad
        out.velocity     = float(self._x[1])
        out.acceleration = float(self._x[2])
        out.pos_variance = float(self._P[0, 0])
        out.vel_variance = float(self._P[1, 1])
        out.acc_variance = float(self._P[2, 2])
        out.raw_ticks    = msg.ticks
        self._pub.publish(out)

    # ── Zero command ────────────────────────────────────────────────────────────

    def _zero_cb(self, _msg: Empty) -> None:
        """Capture the current raw position as the new zero reference."""
        self._zero_offset_rad = self._last_raw_pos_rad
        self.get_logger().info(
            f"[zero] /estimated_states zeroed at {self._zero_offset_rad:.5f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
