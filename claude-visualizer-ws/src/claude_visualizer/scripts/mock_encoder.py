#!/usr/bin/env python3
"""
mock_encoder — Phase 1 synthetic encoder publisher
===================================================
Replaces the Teensy 4.1 + micro-ROS stack during Phase 1 testing.

Publishes:
    /encoder_raw  (motion_verifier/EncoderRaw)

Waveform modes (set via 'waveform' parameter):
    sine       — smooth sinusoidal position profile
    trapezoid  — constant-acceleration velocity profile (realistic motion)
    step       — instantaneous position jumps (stress-tests the KF)

All waveforms repeat indefinitely.  Gaussian noise is added to the tick
count to simulate real quadrature encoder behaviour.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from claude_visualizer_interface.msg import EncoderRaw


# ── waveform generators ───────────────────────────────────────────────────────

class SineWaveform:
    """pos(t) = A · sin(2π·f·t)"""

    def __init__(self, amplitude: float, frequency_hz: float) -> None:
        self.amplitude = amplitude
        self.frequency = frequency_hz

    def position(self, t: float) -> float:
        return self.amplitude * math.sin(2.0 * math.pi * self.frequency * t)


class TrapezoidWaveform:
    """
    Trapezoidal velocity profile — realistic robot motion.

    One period:  ramp-up → cruise → ramp-down → (reverse) ramp-up → cruise → ramp-down
    The profile is symmetric around zero and repeats continuously.
    """

    def __init__(
        self,
        max_velocity: float,   # [rad/s]
        acceleration: float,   # [rad/s²]
        cruise_time: float,    # [s]
    ) -> None:
        self.v_max = max_velocity
        self.a = acceleration
        self.t_ramp = max_velocity / acceleration
        self.t_cruise = cruise_time
        self.t_half = self.t_ramp + self.t_cruise + self.t_ramp
        self.t_period = self.t_half * 2.0

        self.d_ramp = 0.5 * acceleration * self.t_ramp ** 2
        self.d_cruise = max_velocity * cruise_time
        self.d_half = self.d_ramp + self.d_cruise + self.d_ramp

    def position(self, t: float) -> float:
        t_mod = math.fmod(t, self.t_period)
        if t_mod >= self.t_half:
            return self.d_half - self._half_pos(t_mod - self.t_half)
        return self._half_pos(t_mod)

    def _half_pos(self, t: float) -> float:
        """Position within one forward half-cycle."""
        if t < self.t_ramp:
            return 0.5 * self.a * t ** 2
        t -= self.t_ramp
        if t < self.t_cruise:
            return self.d_ramp + self.v_max * t
        t -= self.t_cruise
        return self.d_ramp + self.d_cruise + self.v_max * t - 0.5 * self.a * t ** 2


class StepWaveform:
    """Alternates between 0 and amplitude every half-period — stress-tests the KF."""

    def __init__(self, amplitude: float, period: float = 2.0) -> None:
        self.amplitude = amplitude
        self.period = period

    def position(self, t: float) -> float:
        return self.amplitude if math.fmod(t, self.period) >= self.period / 2.0 else 0.0


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class MockEncoderNode(Node):

    def __init__(self) -> None:
        super().__init__("mock_encoder")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("waveform", "trapezoid")
        self.declare_parameter("sine_amplitude", 1.0)
        self.declare_parameter("sine_frequency_hz", 0.5)
        self.declare_parameter("trap_max_velocity", 3.1416)
        self.declare_parameter("trap_acceleration", 1.5708)
        self.declare_parameter("trap_cruise_time_s", 2.0)
        self.declare_parameter("noise_std_ticks", 1.0)
        self.declare_parameter("ticks_per_rev", 8192)

        rate_hz       = self.get_parameter("publish_rate_hz").value
        waveform_str  = self.get_parameter("waveform").value
        noise_std     = self.get_parameter("noise_std_ticks").value
        ticks_per_rev = self.get_parameter("ticks_per_rev").value

        self._ticks_per_rad = ticks_per_rev / (2.0 * math.pi)
        self._noise_std = float(noise_std)
        self._rng = np.random.default_rng()
        self._waveform = self._build_waveform(waveform_str)

        # ── Publisher ────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Relative topic names so the node's namespace (/G<N>) is prepended.
        # self._pub = self.create_publisher(EncoderRaw, "/encoder_raw", qos)
        self._pub = self.create_publisher(EncoderRaw, "encoder_raw", qos)

        # self._debug_pub = self.create_publisher(EncoderRaw, "/euler_velocity", qos)
        self._debug_pub = self.create_publisher(EncoderRaw, "euler_velocity", qos)

        # ── Internal state ───────────────────────────────────────────────────
        # Waveform phase is derived from wall-clock time (time.time()), so that
        # this process and mock_robot_controller produce the same waveform value
        # at the same instant even though they started at different moments.
        # _t_last is monotonic — used only for dt_us computation (NTP-immune).
        self._t_last = time.monotonic()
        self._p_last = 0.0

        # ── Timer ────────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate_hz, self._cb)

        self.get_logger().info(
            f"mock_encoder ready — waveform={waveform_str}  "
            f"rate={rate_hz:.0f} Hz  ticks/rev={ticks_per_rev}  "
            f"noise_std={noise_std:.2f} ticks"
        )

    # ─────────────────────────────────────────────────────────────────────────

    def _cb(self) -> None:
        now = time.monotonic()
        dt_s = now - self._t_last
        self._t_last = now

        true_pos_rad = self._waveform.position(time.time())
        true_ticks = int(true_pos_rad * self._ticks_per_rad)
        noise = int(self._rng.normal(0.0, self._noise_std))

        msg = EncoderRaw()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "encoder"
        msg.ticks = true_ticks + noise
        msg.raw_position = ((true_ticks + noise) / self._ticks_per_rad)  # divide ticks_per_rad == multiply rad_per_tick
        msg.dt_us = max(0, int(dt_s * 1e6))
        self._pub.publish(msg)

        debug_msg = EncoderRaw()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = "euler_velocity"
        debug_msg.raw_position = (true_pos_rad - self._p_last)/dt_s
        self._p_last = true_pos_rad
        self._debug_pub.publish(debug_msg)

    def _build_waveform(self, name: str):
        if name == "sine":
            return SineWaveform(
                self.get_parameter("sine_amplitude").value,
                self.get_parameter("sine_frequency_hz").value,
            )
        elif name == "trapezoid":
            return TrapezoidWaveform(
                self.get_parameter("trap_max_velocity").value,
                self.get_parameter("trap_acceleration").value,
                self.get_parameter("trap_cruise_time_s").value,
            )
        elif name == "step":
            return StepWaveform(
                amplitude=self.get_parameter("sine_amplitude").value,
            )
        else:
            self.get_logger().warn(f"Unknown waveform '{name}', using 'sine'.")
            return SineWaveform(
                self.get_parameter("sine_amplitude").value,
                self.get_parameter("sine_frequency_hz").value,
            )


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()