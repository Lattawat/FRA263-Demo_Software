State Estimator Node (Kalman Filter)
====================================

.. Node description (the design idea, detail, and other crucial info)

**Role.** The node ``encoder_reader`` turns the noisy raw encoder ticks into smooth
velocity and acceleration and a clean position. It uses a **constant-jerk Kalman filter**
whose state is ``[position, velocity, acceleration, jerk]``. It reads one topic and writes
one topic:

.. code-block:: text

   /encoder_raw                 ┌───────────────────────────┐            /estimated_states
   (EncoderRaw)  ──────────────▶│       encoder_reader      │───────────▶ (EncoderState)
   ticks, dt_us                 │   constant-jerk Kalman     │            position, velocity,
                                │   filter                   │            acceleration, variances
                                └───────────────────────────┘

**Tuning — process and measurement noise.** The filter is tuned with two covariances,
both set in ``params.yaml``. The process-noise matrix ``Q`` is a diagonal, and each state
gets a **different value because the states live at different orders of magnitude**:

.. list-table::
   :header-rows: 1
   :widths: 24 14 62

   * - Parameter
     - Value
     - Meaning
   * - ``kf_q_position``
     - ``1e-8``
     - Process noise on position. Near zero — the model is trusted to carry position almost exactly.
   * - ``kf_q_velocity``
     - ``1e-8``
     - Process noise on velocity. Also near zero, for the same reason.
   * - ``kf_q_acceleration``
     - ``1e-3``
     - Process noise on acceleration. Small but non-zero.
   * - ``kf_q_jerk``
     - ``5.0``
     - Process noise on jerk. Large — almost all of the model's slack lives here.
   * - ``kf_r_position``
     - ``4.65e-6``
     - Measurement noise on the encoder position (in rad²). Larger = smoother, but trusts the encoder less.
   * - ``kf_p0``
     - ``1.0``
     - Initial state uncertainty (the starting value of the covariance ``P``).

The point of this ordering — tiny at the bottom of the list, large at the top — is that
the model assumes **jerk is constant** between samples, which is never exactly true. By
giving ``jerk`` a large ``Q`` and ``position`` / ``velocity`` a near-zero ``Q``, the
filter is told: "trust that position and velocity follow the motion equations; let the
surprise land on jerk." When the real motion changes, the filter adapts through the jerk
term, which then feeds acceleration, then velocity, then position. That keeps the output
smooth without making it slow to react.

**Interfaces.** The node has three interfaces:

- **Subscribes** ``/encoder_raw`` (``EncoderRaw``: ``ticks``, ``raw_position``, ``dt_us``)
  — the raw encoder feed the filter runs on.
- **Subscribes** ``/zero_estimated_states`` (``std_msgs/Empty``) — an empty "re-zero now"
  signal. On any message the node captures the current raw position as the new zero, so
  the published position is re-zeroed at the source for every downstream reader at once.
- **Publishes** ``/estimated_states`` (``EncoderState``: ``position``, ``velocity``,
  ``acceleration``, their variances, and ``raw_ticks``). The published ``position`` is the
  raw encoder angle minus the zero offset — **not** the filter's own position state
  ``x[0]``; only ``velocity`` and ``acceleration`` come from the filter.

All topic names are **relative**, so the launch namespace ``/G<N>/`` is added in front (for
example ``/G0/encoder_raw``). The QoS profile is **RELIABLE** with **KEEP_LAST depth 10**.

Node Workflow
-------------

.. The flow chart of this whole node

Every new ``/encoder_raw`` message runs one pass of the filter in the callback ``_cb``. A
separate, rare ``/zero_estimated_states`` message just moves the zero reference. The flow
is:

.. code-block:: text

                    ┌──────────────────────────────────────────────┐
   /encoder_raw ───▶│  _cb(msg)                                    │
                    │    dt = msg.dt_us × 1e-6                      │
                    │    z  = msg.ticks × ticks_to_rad             │
                    └───────────────────────┬──────────────────────┘
                                            ▼
                             first sample ever?  ──── yes ──▶  x[0] = z ; return
                                            │ no
                                            ▼
                                   dt > 0 ?  ──── no ───▶  warn "bad dt" ; skip
                                            │ yes
                                            ▼
                          F = build_F(dt)   (constant-jerk transition matrix)
                                            │
                                            ▼
           PREDICT :  x⁻ = F·x                P⁻ = F·P·Fᵀ + Q
                                            │
                                            ▼
           UPDATE  :  y = z − H·x⁻           K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
                      x = x⁻ + K·y           P = (I − K·H)·P⁻
                                            │
                                            ▼
              position = (ticks × ticks_to_rad) − zero_offset
              velocity = x[1]        acceleration = x[2]
                                            │
                                            ▼
                        publish  /estimated_states  (EncoderState)


   /zero_estimated_states (Empty) ──▶  _zero_cb()  :  zero_offset = last_raw_pos

In words: convert the tick count to radians, seed the filter on the very first sample,
and skip any sample with a bad time step. Otherwise build the transition matrix for this
time step, **predict** the next state, **correct** it with the new measurement, and
publish. The published position is the raw angle minus the current zero offset, while
velocity and acceleration are taken from the filter's state.

Examine the code
----------------

.. referencing the section 2.1 of the mentioned link

The full node lives in ``scripts/Kalman_filter.py``. This section walks through it block by
block.

**Imports and messages.**

.. code-block:: python

   import numpy as np
   import rclpy
   from rclpy.node import Node
   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

   from std_msgs.msg import Empty

   from claude_visualizer_interface.msg import EncoderRaw, EncoderState

``numpy`` does the matrix maths of the Kalman filter. ``rclpy`` is the ROS 2 Python
library, and ``Node`` is the base class every node inherits from. The ``QoS`` imports let
us set how messages are delivered. ``Empty`` is the message type for the "re-zero" signal
(it carries no data — the arrival itself is the signal). ``EncoderRaw`` and
``EncoderState`` are the project's own message types, defined in the interface package.

**The class and its motion model.**

.. code-block:: python

   class KalmanFilterNode(Node):
       """
       Constant-jerk Kalman filter node (encoder_reader).

       State vector  : x = [position, velocity, acceleration, jerk]  (rad, rad/s, rad/s², rad/s³)
       Measurement   : position in radians converted from cumulative ticks

       State-transition model (constant jerk, variable dt):
           pos_{k+1} = pos + vel·dt + ½·acc·dt² + ⅙·jerk·dt³
           vel_{k+1} = vel + acc·dt + ½·jerk·dt²
           acc_{k+1} = acc + jerk·dt
           jerk_{k+1} = jerk
       """

The docstring states the model the whole node is built on. The four equations are the
standard "next position from current motion" equations, carried out to the jerk term. The
last line, ``jerk_{k+1} = jerk``, is the assumption that gives the filter its name: jerk is
held constant from one sample to the next. Everything below is this model written as
matrices.

**Constructor — parameters.**

.. code-block:: python

   def __init__(self):
       super().__init__("encoder_reader")

       self.declare_parameter("ticks_per_rev", 8192)
       self.declare_parameter("kf_q_position", 0.001)
       self.declare_parameter("kf_q_velocity", 0.01)
       self.declare_parameter("kf_q_acceleration", 0.1)
       self.declare_parameter("kf_q_jerk", 1.0)
       self.declare_parameter("kf_r_position", 0.5)
       self.declare_parameter("kf_p0", 1.0)

       ticks_per_rev = self.get_parameter("ticks_per_rev").value
       self._ticks_to_rad = 2.0 * np.pi / ticks_per_rev

The node registers as ``encoder_reader`` and then declares its tunable parameters. The
values passed to ``declare_parameter`` here are only **fallback defaults**; at launch they
are overridden by ``params.yaml`` (the tuned values shown in the table above). Finally it
works out ``_ticks_to_rad``, the factor that turns a raw tick count into radians — one full
revolution is ``ticks_per_rev`` ticks and ``2π`` radians, so the factor is
``2π / ticks_per_rev``.

**Constructor — the Kalman filter matrices.**

.. code-block:: python

       self._x = np.zeros(4)               # [pos, vel, acc, jerk]
       self._P = np.eye(4) * p0            # state covariance
       self._Q = np.diag([q_pos, q_vel, q_acc, q_jerk])
       self._R = np.array([[r_pos]])
       self._H = np.array([[1.0, 0.0, 0.0, 0.0]])
       self._initialized = False

       self._zero_offset_rad = 0.0
       self._last_raw_pos_rad = 0.0

These are the pieces of the filter. ``_x`` is the current estimate of the four states,
starting at zero. ``_P`` is how unsure the filter is about that estimate. ``_Q`` is the
process noise (how much we distrust the motion model) and ``_R`` is the measurement noise
(how much we distrust the encoder). ``_H`` is the measurement matrix ``[1, 0, 0, 0]``,
which says "the encoder measures position only." ``_initialized`` guards the first sample.
The last two lines hold the zeroing state: ``_zero_offset_rad`` is the position we call
zero, and ``_last_raw_pos_rad`` remembers the most recent raw position so a re-zero can
capture it.

**Constructor — ROS inputs and outputs.**

.. code-block:: python

       qos = QoSProfile(
           reliability=ReliabilityPolicy.RELIABLE,
           history=HistoryPolicy.KEEP_LAST,
           depth=10,
       )
       # Relative topic names so the node's namespace (/G<N>) is prepended.
       self._pub = self.create_publisher(EncoderState, "estimated_states", qos)
       self.create_subscription(EncoderRaw, "encoder_raw", self._cb, qos)
       self.create_subscription(Empty, "zero_estimated_states", self._zero_cb, qos)

The QoS profile asks for reliable delivery and keeps the last 10 messages. The node then
creates one publisher and two subscriptions. The subscriptions link each incoming topic to
its callback: ``encoder_raw`` runs ``_cb`` (the filter), and ``zero_estimated_states`` runs
``_zero_cb`` (the re-zero). The topic names are written **without** a leading slash on
purpose — that makes them relative, so the launch file's namespace ``/G<N>/`` is added in
front and each group's nodes stay on their own topics.

**The filter callback.** This is the heart of the node. It runs once per encoder message.
First, the setup and the two early exits:

.. code-block:: python

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

``dt`` is the time step in seconds (the message carries microseconds). ``z`` is the
measurement: the tick count turned into radians. On the very first message there is no
previous state to predict from, so the filter just sets its position to the measurement and
returns. On any later message with a non-positive ``dt`` it warns and skips, because a zero
or negative time step would break the maths.

.. code-block:: python

       F = self._build_F(dt)

       # ── Predict ──
       x_pred = F @ self._x
       P_pred = F @ self._P @ F.T + self._Q

       # ── Update ──
       y = z - self._H @ x_pred
       S = self._H @ P_pred @ self._H.T + self._R
       K = P_pred @ self._H.T @ np.linalg.inv(S)
       self._x = x_pred + (K @ y)
       self._P = (np.eye(4) - K @ self._H) @ P_pred

This is the standard two-step Kalman filter. **Predict**: use the transition matrix ``F``
to push the state forward one time step, and grow the uncertainty by the process noise
``Q``. **Update**: compare the real measurement to the predicted position (``y``, the
"innovation"), work out the Kalman gain ``K`` (how much to trust the measurement versus the
prediction), then nudge the state and shrink the uncertainty. The gain is what balances a
smooth output against a responsive one.

.. code-block:: python

       # ── Publish ──
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

Finally the node fills in and publishes the result. Note the commented-out
``# out.position = float(self._x[0])``: the filter's own position estimate is deliberately
**not** used. Instead the published position is the raw encoder angle minus the zero
offset. This is a design choice — the raw angle is already accurate, and subtracting one
constant offset keeps every consumer on the same zero without the filter's small lag. Only
``velocity`` and ``acceleration`` come from the filter state (``x[1]`` and ``x[2]``); the
variances come from the diagonal of ``P`` and travel with the estimate so a reader can see
how confident the filter is.

**The re-zero callback.**

.. code-block:: python

   def _zero_cb(self, _msg: Empty) -> None:
       """Capture the current raw position as the new zero reference."""
       self._zero_offset_rad = self._last_raw_pos_rad
       self.get_logger().info(
           f"[zero] /estimated_states zeroed at {self._zero_offset_rad:.5f} rad"
       )

When an empty message arrives on ``/zero_estimated_states``, this stores the most recent
raw position as the new zero offset. From the next published message on, ``position``
counts from that point. Because the offset is applied here, at the source, every downstream
reader (the web UI, the LSL stream, the evaluator) sees the same zeroed frame — no reader
has to correct the position itself.

**Main.**

.. code-block:: python

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

This is the standard ROS 2 entry point. It starts ROS, creates the node, and calls
``rclpy.spin`` to hand control to ROS so the callbacks run whenever a message arrives.
``spin`` blocks until the node is stopped (``Ctrl+C`` raises ``KeyboardInterrupt``); the
``finally`` block then cleans up the node and shuts ROS down tidily.
