claude_visualizer_interface/EncoderState Message
-------------------------------------------------

File: ``claude_visualizer_interface/msg/EncoderState.msg``

Raw Message Definition
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   # Kalman-filtered kinematic estimates published by encoder_reader node.
   # Consumers: web_visualizer node, robot_controller.py (via LSL re-pub).

   std_msgs/Header header          # ROS 2 standard header (timestamp + frame_id)

   float64 position                # estimated position  [rad or m depending on setup]
   float64 velocity                # estimated velocity  [rad/s or m/s]
   float64 acceleration            # estimated acceleration [rad/s² or m/s²]

   # Kalman filter diagnostics (optional but useful during tuning)
   float64 pos_variance            # posterior variance — position
   float64 vel_variance            # posterior variance — velocity
   float64 acc_variance            # posterior variance — acceleration

   # Raw tick count passed through for logging / sanity checks
   int32   raw_ticks

Compact Message Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   std_msgs/Header header
   float64 position
   float64 velocity
   float64 acceleration
   float64 pos_variance
   float64 vel_variance
   float64 acc_variance
   int32 raw_ticks

Where it is used
^^^^^^^^^^^^^^^^

Published by the **State Estimator node**. Subscribed by the **Web Visualizer node**
(which forwards it to the browser and pushes it to the Base System through an LSL outlet) and by the
**Experiment Evaluator node**.

.. This message is the single source of motion truth for the rest of the system. The evaluator
.. computes *every* metric from it — error, overshoot, settling time, peak speed, peak
.. acceleration, precision spread — and the browser plots it live. An encoder gives position
.. only, so without the velocity and acceleration estimates carried here there would be no way
.. to score a speed or acceleration test at all.

.. The three ``*_variance`` fields are **diagnostics**, not control data. A variance is the
.. filter's own statement of how uncertain it is about that state, so watching these values
.. settle to small numbers is how you confirm the filter is converging while tuning the ``Q``
.. and ``R`` values. Nothing in the system makes a decision from them.

.. ``raw_ticks`` is likewise a pass-through, carried only so that a log can be checked against
.. the untouched hardware count.

Note:

- The ``*_variance`` fields are the calculated variance of the Kalman Filter. You can confirm the
  convergence of the filter by observing these values settle to a small value while tuning the
  ``Q`` and ``R`` values.

.. note::

   The ``position`` field is **not** the Kalman-estimated position; it is the raw measured
   position, and only ``velocity`` and ``acceleration`` come from the filter. The reason is
   explained on the State Estimator page.