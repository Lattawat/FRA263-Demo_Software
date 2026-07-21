claude_visualizer_interface/EncoderRaw Message
----------------------------------------------

File: ``claude_visualizer_interface/msg/EncoderRaw.msg``

Raw Message Definition
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   # Raw quadrature encoder ticks published by micro-ROS on Teensy 4.1.
   # In Phase 1 this is published by mock_encoder.py instead.

   std_msgs/Header header          # ROS 2 standard header (timestamp + frame_id)
   int32   ticks                   # cumulative tick count (signed)
   float64 raw_position            # position converted from ticks [rad or m depending on setup]
   uint32  dt_us                   # microseconds since last sample

Compact Message Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   std_msgs/Header header
   int32 ticks
   float64 raw_position
   uint32 dt_us

Where it is used
^^^^^^^^^^^^^^^^

Published by the **Teensy 4.1 firmware** through micro-ROS when the system is run on hardware,
and by ``mock_ui`` when the system is run without hardware. Subscribed by the **State Estimator
node**.

.. This is the entry point of the whole measurement chain. Everything downstream —
.. the filtered states, every evaluation metric, every plot in the browser — is derived from
.. this message. If it stops arriving, the State Estimator has nothing to predict from and the
.. rest of the system goes silent.

.. Two fields need a note beyond their comment:

.. - ``ticks`` is **cumulative and signed**, not a per-sample delta. It keeps counting up or
..   down as the shaft turns, so the consumer works out movement by subtracting the previous
..   value.
.. - ``dt_us`` is currently **hard-coded to 10000** by the firmware (10 ms, i.e. 100 Hz)
..   rather than measured from the real elapsed time. The State Estimator therefore treats the
..   sample interval as fixed. See the Firmware page for why.

Note:

- ``dt_us`` is currently **hard-coded to 10000** by the firmware (10 ms, i.e. 100 Hz)
  rather than measured from the real elapsed time. The State Estimator therefore treats the
  sample interval as fixed.