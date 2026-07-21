claude_visualizer_interface/ActualStates Message
-------------------------------------------------

File: ``claude_visualizer_interface/msg/ActualStates.msg``

Raw Message Definition
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   # Actual robot state telemetry.
   # Phase 2: produced by mock_robot_controller.py via LSL and bridged into ROS 2 by
   # web_visualizer. Fields can be extended as the robot controller scope grows.

   std_msgs/Header header          # timestamp assigned by the source / bridge

   # Actual states (measured / computed by the controller)
   float64 actual_position         # actual position     [rad or m]
   float64 actual_velocity         # actual velocity     [rad/s or m/s]
   float64 actual_acceleration     # actual acceleration [rad/s² or m/s²]

Compact Message Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   std_msgs/Header header
   float64 actual_position
   float64 actual_velocity
   float64 actual_acceleration

Where it is used
^^^^^^^^^^^^^^^^

This is the actual robot data sent over LSL by the Base System or ``mock_ui`` (if the system is run 
without hardware), and turned into a ROS message by the **Web Visualizer node**. The Web Visualizer 
then subscribes to the very same topic it publishes on, and forwards whatever arrives to the browser. 
This data will be used for comparing the actual robot performance with the measured robot performance
(``/estimated_states``).

Publishing and subscribing to one topic in the same node looks strange at first, but it is
deliberate. It is a **loopback**: the node does not treat its own LSL bridge as the only
possible source. Any other producer on the network — ``mock_ui``, for example — can publish
``actual_states`` and the browser still receives it, because the browser is fed from the
subscription rather than from the bridge.

.. The importance of this message is comparison. ``EncoderState`` is what the *encoder*
.. measured; ``ActualStates`` is what the *controller believes it did*. Plotting the two
.. against each other is what exposes a controller that reports a move it never actually
.. completed.
