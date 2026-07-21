claude_visualizer_interface/EventTrigger Message
-------------------------------------------------

File: ``claude_visualizer_interface/msg/EventTrigger.msg``

Raw Message Definition
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   # Trigger signal from robot_controller.py via LSL marker stream,
   # bridged into ROS 2 by web_visualizer node.

   std_msgs/Header header          # timestamp assigned by the bridge on receipt
   string  event                   # Full JSON payload string. Must contain an "event" key
                                   # (the type discriminator). All other keys are event-specific.

Compact Message Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   std_msgs/Header header
   string event

Where it is used
^^^^^^^^^^^^^^^^

This is the command/event signal sent over LSL by the Base System and ``mock_ui``, using the same
bridging concept as the ``/actual_states`` data. It is subscribed by the **Experiment Evaluator
node**, and by the Web Visualizer itself as a loopback so the browser sees every command.

The Experiment Evaluator uses this message to start the evaluation process. The data inside the
message is event data, telling the Experiment Evaluator which evaluation logic
and set of criteria to use for this session. The trade-off of this method is that
the fields are no longer checked by the ROS type system, so the keys are documented in
:ref:`event-payloads` below.

.. Published by the **Web Visualizer node** (bridging the LSL marker stream, plus two commands
.. the browser injects directly) and by **mock_ui**. Subscribed by the **Experiment Evaluator
.. node**, and by the Web Visualizer itself as a loopback so the browser sees every command.

.. This message is the evaluator's ears. The evaluator is event-driven: it sits idle and
.. measures nothing until an ``EventTrigger`` tells it a test has started, which test it is,
.. and with what settings. Without this topic the evaluator would never wake up, and no run
.. would ever be scored.

.. The ``event`` field is a single ``string``, but it carries a whole JSON object. This keeps
.. the message type stable — a new test type or a new setting is a new JSON key, not a
.. rebuild of the interface package and every node that depends on it. The trade-off is that
.. the fields are no longer checked by the ROS type system, so the keys are documented in
.. :ref:`event-payloads` below.

.. note::

   The comment in the ``.msg`` file says the JSON must contain an ``"event"`` key as the
   type discriminator. The system as built does not do this: every real payload uses
   ``mode`` and ``action`` as the discriminator pair instead, and the ``"event"`` key
   appears only in the fallback described below. Trust the payload tables, not the comment.
