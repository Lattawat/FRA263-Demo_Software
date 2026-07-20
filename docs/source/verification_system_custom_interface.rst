Custom Interface
================

.. The layout of each message section follows the ROS message documentation format,
   for example "https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html"

The Verification System does not use only the standard ROS 2 message types. It defines
its own **interface package**, ``claude_visualizer_interface``, which holds five message
types and one service type. This page is the reference for all of them: what each field
means, and which node produces or consumes it.

An interface package is a package that contains **only** definitions — no node code. It is
built with ``ament_cmake`` and the ``rosidl_generate_interfaces`` macro, which reads every
``.msg`` and ``.srv`` file listed in ``CMakeLists.txt`` and generates the matching C++ and
Python classes at build time:

.. code-block:: cmake

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/EncoderRaw.msg"
     "msg/EncoderState.msg"
     "msg/EventTrigger.msg"
     "msg/ActualStates.msg"
     "msg/ExperimentEval.msg"
     "srv/UpdateCriteria.srv"

     DEPENDENCIES builtin_interfaces std_msgs
   )

Keeping the definitions in a separate package is what lets very different programs agree on
the same data layout. The Teensy firmware is written in C and the four nodes are written in
Python, but both are generated from the same ``.msg`` file, so a message built on the
microcontroller can be unpacked by a Python node without anyone writing a converter.

In code the types are referred to by package and kind, for example
``claude_visualizer_interface/msg/EncoderRaw`` or
``claude_visualizer_interface/srv/UpdateCriteria``.

Summary
-------

.. list-table::
   :header-rows: 1
   :widths: 18 8 20 22 32

   * - Interface
     - Kind
     - Topic / service
     - Published by
     - Purpose
   * - ``EncoderRaw``
     - msg
     - ``encoder_raw``
     - Teensy firmware, ``mock_encoder``, ``mock_ui``
     - Raw encoder ticks straight from the hardware.
   * - ``EncoderState``
     - msg
     - ``estimated_states``
     - ``encoder_reader``
     - Kalman-filtered position, velocity and acceleration.
   * - ``ActualStates``
     - msg
     - ``actual_states``
     - ``web_visualizer``
     - What the robot controller reports about itself.
   * - ``EventTrigger``
     - msg
     - ``event_trigger``
     - ``web_visualizer``, ``mock_ui``
     - The command that starts, stops or skips a test.
   * - ``ExperimentEval``
     - msg
     - ``eval_live``, ``eval_summary``
     - ``experiment_evaluator``
     - Live metrics during a run, and the final pass/fail verdict.
   * - ``UpdateCriteria``
     - srv
     - ``update_criteria``
     - server: ``experiment_evaluator``
     - Read and change the pass/fail limits while the system is running.

Every topic name above is written as a **relative name** — no leading slash. A relative
name is expanded with the namespace the node was launched into, so under the launch
namespace ``/G7/`` the topic ``encoder_raw`` becomes ``/G7/encoder_raw``. This is what
allows several groups to run their own copy of the system on one network without their
topics colliding.

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

Published by the **Teensy 4.1 firmware** through micro-ROS, and by ``mock_encoder`` or
``mock_ui`` when the system is run without hardware. Subscribed by the **State Estimator
node** (``encoder_reader``), which is the only consumer.

This is the entry point of the whole measurement chain. Everything downstream —
the filtered states, every evaluation metric, every plot in the browser — is derived from
this message. If it stops arriving, the State Estimator has nothing to predict from and the
rest of the system goes silent.

Two fields need a note beyond their comment:

- ``ticks`` is **cumulative and signed**, not a per-sample delta. It keeps counting up or
  down as the shaft turns, so the consumer works out movement by subtracting the previous
  value.
- ``dt_us`` is currently **hard-coded to 10000** by the firmware (10 ms, i.e. 100 Hz)
  rather than measured from the real elapsed time. The State Estimator therefore treats the
  sample interval as fixed. See the Firmware page for why.

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
(which forwards it to the browser and pushes it back out on an LSL outlet) and by the
**Experiment Evaluator node**.

This message is the single source of motion truth for the rest of the system. The evaluator
computes *every* metric from it — error, overshoot, settling time, peak speed, peak
acceleration, precision spread — and the browser plots it live. An encoder gives position
only, so without the velocity and acceleration estimates carried here there would be no way
to score a speed or acceleration test at all.

The three ``*_variance`` fields are **diagnostics**, not control data. A variance is the
filter's own statement of how uncertain it is about that state, so watching these values
settle to small numbers is how you confirm the filter is converging while tuning the ``Q``
and ``R`` values. Nothing in the system makes a decision from them.

``raw_ticks`` is likewise a pass-through, carried only so that a log can be checked against
the untouched hardware count.

.. note::

   The ``position`` field does **not** carry the Kalman-estimated position. The State
   Estimator deliberately publishes the raw measured position instead, and only
   ``velocity`` and ``acceleration`` come from the filter. The reason is explained on the
   State Estimator page.

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

Produced by the robot controller on the Base System side, sent over LSL, and turned into a
ROS message by the **Web Visualizer node**. The Web Visualizer then subscribes to the very
same topic it publishes on, and forwards whatever arrives to the browser.

Publishing and subscribing to one topic in the same node looks strange at first, but it is
deliberate. It is a **loopback**: the node does not treat its own LSL bridge as the only
possible source. Any other producer on the network — ``mock_ui``, for example — can publish
``actual_states`` and the browser still receives it, because the browser is fed from the
subscription rather than from the bridge.

The importance of this message is comparison. ``EncoderState`` is what the *encoder*
measured; ``ActualStates`` is what the *controller believes it did*. Plotting the two
against each other is what exposes a controller that reports a move it never actually
completed.

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

Published by the **Web Visualizer node** (bridging the LSL marker stream, plus two commands
the browser injects directly) and by **mock_ui**. Subscribed by the **Experiment Evaluator
node**, and by the Web Visualizer itself as a loopback so the browser sees every command.

This message is the evaluator's ears. The evaluator is event-driven: it sits idle and
measures nothing until an ``EventTrigger`` tells it a test has started, which test it is,
and with what settings. Without this topic the evaluator would never wake up, and no run
would ever be scored.

The ``event`` field is a single ``string``, but it carries a whole JSON object. This keeps
the message type stable — a new test type or a new setting is a new JSON key, not a
rebuild of the interface package and every node that depends on it. The trade-off is that
the fields are no longer checked by the ROS type system, so the keys are documented in
:ref:`event-payloads` below.

.. note::

   The comment in the ``.msg`` file says the JSON must contain an ``"event"`` key as the
   type discriminator. The system as built does not do this: every real payload uses
   ``mode`` and ``action`` as the discriminator pair instead, and the ``"event"`` key
   appears only in the fallback described below. Trust the payload tables, not the comment.

claude_visualizer_interface/ExperimentEval Message
---------------------------------------------------

File: ``claude_visualizer_interface/msg/ExperimentEval.msg``

Raw Message Definition
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   std_msgs/Header header
   string action    # experiment type: point_to_point | pick_place | performance | precision
   string data      # JSON metrics (eval_live) or summary with pass/fail (eval_summary)

Compact Message Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   std_msgs/Header header
   string action
   string data

Where it is used
^^^^^^^^^^^^^^^^

Published by the **Experiment Evaluator node** on two topics, and subscribed by the **Web
Visualizer node**, which forwards both to the browser:

- ``eval_live`` — sent repeatedly while a run is in progress, throttled to 10 Hz. It drives
  the live evaluation panel, so the lecturer can watch the error shrinking instead of
  waiting for the run to end. The throttle exists because the states arrive at 100 Hz and
  no browser needs to redraw that often.
- ``eval_summary`` — sent once, when the run finishes. This is the pass/fail verdict, and
  it is the reason the whole Verification System exists.

Both topics carry the same message type, which is why ``action`` matters: it tells the
receiver which of the four test types produced this message, and therefore how to read
``data``. The same JSON-in-a-string approach as ``EventTrigger`` is used here, for the same
reason — each test aspect reports a different set of metrics, and a fixed field list could
not hold all of them.

claude_visualizer_interface/UpdateCriteria Service
---------------------------------------------------

File: ``claude_visualizer_interface/srv/UpdateCriteria.srv``

Raw Service Definition
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   string criteria_json
   ---
   bool success
   string message
   string current_criteria_json

Compact Service Definition
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   string criteria_json
   ---
   bool success
   string message
   string current_criteria_json

Where it is used
^^^^^^^^^^^^^^^^

The **Experiment Evaluator node** is the service **server** — the side that answers. The
**Web Visualizer node** is the **client** — the side that asks, on behalf of the browser.

A service is different from a topic. A topic is one-way and fire-and-forget: the publisher
never learns whether anyone listened. A service is a request/response pair, so the caller
gets an answer back and knows whether the work succeeded. Changing a pass/fail limit needs
that confirmation, which is why it is a service and not a topic.

The ``---`` line in the definition is the separator between the **request** part (above)
and the **response** part (below). Everything above ``---`` is what the client sends;
everything below is what the server sends back.

How it behaves:

- Sending an empty object ``{}`` changes nothing and simply reads the current limits back.
  The Web Visualizer does exactly this at startup to show the browser the current values.
- Validation is **all-or-nothing**. The server checks every key in the request first — the
  key must be a known criterion and the value must be a non-negative number. If any one
  entry fails, *nothing* is applied and ``success`` is ``false``. This prevents a typo from
  leaving the criteria half-updated, which would silently score the next run against a
  mixture of old and new limits.
- ``current_criteria_json`` always carries the **complete** criteria table, whether the
  update succeeded or not. The browser therefore never has to guess what state the server
  is in; it just displays what came back.

Without this service the limits could only be changed by editing ``criteria.yaml`` and
restarting the node, which would interrupt a demonstration in progress.

.. _event-payloads:

JSON payloads
-------------

Two fields — ``EventTrigger.event`` and ``ExperimentEval.data`` — are declared as plain
strings but actually carry JSON. The ROS type system cannot describe their contents, so
this section documents them.

EventTrigger commands
^^^^^^^^^^^^^^^^^^^^^

Every payload is a flat JSON object. Two keys are always present and act as the
**type discriminator** — the pair of values that tells the receiver what kind of message
this is:

- ``mode`` — which screen the operator was on: ``Manual``, ``Auto``, ``Test`` or ``STOP``
- ``action`` — the specific command

The Experiment Evaluator reads exactly these two keys to decide what to do, and ignores any
payload whose ``action`` it does not recognise.

Five of the seven commands are built by the Base System backend (``server_111.py``) and
travel to the Verification System over LSL. The remaining two, **stop** and
**skip_iteration**, are built by the Web Visualizer when a button is pressed in the browser
and are published straight onto ``event_trigger`` — they never make the LSL round trip.

.. list-table::
   :header-rows: 1
   :widths: 12 20 40 28

   * - ``mode``
     - ``action``
     - Extra keys
     - Origin
   * - ``Manual``
     - ``jog``
     - ``value``, ``direction``, ``timestamp``
     - Base System backend
   * - ``Auto``
     - ``point_to_point``
     - ``unit``, ``value``, ``timestamp``
     - Base System backend
   * - ``Auto``
     - ``pick_place``
     - ``order_sequence``, ``direction_sequence``, ``use_gripper``, ``num``, ``timestamp``
     - Base System backend
   * - ``Test``
     - ``performance``
     - ``speed``, ``accel``, ``timestamp``
     - Base System backend
   * - ``Test``
     - ``precision``
     - ``init_pos``, ``target_pos``, ``repeat``, ``unit``, ``timestamp``
     - Base System backend
   * - ``STOP``
     - ``stop``
     - —
     - Browser button
   * - *(none)*
     - ``skip_iteration``
     - —
     - Browser button

Several commands carry a ``unit`` key. It tells the evaluator how to convert the numbers
into radians before doing any maths:

- ``"index"`` — a hole number on the 72-hole plate, so one index is 360/72 = 5°
  (≈ 0.08727 rad)
- ``"degree"`` — degrees
- anything else — the value is assumed to be radians already

**Manual — jog.** A hand-driven nudge of the shaft. The evaluator does not score this
command; it is broadcast so the browser can show what the operator did.

.. code-block:: json

   {
     "mode": "Manual",
     "action": "jog",
     "value": 5,
     "direction": "CCW",
     "timestamp": 1753003812.4471
   }

**Auto — point_to_point.** Start a point-to-point test: move ``value`` away from wherever
the shaft is standing now.

.. code-block:: json

   {
     "mode": "Auto",
     "action": "point_to_point",
     "unit": "degree",
     "value": 90,
     "timestamp": 1753003845.1902
   }

**Auto — pick_place.** Start a pick-and-place test. ``order_sequence`` is the list of hole
indices to visit in order, and ``num`` is the number of pick/place pairs. ``use_gripper``
tells the robot whether to actually operate the gripper at each hole.

.. code-block:: json

   {
     "mode": "Auto",
     "action": "pick_place",
     "order_sequence": [0, 18, 36, 54],
     "direction_sequence": ["CCW", "CCW", "CW"],
     "use_gripper": true,
     "num": 2,
     "timestamp": 1753003901.7734
   }

.. warning::

   ``mock_ui`` sends this same list under the key ``directions``, while the Base System
   backend sends it as ``direction_sequence``. Nothing breaks today because no consumer
   reads either key — the evaluator only uses ``order_sequence`` — but a future consumer
   written against one spelling would silently miss the other. **The backend spelling,
   ``direction_sequence``, is the authoritative one.**

**Test — performance.** Start a performance test. The two values are the speed and
acceleration the robot was *commanded* to reach; the evaluator passes the run only if the
measured peaks reach them. If either key is missing, the evaluator falls back to the
``min_speed`` and ``min_acceleration`` limits from the criteria file.

.. code-block:: json

   {
     "mode": "Test",
     "action": "performance",
     "speed": 3.0,
     "accel": 4.0,
     "timestamp": 1753003950.0128
   }

**Test — precision.** Start a precision test: drive from ``init_pos`` to ``target_pos`` and
back again, ``repeat`` times, so the spread of the stopping positions can be measured.

.. code-block:: json

   {
     "mode": "Test",
     "action": "precision",
     "init_pos": 0,
     "target_pos": 18,
     "repeat": 10,
     "unit": "index",
     "timestamp": 1753004010.6650
   }

**Stop.** Ends whatever run is active and forces the evaluator to publish its summary
immediately. The evaluator accepts either ``"mode": "STOP"`` or ``"action": "stop"``, so
both spellings work.

.. code-block:: json

   {
     "mode": "STOP",
     "action": "stop"
   }

**Skip iteration.** Abandons the current waypoint or trial and moves on to the next one,
without ending the run. This exists for the case where the robot gets stuck and would
otherwise block the whole test.

.. code-block:: json

   {
     "action": "skip_iteration"
   }

**Fallback for non-JSON samples.** If a sample arrives on the LSL marker stream that is not
valid JSON, the Web Visualizer does not throw it away. It wraps the raw text instead, so a
plain text marker still reaches the system as a well-formed payload:

.. code-block:: json

   {
     "event": "LABEL",
     "label": "the raw text that arrived"
   }

The evaluator ignores this one, because ``action`` is missing.

ExperimentEval metrics
^^^^^^^^^^^^^^^^^^^^^^

The ``data`` field holds eight different shapes: one **live** shape and one **summary**
shape for each of the four test types. Which shape it is depends on the topic
(``eval_live`` or ``eval_summary``) and on the ``action`` field.

A live payload reports progress. This one is from a point-to-point run:

.. code-block:: json

   {
     "target_rad": 1.5708,
     "current_pos_rad": 1.4912,
     "current_error_rad": 0.0796,
     "elapsed_s": 0.842
   }

A summary payload reports the verdict, and always contains one or more ``pass_*`` boolean
fields. This is the matching point-to-point summary:

.. code-block:: json

   {
     "target_rad": 1.5708,
     "final_error_rad": 0.0041,
     "overshoot_pct": 6.213,
     "settling_time_s": 1.284,
     "pass_final_error": true,
     "pass_overshoot": true,
     "pass_settling": true
   }

The remaining six shapes are listed field by field, together with the algorithm that fills
them in, on the Experiment Evaluator page. They are not repeated here.

Notation
--------

Words used on this page.

interface package
    A ROS 2 package that contains only message and service definitions, no runnable code.
    Other packages depend on it to share one common data layout.

rosidl
    The ROS 2 code generator. It reads ``.msg`` and ``.srv`` files and writes the C++ and
    Python classes that nodes actually use, so the definition is written once.

field
    One named entry inside a message — a type and a name, such as ``float64 velocity``.

message vs service
    A **message** travels one way on a topic, and the sender never learns who received it.
    A **service** is a request/response pair: the caller waits and gets an answer back.

request / response
    The two halves of a service definition, separated by the ``---`` line. Above it is what
    the client sends, below it is what the server sends back.

payload
    The actual content carried inside a field — here, the JSON object packed into a
    ``string``.

type discriminator
    The key (or keys) a receiver reads first to work out what kind of payload it is holding,
    so it knows how to read the rest. In this system it is the ``mode`` + ``action`` pair.

loopback
    When a node subscribes to a topic it also publishes on. It is done on purpose here, so
    that messages from *any* producer — not only the node's own bridge — still reach the
    browser.

relative vs absolute topic name
    An **absolute** name starts with a slash (``/encoder_raw``) and is used exactly as
    written. A **relative** name has no leading slash (``encoder_raw``) and gets the node's
    namespace added in front, becoming ``/G7/encoder_raw`` under the namespace ``/G7/``.
    This system uses relative names everywhere so that several groups can run at once.

variance
    The filter's own measure of how uncertain it is about a state. A small variance means
    the estimate has converged; it is useful while tuning, but nothing in the system makes
    a decision from it.
