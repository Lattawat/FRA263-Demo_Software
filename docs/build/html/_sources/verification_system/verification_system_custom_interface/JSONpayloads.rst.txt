.. _event-payloads:

JSON payloads
-------------

This section provides the information about the JSON payload format of two fields,
``EventTrigger.event`` and ``ExperimentEval.data``.

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