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
Visualizer node**, which forwards both to the browser. The ``data`` field differs across
each experiment, which is possible because it is carried as a JSON string:

- ``eval_live`` — sent repeatedly while a run is in progress, throttled to 10 Hz. It updates
  the live evaluation panel, so the lecturer can observe the key evaluation information without
  waiting for the run to end. The throttle exists because the states arrive at 100 Hz and
  no browser needs to redraw that often.
- ``eval_summary`` — sent once, when the run finishes. This contains the summary information
  of that particular experiment with the pass/fail judgement for each requirement.

.. Both topics carry the same message type, which is why ``action`` matters: it tells the
.. receiver which of the four test types produced this message, and therefore how to read
.. ``data``. The same JSON-in-a-string approach as ``EventTrigger`` is used here, for the same
.. reason — each test aspect reports a different set of metrics, and a fixed field list could
.. not hold all of them.