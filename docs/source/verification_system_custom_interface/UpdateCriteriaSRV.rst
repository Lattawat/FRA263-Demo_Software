claude_visualizer_interface/UpdateCriteria Service
---------------------------------------------------

File: ``claude_visualizer_interface/srv/UpdateCriteria.srv``

Service Definition
^^^^^^^^^^^^^^^^^^

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

.. A service is different from a topic. A topic is one-way and fire-and-forget: the publisher
.. never learns whether anyone listened. A service is a request/response pair, so the caller
.. gets an answer back and knows whether the work succeeded. Changing a pass/fail limit needs
.. that confirmation, which is why it is a service and not a topic.

.. The ``---`` line in the definition is the separator between the **request** part (above)
.. and the **response** part (below). Everything above ``---`` is what the client sends;
.. everything below is what the server sends back.

Use cases:

- At startup, the Web Visualizer sends an empty object ``{}`` to the Experiment Evaluator
  through an ``UpdateCriteria`` service call. On an empty object the Experiment Evaluator does not
  change any criteria, but sends the current criteria back to the Web Visualizer to use for visualization.
- When there is an update, the server checks every key in the request first — the key must be a 
  known criterion and the value must be a non-negative number. If any one entry fails, *nothing* 
  is applied and ``success`` is ``false``. This prevents a typo from leaving the criteria 
  half-updated, which would score the next run against a mixture of old and new limits.
- On a successful service call ``current_criteria_json`` always carries the full list of criteria.
  However, on a failed call, this field is empty, and the browser keeps whoeing what it already had.

Without this service the limits could only be changed by editing ``criteria.yaml`` and
restarting the node.