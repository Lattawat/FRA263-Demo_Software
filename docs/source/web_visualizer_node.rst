Web Visualizer Node (LSL ↔ ROS 2 Bridge + Web Server)
======================================================

.. Node description (the design idea, detail, and other crucial info)

This node is the connector of the Verification System. It links three worlds that cannot
talk to each other directly: **LSL** (the Base System's streams), **ROS 2** (the
verification pipeline), and the **browser** (the verifier dashboard). It is one process
(node name ``web_visualizer``) that runs **five workers** — five threads, one of which
runs an asyncio event loop — side by side. Together they carry four data flows, plus one
command flow coming back from the browser:

1. **ROS → LSL** : ``/estimated_states`` → LSL outlet ``EstimatedStates`` — the robot side
   can read the filtered states back.
2. **LSL → ROS** : LSL inlet ``ActualStates`` → ``/actual_states``.
3. **LSL → ROS** : LSL inlet ``EventTrigger`` → ``/event_trigger``.
4. **ROS → WebSocket** : every topic above (plus the evaluator's results) → JSON to the
   browser.
5. **Command flow** : the browser sends commands (zero, stop, skip, criteria update,
   sync) back over the same WebSocket.

.. code-block:: text

   [ from Base System ]                                          [ to robot side ]
   LSL "ActualStates"  ──────▶ ┌─────────────────────────┐ ────▶ LSL "EstimatedStates"
                               |                         |
   LSL "EventTrigger"  ──────▶ │                         │       [ to other ROS 2 nodes ]
                               │     web_visualizer      │ ────▶ /actual_states         (pub)
   [ from other ROS 2 nodes ]  │                         │ ────▶ /event_trigger         (pub)
   /estimated_states  ───────▶ │   LSL ↔ ROS 2 bridge    │ ────▶ /zero_estimated_states (pub)
   /actual_states     ───────▶ │      + web server       │ ────▶ update_criteria (service call)
   /event_trigger     ───────▶ │                         │
   /eval_live         ───────▶ │                         │        [ to browser ]
   /eval_summary      ───────▶ │                         │ ────▶ HTTP :8000      (dashboard files)
                               └─────────────────────────┘ ◀───▶ WebSocket :9090 (JSON ⇵ commands) 

**Parameters.** All set in ``params.yaml``:

.. list-table::
   :header-rows: 1
   :widths: 34 18 48

   * - Parameter
     - Value
     - Meaning
   * - ``ws_port``
     - ``9090``
     - Port of the WebSocket server (live JSON to the browser, commands back).
   * - ``http_port``
     - ``8000``
     - Port of the HTTP file server (the dashboard page itself).
   * - ``ws_host``
     - ``"0.0.0.0"``
     - Listen on every network interface, so another machine on the LAN can open the page.
   * - ``group_number``
     - ``"0"``
     - Multi-group isolation. Group ``N`` adds the suffix ``_N`` to every LSL stream name
       **and** its ``source_id``; ``"0"`` or ``""`` = no suffix.
   * - ``lsl_params.resolve_timeout_s``
     - ``5.0``
     - How long one attempt to find (resolve) an LSL stream on the network may take.
   * - ``lsl_params.actual_states_stream.name``
     - ``"ActualStates"``
     - Name of the LSL inlet carrying the robot's actual states.
   * - ``lsl_params.event_trigger_stream.name``
     - ``"EventTrigger"``
     - Name of the LSL inlet carrying one message per user action.
   * - ``lsl_params.estimated_states_stream.*``
     - see meaning
     - The LSL outlet: name ``EstimatedStates``, 3 float channels (position, velocity,
       acceleration), REGULAR rate **500 Hz**, ``source_id`` ``web_visualizer``.
   * - ``topics.*``
     - relative names
     - The ROS topic names the node uses: ``actual_states``, ``event_trigger``,
       ``estimated_states``.

**Interfaces.** This node has many interfaces, so they are grouped by kind:

- **ROS 2 subscriptions (5):**

  - ``/estimated_states`` (``EncoderState``) — the State Estimator's output; pushed to the
    LSL outlet and broadcast to the browser.
  - ``/actual_states`` (``ActualStates``) — broadcast to the browser.
  - ``/event_trigger`` (``EventTrigger``) — broadcast to the browser.
  - ``/eval_live``, ``/eval_summary`` (``ExperimentEval``) — the evaluator's live metrics
    and final summary; broadcast to the browser.

- **ROS 2 publishers (3):**

  - ``/actual_states`` — built from the LSL ``ActualStates`` samples.
  - ``/event_trigger`` — built from the LSL ``EventTrigger`` samples, and also from
    browser commands (stop, skip).
  - ``/zero_estimated_states`` (``std_msgs/Empty``) — the "re-zero" signal to the State
    Estimator node.

- **ROS 2 service client (1):** ``update_criteria`` (``UpdateCriteria``) — reads and
  updates the evaluator's pass/fail criteria.
- **LSL (2 inlets, 1 outlet):** inlets ``ActualStates`` and ``EventTrigger``; outlet
  ``EstimatedStates``.
- **Web (2 servers):** HTTP on port 8000 (dashboard files + ``/config.json``) and
  WebSocket on port 9090 (live JSON down, commands up).

Note that ``/actual_states`` and ``/event_trigger`` appear as **both** a publisher and a
subscription on purpose — the reason is explained in the Node Workflow section. As in the
other nodes, topic names are **relative**, so the launch namespace ``/G<N>/`` is added in
front, and the QoS profile is **RELIABLE** with **KEEP_LAST depth 10**.

**JSON wire format.** Every message to the browser uses one envelope shape — a
``topic`` name plus a ``data`` object holding the fields of the ROS message:

.. code-block:: text

   {"topic": "estimated_states",
    "data": {"stamp": 1721.4, "position": 0.52, "velocity": 1.10, "acceleration": 0.03, ...}}

The node keeps a cache (the stored last message of every topic). A browser that connects
late immediately receives that snapshot, so the dashboard never starts empty.

Node Workflow
-------------

.. The flow chart of this whole node

The constructor does not process any data itself — it starts the five workers and then
hands its own thread to ``rclpy.spin``. Each worker has one job, and they meet in two
places: **ROS topics** (where data enters the ROS world) and the **WebSocket broadcast**
(where data leaves to the browser). Each worker below is roughly the size of the whole
State Estimator node, so take them one at a time.

The big picture
^^^^^^^^^^^^^^^

.. code-block:: text

   [thread: lsl-actual-states]                          (Flow 2 — LSL → ROS)
   LSL inlet "ActualStates" ──pull──▶ deg→rad, − actual offset ──▶ publish /actual_states

   ──────────────────────────────────────────────────────────────────────────────────────

   [thread: lsl-event-trigger]                          (Flow 3 — LSL → ROS)
   LSL inlet "EventTrigger" ──pull──▶ parse the JSON string ──▶ publish /event_trigger

   ──────────────────────────────────────────────────────────────────────────────────────

   [thread: main — rclpy.spin]                          (Flow 1 + Flow 4 — the fan-out hub)
   /estimated_states ──▶ _estimated_states_cb ──┬─▶ push LSL outlet "EstimatedStates"
                                                └─▶ broadcast
   /actual_states    ──▶ _actual_states_cb  ──────▶ broadcast
   /event_trigger    ──▶ _event_trigger_cb  ──────▶ broadcast
   /eval_live        ──▶ _eval_live_cb      ──────▶ broadcast
   /eval_summary     ──▶ _eval_summary_cb   ──────▶ broadcast

   ──────────────────────────────────────────────────────────────────────────────────────

   [thread: ws-server — the asyncio event loop]        (Flow 4 + the command flow)
   broadcast ──▶ send the JSON envelope to every connected browser
   browser command ──▶ _handle_command ──▶ publish / service call / change offset

   ──────────────────────────────────────────────────────────────────────────────────────

   [thread: http-server]
   browser GET :8000 ──▶ dashboard files ;  GET /config.json ──▶ {"ws_port": 9090}

All five threads run at the same time. The two LSL workers feed data **into** ROS; the
main thread fans everything **out** of ROS; the WebSocket thread owns the browser
connections; the HTTP thread only serves files.

ROS callback worker (the fan-out hub)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The main thread sits in ``rclpy.spin`` and runs the five subscription callbacks. Every
callback does the same small job: serialize the ROS message into the JSON envelope and
hand it to ``_broadcast``. Only ``_estimated_states_cb`` has a second job — it also pushes
``[position, velocity, acceleration]`` into the LSL outlet ``EstimatedStates`` so the
robot side can read the filtered states.

The key design decision here: the LSL workers **publish into ROS and never broadcast
directly**. Because the node subscribes to ``/actual_states`` and ``/event_trigger`` —
the same topics it publishes — every message passes through a subscription callback on
its way to the browser. The result is a single broadcast path: it does not matter whether
a message was produced by the LSL worker, by ``mock_ui``, or by a future robot node — it
reaches the browser the same way.

LSL inlet workers
^^^^^^^^^^^^^^^^^

The two LSL threads (``lsl-actual-states`` and ``lsl-event-trigger``) run the same loop
shape; only the message building differs:

.. code-block:: text

   inlet = None
        │
        ▼
   while the stop flag is not set:
        │
        ├─ inlet is None?  ──▶ resolve the stream by name (timeout 5 s)
        │                      not found → warn, try again next pass
        │
        ├─ pull one sample from the inlet (timeout 1 s)
        │      error   → drop the inlet, re-resolve next pass
        │      nothing → loop again
        │
        └─ sample arrived:
               build the ROS message  ──▶  publish
               (no WS broadcast here — the subscription callback does it)

The re-resolve step makes the workers self-healing: if the Base System restarts, the old
inlet fails, the worker throws it away, and the next pass finds the new stream. The
differences between the two workers:

- **ActualStates** — a sample is 3 floats. The position arrives in degrees, so it is
  converted to radians and the local actual-side offset (``_actual_pos_offset_rad``) is
  subtracted before publishing ``/actual_states``.
- **EventTrigger** — a sample is 1 string containing JSON. It is parsed (with a fallback
  for a plain string) and re-packed into the ``event`` field of ``/event_trigger``.

WebSocket server worker
^^^^^^^^^^^^^^^^^^^^^^^

This thread runs its own **asyncio event loop** — a scheduler that serves many browser
connections in one thread. Each new connection goes through the same steps:

.. code-block:: text

   browser connects to ws://<host>:9090
        │
        ▼
   replay the cached last message of every topic     (instant picture)
        │
        ▼
   send the criteria snapshot                        (service call to the evaluator)
        │
        ▼
   add the client to the broadcast set
        │
        ▼
   wait for messages from the browser ──▶ _handle_command(...)
        │ connection closed
        ▼
   remove the client from the broadcast set

Outgoing data crosses a thread boundary here: the ROS callbacks run in the main thread,
but the WebSocket objects live in this thread's event loop. ``_broadcast`` bridges the
two safely with ``asyncio.run_coroutine_threadsafe`` — it hands the send job **to** the
event loop instead of touching the sockets from the wrong thread. This is what makes the
broadcast thread-safe.

HTTP server worker
^^^^^^^^^^^^^^^^^^

The simplest worker. It serves the dashboard files (HTML/JS/CSS installed in the
package's ``web/`` folder) on port 8000, using Python's built-in file server. It answers
one special path itself: ``GET /config.json`` returns ``{"ws_port": 9090}``, so the
browser can discover the right WebSocket port from the same machine that served the page
— no hardcoded port in the JavaScript.

Browser commands
^^^^^^^^^^^^^^^^

Commands arrive as JSON over the WebSocket and are handled by ``_handle_command`` (a
dispatcher — one ``if/elif`` chain keyed on the ``command`` field):

.. list-table::
   :header-rows: 1
   :widths: 24 76

   * - Command
     - What the node does
   * - ``time_sync``
     - Replies with the node's current ROS time, so the browser can align its chart
       timestamps with the data stamps.
   * - ``stop_experiment``
     - Publishes an ``EventTrigger`` with ``{"mode": "STOP", "action": "stop"}`` — the
       evaluator sees it and ends the run.
   * - ``skip_iteration``
     - Publishes an ``EventTrigger`` with ``{"action": "skip_iteration"}`` — the evaluator
       drops the stuck waypoint / trial.
   * - ``criteria_update``
     - Calls the ``update_criteria`` service with the new limits, then broadcasts a
       ``criteria_ack`` to **every** browser so all open dashboards stay in sync.
   * - ``pos_sync``
     - Adds ``delta_rad`` to the actual-side position offset.
   * - ``zero_set``
     - Two sides at once. Estimated side: publishes ``Empty`` on
       ``/zero_estimated_states`` so the State Estimator re-zeros at the source. Actual
       side: adds the current actual position to the local offset, because
       ``/actual_states`` is produced inside this node.

Examine the code
----------------

.. referencing the section 2.1 of the mentioned link

The full node lives in ``scripts/web_visualizer.py`` (about 600 lines). This section walks
through every mechanism, but compresses the repeated parts: only one of the four JSON
serializers is shown, the parameter block is covered by the table above, and commented-out
legacy lines are trimmed from the excerpts.

**Imports.**

.. code-block:: python

   import asyncio
   import functools
   import http.server
   import json
   import math
   import os
   import threading

   from ament_index_python.packages import get_package_share_directory

   import rclpy
   from rclpy.node import Node
   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

   import pylsl
   import websockets

   from std_msgs.msg import Empty

   from claude_visualizer_interface.msg import EncoderState, ActualStates, EventTrigger, ExperimentEval
   from claude_visualizer_interface.srv import UpdateCriteria
   from claude_visualizer.utils import create_outlet

This node talks to everything, so it needs more libraries than the others: ``asyncio`` +
``websockets`` for the WebSocket server, ``http.server`` for the file server,
``threading`` for the workers, ``pylsl`` for the LSL inlets/outlet, and
``get_package_share_directory`` to find the installed ``web/`` folder. From the project's
interface package it uses four message types and the ``UpdateCriteria`` service, plus the
``create_outlet`` helper that builds an LSL outlet from a settings dictionary.

**Serializers — ROS message → JSON envelope.**

.. code-block:: python

   def _stamp_to_sec(stamp) -> float:
       return stamp.sec + stamp.nanosec * 1e-9


   def estimated_states_to_json(msg: EncoderState) -> dict:
       return {
           "topic": "estimated_states",
           "data": {
               "stamp":        _stamp_to_sec(msg.header.stamp),
               "position":     msg.position,
               "velocity":     msg.velocity,
               "acceleration": msg.acceleration,
               "pos_variance": msg.pos_variance,
               "vel_variance": msg.vel_variance,
               "acc_variance": msg.acc_variance,
               "raw_ticks":    msg.raw_ticks,
           },
       }

A ROS time stamp is two integers (seconds + nanoseconds); ``_stamp_to_sec`` folds them
into one float that JavaScript can use directly. The serializer copies the message fields
into the ``{"topic": ..., "data": ...}`` envelope. There are three more serializers with
exactly the same shape — ``actual_states_to_json``, ``event_trigger_to_json``, and
``_eval_to_json``. The last two also ``json.loads`` the JSON string embedded in the
message, with a fallback, so one malformed string cannot crash the pipeline.

**The HTTP handler and** ``/config.json``.

.. code-block:: python

   class _SilentHTTPHandler(http.server.SimpleHTTPRequestHandler):
       def log_message(self, *args): pass

       # Serve the runtime WebSocket port to the browser so app.js no longer needs a
       # hardcoded port. ws_port is attached to the server object in _run_http_server.
       def do_GET(self):
           if self.path == "/config.json":
               body = json.dumps({"ws_port": self.server.ws_port}).encode()
               self.send_response(200)
               self.send_header("Content-Type", "application/json")
               self.send_header("Content-Length", str(len(body)))
               self.end_headers()
               self.wfile.write(body)
               return
           return super().do_GET()

A small subclass of Python's built-in file handler. ``log_message`` is overridden to
silence the one-line-per-request console spam. ``do_GET`` intercepts exactly one path:
``/config.json`` is answered by the handler itself with the runtime WebSocket port; every
other path falls through to normal file serving. This is how the browser finds the
WebSocket server without a hardcoded port in the JavaScript.

**Constructor — the group suffix.** The parameter declarations follow the same
``declare_parameter`` / ``get_parameter`` pattern as the State Estimator (values in the
table above), so the excerpt starts at the interesting part:

.. code-block:: python

       group_number = str(self.get_parameter("group_number").value or "").strip()
       _suf = (lambda n: f"{n}_{group_number}") if group_number and group_number != "0" else (lambda n: n)
       actual_states_stream  = _suf(self.get_parameter("lsl_params.actual_states_stream.name").value)
       event_trigger_stream  = _suf(self.get_parameter("lsl_params.event_trigger_stream.name").value)

``_suf`` is a tiny helper function: for group ``N`` (not ``0``) it turns a stream name
into ``name_N``; for group 0 it changes nothing. It is applied to both inlet names here,
and later to the outlet's name **and** ``source_id``. This is the whole multi-group
mechanism on the LSL side — each verifier resolves only its own robot's streams, even on
a shared LAN.

**Constructor — WebSocket state and starting the threads.**

.. code-block:: python

       # ── WebSocket server state ──────────────────────────────────────────
       self._ws_clients = set()                 # touched only by ws thread's loop
       self._ws_last_messages = {}              # topic → last serialized JSON str

       self._ws_loop_scheduler = asyncio.new_event_loop()
       #functionally, the AbstractEventLoop acts as a scheduler for asynchronous tasks.
       #However, it is called a "loop" because its internal mechanism is literally
       #a continuous while loop that keeps the program alive.

       self._ws_thread_exec = threading.Event()
       self._ws_thread = threading.Thread(
           target=self._run_ws_loop, daemon=True, name="ws-server",
       )
       self._ws_thread.start()
       self._ws_thread_exec.wait()

       self._http_thread = threading.Thread(
           target=self._run_http_server, daemon=True, name="http-server",
       )
       self._http_thread.start()

Two shared pieces of state first: the set of connected clients and the last-message cache.
``asyncio.new_event_loop()`` creates the event loop but does not run it — running it is
the ws thread's job. Then comes a **handshake** with ``threading.Event``: the constructor
starts the ws thread and immediately blocks on ``wait()``; the ws thread calls ``set()``
only after the WebSocket server is actually listening. This guarantees the order — nothing
later in the constructor can try to broadcast before the server exists. The HTTP server
gets its own thread because ``serve_forever`` is blocking. Both threads are ``daemon``
threads, so they die automatically with the process.

**Constructor — ROS wiring.**

.. code-block:: python

       # ── Estimated States data flow: ROS → LSL (subscribe to /estimated_states, push to LSL) ────
       self._estimated_states_outlet = create_outlet(estimated_states_lsl_stream_info, pylsl.cf_float32)
       self.create_subscription(EncoderState, topic_encoder, self._estimated_states_cb, qos)

       # ── Actual States and Trigger data flow: LSL → ROS (pull from LSL inlet, publish to ROS) ─────
       self._actual_states_pub = self.create_publisher(ActualStates, actual_states_topic, qos)
       self._event_trigger_pub = self.create_publisher(EventTrigger, event_trigger_topic,  qos)
       # Tells the encoder_reader (Kalman) node to re-zero /estimated_states at the
       # source, so the evaluator + LSL outlet + WS all share one zeroed frame.
       self._zero_estimated_pub = self.create_publisher(Empty, "zero_estimated_states", qos)

       # Also subscribe to these topics so that ANY publisher (our LSL worker
       # OR mock_encoder OR a future robot_controller ROS node) gets
       # broadcast to the WebSocket clients. LSL workers therefore do NOT
       # broadcast directly — the subscription callbacks do.
       self.create_subscription(ActualStates,    actual_states_topic,  self._actual_states_cb,  qos)
       self.create_subscription(EventTrigger,    event_trigger_topic,  self._event_trigger_cb,  qos)
       self.create_subscription(ExperimentEval,  "eval_live",          self._eval_live_cb,      qos)
       self.create_subscription(ExperimentEval,  "eval_summary",       self._eval_summary_cb,   qos)

       # Relative service name → resolves to /G<N>/update_criteria, matching the evaluator's server.
       self._criteria_client = self.create_client(UpdateCriteria, "update_criteria")
       self._last_criteria_json = "{}"

       # Actual-side zero offset stays here because /actual_states is produced
       # locally (LSL worker). The estimated-side zero now lives in the Kalman
       # node so it applies upstream of every consumer.
       self._actual_pos_offset_rad = 0.0

This block wires every flow. ``create_outlet`` builds the LSL outlet from the settings
dictionary (Flow 1), and the publishers cover Flows 2 and 3 plus the re-zero signal. The
comment in the middle explains the fan-out design: the node subscribes to the very topics
it publishes, so the subscription callbacks are the **only** place that broadcasts to the
browser. The criteria client uses a relative service name for the same namespace reason
as the topics. Finally, the actual-side zero offset lives here (the actual data is
produced in this node), while the estimated-side zero lives in the State Estimator.

**Constructor — starting the LSL workers.**

.. code-block:: python

       self._lsl_threads_exec = threading.Event()
       self._lsl_threads = [
           threading.Thread(
               target=self._actual_states_worker,
               args=(actual_states_stream, resolve_timeout),
               daemon=True, name="lsl-actual-states",
           ),
           threading.Thread(
               target=self._event_trigger_worker,
               args=(event_trigger_stream, resolve_timeout),
               daemon=True, name="lsl-event-trigger",
           ),
       ]
       for t in self._lsl_threads: #activate all _lsl_threads to receive the stream
           t.start()

A second ``threading.Event``, this time used as a **stop flag**: the worker loops run
``while not self._lsl_threads_exec.is_set()``, so setting it later (in ``destroy_node``)
is how the node asks both loops to finish. Each worker gets its stream name (already
suffixed by ``_suf``) and the resolve timeout as arguments.

**The HTTP server thread.**

.. code-block:: python

   def _run_http_server(self) -> None:
       web_dir = os.path.join(
           get_package_share_directory("claude_visualizer"), "web"
       )
       handler = functools.partial(_SilentHTTPHandler, directory=web_dir)
       self._http_server = http.server.HTTPServer(("0.0.0.0", self._http_port), handler)
       self._http_server.ws_port = self._ws_port   # exposed to the browser via GET /config.json
       self._http_server.serve_forever()

The dashboard files are installed into the package's share directory, so
``get_package_share_directory`` finds them wherever the workspace lives.
``functools.partial`` pre-fills the ``directory`` argument of the handler class. The
``ws_port`` is attached to the server object — that is where the ``/config.json`` route
reads it from. ``serve_forever`` then blocks this thread for the life of the node.

**The WebSocket thread and its event loop.**

.. code-block:: python

   def _run_ws_loop(self) -> None:
       asyncio.set_event_loop(self._ws_loop_scheduler) #tell the asyncio that self._ws_loop_scheduler is the asyncio event loop
       self._ws_loop_scheduler.run_until_complete(self._start_ws_server())
       self._ws_thread_exec.set()
       self._ws_loop_scheduler.run_forever()

   async def _start_ws_server(self) -> None:
       self._ws_server = await websockets.serve(
           self._handle_client, self._ws_host, self._ws_port,
       )
       self._ws_bound_addresses = [
           sock.getsockname() for sock in self._ws_server.sockets
       ]

``set_event_loop`` pins the event loop to this thread. ``run_until_complete`` runs one
coroutine — starting the server — and returns when it is up. Only then is the handshake
Event ``set()``, releasing the constructor. ``run_forever`` keeps the loop alive to serve
clients and to run every broadcast job handed over from the other threads.
``websockets.serve`` registers ``_handle_client`` as the callback that runs **once per
browser connection**; the bound addresses are recorded for the startup log message.

**One client's life —** ``_handle_client``.

.. code-block:: python

   async def _handle_client(self, websocket) -> None:
       # Send the last known value for each topic so the browser has an
       # immediate picture instead of waiting for the next tick.
       for cached in self._ws_last_messages.values():
           await websocket.send(cached)

       snapshot = await self._get_criteria_snapshot()
       if snapshot:
           await websocket.send(snapshot)

       self._ws_clients.add(websocket)
       try:
           async for raw in websocket:
               try:
                   await self._handle_command(json.loads(raw))
               except Exception:
                   pass
       except websockets.ConnectionClosed:
           pass
       finally:
           self._ws_clients.discard(websocket)

The connect steps from the workflow diagram, in code: replay the cache, send the criteria
snapshot, register the client. ``async for raw in websocket`` then waits for incoming
messages — each one is a command. The inner ``try/except`` swallows a broken command so
one typo from the browser cannot close the connection; the ``finally`` guarantees the
client is removed from the broadcast set no matter how the connection ends.

**The thread-safe broadcast bridge.**

.. code-block:: python

   def _broadcast(self, payload: dict) -> None:
       """Thread-safe: called from ROS/LSL threads. Pushes JSON to all clients."""
       message = json.dumps(payload)
       self._ws_last_messages[payload["topic"]] = message
       if self._ws_clients:
           asyncio.run_coroutine_threadsafe(
               self._broadcast_async(message), self._ws_loop_scheduler,
           )

   async def _broadcast_async(self, message: str) -> None:
       for client in list(self._ws_clients):
           try:
               await client.send(message)
           except Exception as e:
               self.get_logger().warn(f"[_broadcast_async] send failed: {type(e).__name__}: {e}")
               self._ws_clients.discard(client)

This pair is the bridge between the threads. ``_broadcast`` is called from the ROS
callbacks (main thread), but the WebSocket objects belong to the event loop in the ws
thread — touching them directly from another thread would corrupt them.
``run_coroutine_threadsafe`` solves this: it hands the coroutine **to** the event loop
and lets the loop run it in its own thread. Note the cache is updated even when no client
is connected, so a browser that arrives later still gets the newest values.
``_broadcast_async`` iterates over a **copy** of the client set (``list(...)``) so it can
safely drop a dead client while looping.

**Reading the criteria —** ``_get_criteria_snapshot``.

.. code-block:: python

   async def _get_criteria_snapshot(self) -> str | None:
       if not self._criteria_client.wait_for_service(timeout_sec=0.5):
           return None
       req = UpdateCriteria.Request()
       req.criteria_json = "{}"
       future = self._criteria_client.call_async(req)
       while not future.done():
           await asyncio.sleep(0.05)

       result = future.result()
       if result and result.success:
           self._last_criteria_json = result.current_criteria_json
           return json.dumps({
               "topic": "criteria_snapshot",
               "data": json.loads(result.current_criteria_json)
           })

       return None

The trick here is calling ``update_criteria`` with an **empty** ``"{}"`` — change
nothing, just return the current criteria. ``call_async`` returns a *future* (a result
that is not ready yet); the ``while`` + ``await asyncio.sleep`` polls it gently, so the
event loop stays free to serve other clients while waiting. A plain blocking wait here
would freeze every WebSocket connection. If the evaluator is not up within 0.5 s, the
snapshot is simply skipped.

**The command dispatcher —** ``_handle_command``. First the simple commands:

.. code-block:: python

   async def _handle_command(self, cmd: dict) -> None:
       command = cmd.get("command")
       if command == "time_sync":
           ref_sec = self.get_clock().now().nanoseconds / 1e9
           payload = json.dumps({"topic": "time_sync", "data": {"ref_stamp": ref_sec}})
           self._ws_last_messages["time_sync"] = payload
           await self._broadcast_async(payload)
       elif command == "stop_experiment":
           msg = EventTrigger()
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = "browser"
           msg.event = json.dumps({"mode": "STOP", "action": "stop"})
           self._event_trigger_pub.publish(msg)
       elif command == "skip_iteration":
           msg = EventTrigger()
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = "browser"
           msg.event = json.dumps({"action": "skip_iteration"})
           self._event_trigger_pub.publish(msg)

``time_sync`` answers with the node's current ROS time so the browser can align its chart
axis with the data stamps. ``stop_experiment`` and ``skip_iteration`` are turned into
normal ``EventTrigger`` messages — with ``frame_id = "browser"`` marking where they came
from — and published on the same ``/event_trigger`` topic the evaluator already listens
to. Reusing the existing topic means the evaluator needs no extra input channel for
browser actions.

Then the stateful commands:

.. code-block:: python

       elif command == "criteria_update":
           data = cmd.get("data", {})
           if not self._criteria_client.wait_for_service(timeout_sec=0.5):
               return
           req = UpdateCriteria.Request()
           req.criteria_json = json.dumps(data)
           future = self._criteria_client.call_async(req)
           while not future.done():
               await asyncio.sleep(0.05)
           result = future.result()
           acknowledgement = {"topic": "criteria_ack", "data": {
                          "success": result.success,
                          "message": result.message,
                       }}
           if result.success:
               self._last_criteria_json = result.current_criteria_json
               acknowledgement["data"]["criteria"] = json.loads(result.current_criteria_json)

           await self._broadcast_async(json.dumps(acknowledgement))
       elif command == "pos_sync":
           delta = float(cmd.get("data", {}).get("delta_rad", 0.0))
           self._actual_pos_offset_rad += delta
       elif command == "zero_set":
           act_now = float(cmd.get("data", {}).get("act_rad", 0.0))
           # Estimated side: ask the Kalman node to re-zero at the source so the
           # evaluator and LSL outlet stay in the same frame as the display.
           self._zero_estimated_pub.publish(Empty())
           # Actual side: /actual_states is produced here, so offset it locally.
           self._actual_pos_offset_rad += act_now

``criteria_update`` forwards the new limits to the evaluator through the service (same
polling pattern as the snapshot) and then broadcasts a ``criteria_ack`` to **every**
client — so if two browsers are open, both see the new limits, not just the one that
changed them. ``pos_sync`` nudges the actual-side offset by a delta. ``zero_set`` splits
the zeroing in two, exactly as the comments say: the estimated side is re-zeroed at the
source (one ``Empty`` message to the State Estimator), and the actual side is offset
locally because ``/actual_states`` is built inside this node.

**The ROS subscription callbacks.**

.. code-block:: python

   def _estimated_states_cb(self, msg: EncoderState) -> None:
       # msg.position already arrives zeroed from the Kalman node, so the LSL
       # outlet and the WS broadcast both carry the same frame, no patching here.
       self._estimated_states_outlet.push_sample(
           [float(msg.position), float(msg.velocity), float(msg.acceleration)],
           timestamp=_stamp_to_sec(msg.header.stamp),
       )
       self._broadcast(estimated_states_to_json(msg))

   def _actual_states_cb(self, msg: ActualStates) -> None:
       self._broadcast(actual_states_to_json(msg))

``_estimated_states_cb`` is the only callback with two jobs: push the three states into
the LSL outlet (using the ROS stamp as the LSL timestamp) and broadcast the full message
to the browser. The position needs no correction here — it already arrives zeroed from
the State Estimator. The other four callbacks all look like ``_actual_states_cb``: one
line, serialize and broadcast.

**Pulling from LSL —** ``extract_sample_from_lsl_inlet`` **and** ``_resolve_stream``.

.. code-block:: python

   def extract_sample_from_lsl_inlet(self, inlet, name: str, timeout: float):
       """Pull one sample. inlet persists across calls via the caller's local variable.

       Returns (sample, inlet):
         - sample is None if nothing arrived or an error occurred
         - inlet is None when the stream needs to be re-resolved next call
       """
       if inlet is None:
           inlet = self._resolve_stream(name, timeout)
           if inlet is None:
               return None, None
       try:
           sample, _ = inlet.pull_sample(timeout=1.0)
       except Exception as e:
           self.get_logger().warn(f"[{name}] pull_sample error: {e}")
           return None, None  # inlet=None triggers re-resolve next iteration

       if sample is None:
           return None, inlet

       if type(sample[0]) == float:
           if len(sample) < len(self.actual_states_channel):
               self.get_logger().warn(
                   f"[actual_states] expected {len(self.actual_states_channel)} channels, got {len(sample)}"
               )
               return None, inlet
           return sample, inlet

       elif type(sample[0]) == str:
           try:
               return json.loads(sample[0]), inlet
           except (TypeError, ValueError):
               return {"event": "LABEL", "label": str(sample[0])}, inlet

       else:
           self.get_logger().info("Sample type not match with the condition")
           return None, inlet

Both LSL workers call this helper once per pass. The **lazy resolve**: if there is no
inlet yet, find the stream first. If ``pull_sample`` raises an error, the helper returns
``inlet=None`` — the caller stores that, and the next pass resolves again. This is the
self-healing behaviour from the workflow section. The sample is then handled by type: a
float first channel means a numeric ``ActualStates`` sample (with a channel-count sanity
check); a string first channel means an ``EventTrigger`` JSON payload, parsed with a
fallback that wraps a plain string as a label event.

.. code-block:: python

   def _resolve_stream(self, name: str, timeout: float):
       self.get_logger().info(f"Resolving LSL stream '{name}' (timeout={timeout}s)…")
       streams = pylsl.resolve_byprop("name", name, timeout=timeout)
       if not streams:
           self.get_logger().warn(
               f"LSL stream '{name}' not found — will keep retrying"
           )
           return None
       self.get_logger().info(f"LSL stream '{name}' resolved.")
       return pylsl.StreamInlet(streams[0])

Resolving = searching the network for a stream whose ``name`` property matches, for at
most ``timeout`` seconds (5 s from ``params.yaml``). If found, an inlet is opened on the
first match.

**The two worker loops.**

.. code-block:: python

   def _actual_states_worker(self, name: str, timeout: float) -> None:
       inlet = None
       while not self._lsl_threads_exec.is_set():
           sample, inlet = self.extract_sample_from_lsl_inlet(inlet, name, timeout)
           if sample is not None:
               msg = ActualStates()
               msg.header.stamp = self.get_clock().now().to_msg()
               msg.header.frame_id = "robot_controller"
               msg.actual_position     = float(sample[0]) * _DEG_TO_RAD - self._actual_pos_offset_rad
               msg.actual_velocity     = float(sample[1])
               msg.actual_acceleration = float(sample[2])
               self._actual_states_pub.publish(msg)
               # No _broadcast here — the ROS subscription callback handles it.

   def _event_trigger_worker(self, name: str, timeout: float) -> None:
       inlet = None
       while not self._lsl_threads_exec.is_set():
           json_payload, inlet = self.extract_sample_from_lsl_inlet(inlet, name, timeout)

           if json_payload is not None:
               msg = EventTrigger()
               msg.header.stamp    = self.get_clock().now().to_msg()
               msg.header.frame_id = "robot_controller"
               msg.event = json.dumps(json_payload)
               self.get_logger().info(
                   f"[EventTrigger] event={json_payload.get('event')} payload={msg.event}"
               )
               self._event_trigger_pub.publish(msg)
               # No _broadcast here — the ROS subscription callback handles it.

The same skeleton twice: loop until the stop flag is set, pull one sample per pass,
publish if something arrived. The actual-states worker converts the position from degrees
to radians (``_DEG_TO_RAD``) and subtracts the actual-side offset; the message is stamped
with the node's current time and marked ``frame_id = "robot_controller"`` to record where
the data came from. The event worker re-packs the parsed payload as a JSON string in the
``event`` field and logs each event. Both end at ``publish`` — the "No ``_broadcast``
here" comments are the single-broadcast-path rule from the workflow section.

**Shutdown —** ``destroy_node``.

.. code-block:: python

   def destroy_node(self):
       self._lsl_threads_exec.set()
       for t in self._lsl_threads:
           t.join(timeout=1.5)
       try:
           self._ws_loop_scheduler.call_soon_threadsafe(self._ws_loop_scheduler.stop)
       except Exception:
           pass
       try:
           self._http_server.shutdown()
       except Exception:
           pass
       super().destroy_node()

The constructor in reverse. Setting the stop flag ends both LSL ``while`` loops, and
``join`` waits (up to 1.5 s each) for them to finish. The event loop must be stopped
from its own thread, so — the same rule as broadcasting — the stop request is handed
over with ``call_soon_threadsafe``. The HTTP server has its own ``shutdown()``. Finally
the base class cleans up the ROS side. ``main`` is the same standard entry point as on
the :doc:`State Estimator page <state_estimator_node>`, so it is not repeated here.

Notation
--------

Words used on this page.

thread / daemon thread
    A worker running inside the same process, sharing its memory. A **daemon** thread is
    killed automatically when the program exits, so it never blocks shutdown.

event loop (asyncio)
    The scheduler that runs many small async tasks inside one thread. It is called a
    "loop" because its internal mechanism is literally a continuous ``while`` loop.

coroutine / async function
    A function (``async def``) that the event loop can pause at every ``await`` and
    resume later, so one thread can serve many clients.

callback
    A function that runs automatically when something arrives — a ROS message, or a new
    WebSocket connection.

thread-safe
    Safe to call from another thread without corrupting shared data. Here it is done by
    handing work to the event loop with ``asyncio.run_coroutine_threadsafe``.

serializer / JSON envelope
    A serializer turns a ROS message into JSON. The envelope is the fixed outer shape
    ``{"topic": ..., "data": ...}`` that every message to the browser uses.

inlet / outlet (LSL)
    The two ends of an LSL stream: a source publishes through an **outlet**, a reader
    receives through an **inlet**.

resolve (LSL)
    Searching the network for a stream (here: by its ``name`` property) before opening
    an inlet on it.

cache / snapshot
    The stored last message of every topic. It is replayed to a newly connected browser
    (the snapshot), so the dashboard starts with a full picture instead of empty charts.

service client
    The caller side of a ROS 2 service — a request/response call, used here for
    ``update_criteria``. ``call_async`` returns a *future*: a result that is not ready
    yet and must be polled or awaited.

blocking
    A call that holds its thread until it finishes (for example ``serve_forever`` or
    ``rclpy.spin``). Blocking calls are why the servers get their own threads.

handshake (``threading.Event``)
    One thread waits (``wait()``) until another signals (``set()``) that it is ready —
    used so the constructor only continues after the WebSocket server is listening, and
    reused as a stop flag for the LSL workers.
