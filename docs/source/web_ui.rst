Web UI
======

.. Node description (the design idea, detail, and other crucial info)

**Verification System Dashboard**

The Web UI is the browser dashboard used to watch a run. It shows the live motion plots,
the per-experiment profile, the evaluation verdict, and the pass/fail criteria editor. It is
three static files served by the ``web_visualizer`` node, and it talks to that node over one
WebSocket connection.

.. code-block:: text

                 ┌───────────────────────────────┐                     ┌──────────────────┐
                 │  Browser                       │   HTTP GET          │  web_visualizer  │
                 │  index.html + app.js + style   │◀───(files +─────────│  node            │
                 │                                │     /config.json)   │                  │
                 │                                │                     │  HTTP server     │
                 │                                │   WebSocket         │  WebSocket server│
                 │                                │◀───topics down──────│                  │
                 │                                │────commands up─────▶│                  │
                 └───────────────────────────────┘                     └──────────────────┘

There are two channels. HTTP serves the three files and a small ``/config.json`` once, at
load. The WebSocket then stays open for the whole session: the node pushes topics down to the
browser, and the browser sends commands back up.

The three files
---------------

- ``index.html`` — the page skeleton: the header controls, the left live panel, the right
  ``PROFILE`` / ``ZOOM`` / ``CRITERIA`` tabs, and the footer. It also loads the **uPlot**
  plotting library from a CDN, plus ``style.css`` and ``app.js``. Without it there are no
  elements for the code to fill.
- ``app.js`` — all of the behaviour. It opens the WebSocket, receives the topics and draws
  them, and sends the button commands back. It is the only file with logic.
- ``style.css`` — appearance and layout only: panels, tabs, plot sizing, colours. It has no
  logic, and nothing in it talks to the back-end.

Interfaces
----------

The browser has no ROS topics of its own. Its interface is the WebSocket contract with the
``web_visualizer`` node — the messages it **receives** and the commands it **sends**.

**Receives** (node → browser, routed by ``msg.topic``):

- ``estimated_states`` — updates the live position, velocity and acceleration plots.
- ``actual_states`` — the actual-robot line drawn against the estimated one.
- ``event_trigger`` — starts or stops a profile when a test begins or ends.
- ``eval_live`` — fills the live metrics rows while a run is in progress.
- ``eval_summary`` — fills the final pass/fail summary when the run ends.
- ``criteria_snapshot`` — the current pass/fail limits, shown in the ``CRITERIA`` tab.
- ``criteria_ack`` — the reply after an edit, confirming the new limits.
- ``time_sync`` — the node's time reference, used to align the plot time axis.

**Sends** (browser → node, in a ``command`` field):

- ``stop_experiment`` — the Stop button; ends the current run.
- ``skip_iteration`` — skips the current waypoint or trial without ending the run.
- ``time_sync`` — asks the node for its current time reference.
- ``pos_sync`` — nudges the actual line to match the estimated one.
- ``zero_set`` — sets the current position as the new zero.
- ``criteria_update`` — a changed pass/fail limit from the ``CRITERIA`` tab.

The files and ``/config.json`` are served over HTTP; the WebSocket carries the messages
above. The payload shape of each topic is on the custom-interface pages, and the node side of
this contract is the HTTP-server and WebSocket-server workers on the Web Visualizer page.

UI Workflow
-----------

.. The flow chart of the connection lifecycle

The page connects once at load, then runs two ongoing branches — messages coming in, and
commands going out — until the connection closes, at which point it retries.

.. code-block:: text

   load page
       │
       ▼
   resolveWsUrl()  ──▶ fetch /config.json ──▶ WS_URL = ws://host:<ws_port>
       │
       ▼
   connect()  ──▶ new WebSocket(WS_URL)
       │
       ├──────────────▶  message in  ──▶ switch(msg.topic) ──▶ update the matching panel
       │
       ├──────────────▶  button press ──▶ ws.send({command, data}) ──▶ node
       │
       └──────────────▶  on close ──▶ wait 1 s ──▶ connect()   (retry)

The port is fetched first because the node decides it at launch, so the browser cannot know it
ahead of time. After that the WebSocket is opened once and left open. Every message that
arrives is handed to the panel that needs it, every button builds one command and sends it,
and if the socket ever drops the page reconnects on its own.

How the front-end connects to the back-end
-------------------------------------------

This section walks through the connection code in ``app.js``. It is the only part of the file
examined here; the rest is plotting and UI wiring.

**Port discovery.**

.. code-block:: javascript

   // Fallback default; the real ws_port is fetched from /config.json before connecting.
   let WS_URL          = `ws://${location.hostname || "localhost"}:9090`;

.. code-block:: javascript

   // Fetch the runtime WebSocket port from the server, then connect. Cached in
   // WS_URL so the close-handler's reconnect reuses it without re-fetching.
   async function resolveWsUrl() {
       try {
           const resp = await fetch("/config.json", { cache: "no-store" });
           const cfg  = await resp.json();
           if (cfg && cfg.ws_port) {
               WS_URL = `ws://${location.hostname || "localhost"}:${cfg.ws_port}`;
           }
       } catch (_) { /* keep fallback WS_URL */ }
       document.getElementById("ws-url").textContent = WS_URL;
   }

   // connect();
   resolveWsUrl().then(connect);

``WS_URL`` starts as a fallback in case the fetch fails. ``resolveWsUrl`` asks the node for
``/config.json``, which carries the real ``ws_port``, and rewrites ``WS_URL`` with it. The
last line is the boot order: fetch the port first, *then* ``connect``.

**Opening the socket.**

.. code-block:: javascript

   function connect() {
       ws = new WebSocket(WS_URL);

       ws.addEventListener("open", () => {
           statusDot.className    = "dot connected";
           statusText.textContent = "Connected";
       });

       ws.addEventListener("close", () => {
           statusDot.className    = "dot disconnected";
           statusText.textContent = "Disconnected — retrying";
           ws = null;
           setTimeout(connect, 1000);
       });

       ws.addEventListener("error", () => { /* close handler retries */ });

``connect`` opens the socket and sets the status dot on open. On close it marks the UI
disconnected and calls itself again after one second, so the page recovers by itself if the
node restarts mid-session. An error just falls through to the close handler.

**Receiving — the topic router.**

.. code-block:: javascript

       ws.addEventListener("message", ev => {
           let msg;
           try { msg = JSON.parse(ev.data); } catch (_) { return; }

           switch (msg.topic) {
               case "estimated_states": pushLive(msg);        break;
               case "actual_states":   onActualStates(msg);  break;
               case "event_trigger":   onEventTrigger(msg);  break;
               case "eval_live":          onEvalLive(msg);          break;
               case "eval_summary":       onEvalSummary(msg);       break;
               case "criteria_snapshot":  onCriteriaSnapshot(msg);  break;
               case "criteria_ack":       onCriteriaAck(msg);       break;
               case "time_sync":
                   state.timeRef = msg.data.ref_stamp;
                   break;
           }
       });
   }

Every message is one JSON object with a ``topic`` field. The ``switch`` reads that field and
hands the message to the handler for it — the full list of topics is in Interfaces above. A
message that is not valid JSON is dropped.

**Sending — a command.**

.. code-block:: javascript

   document.getElementById("btn-stop").addEventListener("click", () => {
       if (ws && ws.readyState === WebSocket.OPEN) {
           ws.send(JSON.stringify({ command: "stop_experiment" }));
       }
   });

Every button follows this shape: check the socket is open, then send one JSON object with a
``command`` field. The ``readyState === WebSocket.OPEN`` guard avoids sending into a socket
that is still connecting or already closed. The node receives the command and turns it into
the matching ROS action; the other five commands are listed in Interfaces above.

Important UI functions
----------------------

The parts of the dashboard and what each is for.

- **Live panel** — rolling plots of position, velocity and acceleration, estimated against
  actual. It runs the whole time the page is open.
- **PROFILE tab** — opens when a test starts. It shows the run's own plots (estimated against
  target), the live metrics while it runs, and the pass/fail summary when it ends.
- **ZOOM tab** — press Crop, then drag a time range on the live plots to inspect it closely.
- **CRITERIA tab** — shows the current pass/fail limits and lets you edit them live. An edit
  is sent as a ``criteria_update`` command.
- **Header** — Time Sync aligns the time axis with the node, Pos Sync lines the actual plot up
  with the estimated one, and Zero sets the current position as zero.
- **Toolbar and footer** — Auto follows the newest data, Stop ends the run, Save CSV exports
  the current data, Skip Iteration skips one step, Clear empties the plots, Pause freezes them,
  Crop starts a zoom selection, and the unit toggle switches between radians and degrees.

Notation
--------

Words used on this page.

WebSocket
    A connection that stays open both ways, so the node can push messages to the browser at
    any time instead of the browser having to ask.

uPlot
    The small, fast plotting library used to draw every chart. Loaded from a CDN in
    ``index.html``.

``/config.json``
    A small file the node serves over HTTP that carries the runtime WebSocket port. The
    browser fetches it before connecting because the port is chosen at launch.

``readyState``
    The state of a WebSocket. The code checks it equals ``OPEN`` before sending, so it never
    sends into a socket that is still connecting or already closed.

topic router
    The ``switch (msg.topic)`` block that reads each incoming message's ``topic`` and hands it
    to the right handler.

command
    One JSON object the browser sends up to the node, carrying a ``command`` field. Each button
    sends one.
