Web UI
======

.. Node description (the design idea, detail, and other crucial info)

**Verification System Dashboard**

The Web UI is the browser dashboard used to watch a run. It shows the live motion plots,
the per-experiment profile, the evaluation verdict, and the pass/fail criteria editor. It is
three static files served by the ``web_visualizer`` node, and it talks to that node over one
WebSocket connection.

.. code-block:: text

                 ┌────────────────────────────────┐                     ┌──────────────────┐
                 │  Browser                       │   HTTP GET          │  web_visualizer  │
                 │  index.html + app.js + style   │◀───(files +─────────│  node            │
                 │                                │     /config.json)   │                  │
                 │                                │                     │  HTTP server     │
                 │                                │   WebSocket         │  WebSocket server│
                 │                                │◀───topics down──────│                  │
                 │                                │────commands up─────▶│                  │
                 └────────────────────────────────┘                     └──────────────────┘

There are two channels. HTTP serves the three files and a small ``/config.json`` once, at
load. The WebSocket then stays open for the whole session: the node pushes topics down to the
browser, and the browser sends commands back up.

The three files
---------------

- ``index.html`` — the page skeleton: the header controls, the left and right panel, the tabs, 
    and the footer. It also loads the **uPlot** plotting library from a CDN, plus ``style.css`` 
    and ``app.js``. Without it there are no elements for the code to fill.
- ``app.js`` — all of the behaviour. It opens the WebSocket, receives the topics and draws
    them, and sends the button commands back. It is the only file with logic.
- ``style.css`` — appearance and layout only: panels, tabs, plot sizing, colours. It has no
    logic, and nothing in it talks to the back-end.

Interfaces
----------

The browser has no ROS topics of its own. Its interface is the WebSocket contract with the
``web_visualizer`` node — the messages it **receives** and the commands it **sends**.

**Receives** (Web Visualizer Node → browser, routed by ``msg.topic``):

The browser (Web UI) receives the data for the visualization purpose as follows:

- ``estimated_states`` — updates the live position, velocity and acceleration plots.
- ``actual_states`` — the actual-robot line drawn against the estimated one.
- ``event_trigger`` — starts or stops a profile when a test begins or ends.
- ``eval_live`` — fills the live metrics rows while a run is in progress.
- ``eval_summary`` — fills the final pass/fail summary when the run ends.
- ``criteria_snapshot`` — the current pass/fail limits, shown in the ``CRITERIA`` tab.
- ``criteria_ack`` — the reply after an edit, confirming the new limits.
- ``time_sync`` — the node's time reference, used to align the plot time axis.

**Sends** (browser → Web Visualizer Node, in a ``command`` field):

The browser (Web UI) interact with user action. So, after user interact with the interface components on
the UI. The brower will tell the Web Visualizer Node that which button user press/action user did.
Then, the Web Visualizer Node will be decided what to do next.

- ``stop_experiment`` — the Stop button; ends the current run.
- ``skip_iteration`` — skips the current waypoint or trial without ending the run.
- ``time_sync`` — asks the node for its current time reference.
- ``pos_sync`` — nudges the actual line to match the estimated one.
- ``zero_set`` — sets the current position as the new zero.
- ``criteria_update`` — a changed pass/fail limit from the ``CRITERIA`` tab.

The files and ``/config.json`` are served over HTTP; the WebSocket carries the messages
above. The payload shape of each topic is on the **custom-interface pages**, and the workers in the 
Web Visualizer Node that work which the browser are the HTTP-server and WebSocket-server.

UI Workflow
-----------

.. The flow chart of the connection lifecycle

The dashboard connects once at load, then the node streams data down while the browser sends
commands up, until the socket closes and the page reconnects. The sequence below reads top to
bottom as a conversation over time between the browser and the node's two servers.

.. code-block:: text

   Browser (app.js)                 HTTP server            WebSocket server
           │                            │                         │
      ── 1. connect ──                  │                         │
           │  GET files + /config.json  │                         │
           │ ──────────────────────────▶│                         │
           │  files + { ws_port }       │                         │
           │◀────────────────────────── │                         │
           │  new WebSocket(ws://host:ws_port)                    │
           │ ════════════════════════════════════════════════════▶│
           │                            │                         │
      ── 2. snapshot + live ──          │                         │
           │  last-known message per topic                        │
           ◀══════════════════════════════════════════════════════│
           │  estimated_states / eval_live / … (live)             │
           ◀══════════════════════════════════════════════════════│
           │                            │                         │
      ── 3. command ──                  │                         │                 
           │  { command } on button press                         │
           │ ════════════════════════════════════════════════════▶│
           │                            │                         │
      ── 4. reconnect ──                │                         │
           │  socket closes → wait 1 s → reconnect                │
           │ ════════════════════════════════════════════════════▶│
           │                            │                         │

The port is fetched first because the node decides it at launch, so the browser cannot know it
ahead of time. After that the WebSocket is opened once and left open. On connect the WebSocket
server first replays the last-known message on each topic, so a freshly opened tab shows the
current state at once, then the live stream follows. Every message is handed to the panel that
needs it, every button builds one command and sends it, and if the socket ever drops the page
reconnects on its own. The server side of this is explained on the Web Visualizer node page.

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

The parts of the dashboard and what each is for. Every function named below lives in
``app.js``; a ``→`` shows the order the functions run.

- **Live panel** — rolling plots of position, velocity and acceleration, estimated against
  actual. It runs the whole time the page is open.
  *In code:* ``pushLive`` (appends each estimated states sample to the rolling buffers) and
  ``onActualStates`` (stores the latest actual states sample) → ``redraw`` (the 30 Hz draw loop
  that repaints every plot).
- **PROFILE tab** — opens when a test starts. It shows the run's own plots (estimated against
  target), the live metrics while it runs, and the pass/fail summary when it ends.
  *In code:* ``onEventTrigger`` (reacts to an event trigger) → ``startExperiment`` (labels the
  run, clears the eval panel) → ``startProfile`` (opens the tab, starts collecting the run's
  data); ``onEvalLive`` (receives the live metrics) → ``renderMetrics`` (draws the live metrics
  rows); ``onEvalSummary`` (receives the summary) → ``renderSummary`` (draws the pass/fail
  summary).
- **ZOOM tab** — press Crop, then drag a time range on the live plots to inspect it closely.
  *In code:* ``enterCropMode`` (arms crop mode on the live plots) → ``updateSelectionRects``
  (draws the drag selection box) → ``applyZoom`` (copies the selected time range into the zoom
  plots).
- **CRITERIA tab** — shows the current pass/fail limits and lets you edit them live. An edit
  is sent as a ``criteria_update`` command.
  *In code:* ``onCriteriaSnapshot`` (builds the criteria input rows) → an edit sends
  ``criteria_update`` (the changed limit) → ``onCriteriaAck`` (handles the reply) →
  ``flashCriteriaInput`` (flashes the field ok or failed).
- **Header** — three buttons, each a click handler that sends one command:

  - **Time Sync** — ``btn-time-sync`` (sends the ``time_sync`` command; the reply sets
    ``state.timeRef``, the plot time reference).
  - **Pos Sync** — ``btn-pos-sync`` (sends the ``pos_sync`` command to line the actual plot up
    with the estimated one).
  - **Zero** — ``btn-zero`` (sends the ``zero_set`` command to set the current position as zero).

- **Toolbar and footer** — the run and plot controls:

  - **Auto** — ``btn-auto`` (toggles ``state.trackEvents`` — whether an event trigger opens a
    profile).
  - **Stop** — ``btn-stop`` (sends the ``stop_experiment`` command, else ``stopProfile`` when
    offline).
  - **Save CSV** — ``btn-save-csv`` → ``buildCSV`` (builds the CSV text) → ``saveCSV`` (writes
    the file); ``updateSaveCsvBtn`` enables the button when there is data.
  - **Skip Iteration** — ``btn-skip-iteration`` (sends the ``skip_iteration`` command);
    ``updateSkipBtn`` shows it only for the multi-iteration tests.
  - **Clear** — ``btn-clear`` (empties the data buffers) → ``redraw``.
  - **Pause** — ``btn-pause`` (toggles ``state.paused``, which freezes the draw loop).
  - **Crop** — ``btn-crop`` → ``enterCropMode`` / ``exitCropMode`` (arm / disarm crop mode).
  - **Unit toggle** — ``btn-unit-live`` / ``btn-unit-profile`` → ``updatePosLabels`` (relabels
    the position axis in radians or degrees).

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
