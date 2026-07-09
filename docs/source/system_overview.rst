System Architecture & Overview
==============================

.. note::

   This page explains the **overview** of the system. For the implementation details, see
   the :doc:`Base System` and :doc:`verification_system` pages.

The software has **two systems that work together**:

1. **Base System** — the part the *students* use. It gives their robot a ready-made
   user interface (UI) and a command layer, so they can drive the robot without
   building their own interface.
2. **Verification System** — the part the *lecturer* uses on demo day. It measures the
   robot with an encoder and evaluates its performance automatically.

**Tech stack:**

- Python (asyncio) for the Base System back-end.
- React web UI in Docker for the front-end.
- ROS 2 Jazzy + micro-ROS for the Verification System.
- The links between parts use Modbus RTU, WebSocket, and LSL.

**GitHub Repository map**:

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Folder
     - What it is
   * - ``FRA263-264_BaseSystem_BackEnd/``
     - Base System back-end (Python): Modbus + WebSocket + LSL.
   * - ``FRA263-264_BaseSystem_FrontEnd/``
     - Base System front-end: the web UI, shipped as a Docker image and an ``.exe``.
   * - ``claude-visualizer-ws/``
     - Verification System: a ROS 2 Jazzy workspace (four nodes + Teensy firmware + Mock UI for debugging).
   * - ``docs/``
     - This documentation (Sphinx / ReadTheDocs).

System Architecture
-------------------

The block diagram below shows how the pieces connect. Each arrow is one link. The text on
an arrow is the protocol, and the topic, stream, or message it carries — so you can read
each block's inputs and outputs straight from the diagram.

.. code-block:: text

   ===================  BASE SYSTEM  ===================

   Robot (STM32)
     |  ^
     |  |  Modbus RTU over USB serial  (registers 0x00-0x31, 230400 8-E-1, slave 21)
     v  |
   Base System Back-End  (Python, asyncio)
     |
     |--- WebSocket + JSON (ws://localhost:8765) --> Front-End UI (browser, port 3000)
     |        down: STATS status      up: {mode, action} commands
     |
     '--- LSL over the LAN --> Verification System
              out: ActualStates, EventTrigger      in: EstimatedStates


   ===============  VERIFICATION SYSTEM  ===============
   ===  ROS 2 Jazzy -- all topics live under namespace /G<N>/  ===

   Encoder (Teensy 4.1)
     |  micro-ROS over serial (baudrate 115200)
     v
   micro_ros_agent
     |  out: /encoder_raw  (EncoderRaw)
     v
   encoder_reader  (Kalman filter)
     |  Kalman gives velocity + acceleration; position = raw angle - zero
     |  out: /estimated_states  (EncoderState)
     |
     |--> experiment_evaluator
     |        in : /event_trigger (start/stop), /estimated_states
     |        out: /eval_live, /eval_summary  (ExperimentEval)
     |
     '--> web_visualizer  (LSL <-> ROS bridge + web server)
              in : /estimated_states, /actual_states, /event_trigger,
                   /eval_live, /eval_summary, LSL ActualStates + EventTrigger
              out: LSL EstimatedStates, /zero_estimated_states,
                   WebSocket JSON --> Browser dashboard (HTTP :8000, WS :9090)

   Browser dashboard: uPlot charts, live evaluation, Zero / Skip / Pos-Sync / CSV

Brief description: the robot talks to the Base System over a USB cable using Modbus RTU.
The back-end of the Base System reads the robot's data (``ActualStates``) and sends it to
the web UI over a WebSocket, and at the same time streams the ``ActualStates`` to the
Verification System over the network using LSL (Lab Streaming Layer library).

The Verification System has an encoder to measure the real position of the robot, a Kalman
Filter to estimate the states, and the node (web visualizer) to gather all data and
visualize it in another web UI. Lastly, the evaluator node runs in parallel to evaluate
the robot's performance and report it back to the user through the web UI also.

..  old version
    In words: the **robot** talks to the **Base System back-end** over a USB cable using
    Modbus RTU. The back-end shows the roendbot's state in the **web UI** over a WebSocket, and
    at the same time streams the same data to the **Verification System** over the network
    using LSL. Inside the Verification System, an **encoder** measures the real motion; a
    **Kalman filter** cleans it up; the **web visualizer** shows it in a browser; and the
    **evaluator** scores the run.

Data Flow & Protocols
~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 22 20 26

   * - Sender
     - Receiver
     - Protocol
     - Address / Port
     - Payload
   * - Robot
     - Base System Back-end
     - Modbus RTU (USB serial) (Bidirectional)
     - 
       * ``/dev/ttyACM*``
       * baudrate 230400
       * 8-E-1
       * slave address 21
     - 16-bit registers ``0x00``–``0x31``
   * - Base System webUI
     - Base System Back-end
     - WebSocket (JSON) (Bidirectional)
     - ``ws://localhost:8765``
     - 
        * ``STATS`` messages to UI
        * ``{mode, action}`` commands from user to Back-end
   * - Base System Back-end
     - Verification System Back-end 
     - 
        * LSL stream ``ActualStates``
        * LSL stream ``EventTrigger``

     - LAN
     - 
        * Robot's actual states: position, speed, accel
        * one JSON message per user action
   * - Verification System Back-end
     - Base System Back-end
     - LSL stream ``EstimatedStates``
     - LAN
     - Estimated states: position, velocity, accel
   * - One node in the Verification System
     - Another node in the Verification System
     - ROS 2 topics/services (DDS)
     - same network and ROS_DOMAIN_ID
     - ``EncoderRaw``, ``EncoderState``, ``EventTrigger``, ``ExperimentEval``
   * - Teensy
     - Verification System Back-end 
     - micro-ROS (USB serial)
     - 
        * ``/dev/ttyACM*``
        * baudrate 115200
     - ``/encoder_raw`` messages
   * - Verification System webUI
     - Verification System Back-end (web_visualizer)
     - HTTP + WebSocket (JSON)
     - 
        * ``http://localhost:8000``
        * ``ws://localhost:9090``
     - 
        * Robot's actual states
        * Estimated states
        * Events triggering signal
        * Live evaluation data 

..  AI suggestion (I don't think it is needed)
    **Base ↔ front-end message shapes.** The front-end and back-end talk over the local
    WebSocket (``ws://localhost:8765``) using JSON. The UI sends **command envelopes** like
    ``{"mode": "Manual", "action": "jog", "value": 10, "direction": "CCW"}``; the back-end
    sends **status messages** like
    ``{"type": "STATS", "pos": 12.3, "speed": 4.5, "mode": "...", "heartbeat_alive": true}``.
    The full Modbus register map — every address the back-end writes and reads — is in the
    :doc:`Base System` page.

    One ``group_number = N`` keeps robot setups apart.
    It puts the Verification System's ROS topics under the namespace ``/G<N>/``, adds a ``_N``
    suffix to the LSL stream names, and selects the group's row in ``criteria.yaml``. The web
    ports stay fixed (HTTP 8000, WS 9090). ``group_number 0`` is the default single-group case:
    namespace ``/G0/``, no LSL suffix, and the ``default`` criteria row.

**Methods to handle multiple machines in the same network** 

* Each Back-end and front-end is on the same local machine, so there is no problem
* ROS message of the Verification System will be published under the namespace ``/G<N>/``, 
specified by a launch argument ``--group-number``. The default is 0.
* LSL streams name is modified by adding a suffix ``_N``, where N is the same launch argument.
* The criteria which is used in the evaluation node will be depended on the ``--group-number`` 
argument also. You can check it out in the ``/claude-visualizer-ws/src/config/criteria.yaml``.

.. note::

    ``group_number 0`` is the default single-group case:
    * namespace ``/G0/``
    * no LSL suffix
    * ``default`` criteria row.


Base System
-----------

The Base System is the part students use to drive their robot. It gives them a finished
UI and a command layer, so they can focus on their robot instead of building an
interface. It has two parts — a **back-end** and a **front-end** — that talk over a local
WebSocket.

Back-end
~~~~~~~~

**Objective.** The back-end is the bridge between the web UI and the robot controller
(an STM32 board). It turns simple UI actions into Modbus commands, reads the robot's
status back, and shares the data with the Verification System.

**Functions and features:**

1. **Built with Python asyncio.** It runs three independent loops at fixed rates, so one
   slow task does not block the others: a **5 Hz** heartbeat, a **25 Hz** status poll, and
   a **25 Hz** broadcast to the web UI and the LSL stream.
2. **Heartbeat.** It reads register ``0x00`` on the robot controller (the STM32 MCU) at
   5 Hz. When the robot writes **YA (22881)**, the back-end writes **HI (18537)** back. If
   YA stops arriving in time, the UI shows the link as "not alive", even when the cable is
   still plugged in.
3. **Status poll.** It reads the register block ``0x00``–``0x31`` on the robot controller
   (the STM32 MCU) at 25 Hz to get position, speed, gripper state, mode, and the emergency
   flag.
4. **Command handling.** It receives messages from the UI in JSON format to command the
   robot — Home, Manual / Jog, Auto (pick-and-place or point-to-point), Test (performance
   or precision), and Stop — and writes the matching Modbus registers.
5. **Verification link.** It publishes two LSL streams: ``ActualStates`` (position, speed,
   accel) and ``EventTrigger`` (a JSON message for every user action), so the Verification
   System can detect the robot's action and evaluate its performance.
6. **Multi-group support.** Setting the group number ``N`` adds a ``_N`` suffix to the LSL
   stream names, so many groups can run on one LAN without mixing their data.

Front-end
~~~~~~~~~

.. note::

   *Author to complete.* The front-end is shipped as a compiled Docker image and an
   ``.exe`` (no source in this repository), so this subsection is left as a skeleton.

``FRA263-264_BaseSystem_FrontEnd/HowToUse.pdf``

.. Objective of the front-end (author to write).

.. Functions and features of the front-end (author to write).

.. Link the front-end manual here:
   - ``FRA263-264_BaseSystem_FrontEnd/README.md`` (how it works + register map)
   - ``FRA263-264_BaseSystem_FrontEnd/HowToUse.pdf``
   - "How to use Basesystem 101" (Canva): https://canva.link/9pr3jhzbh18pxbn

Verification System
-------------------

This part is used to evaluate the robot on demo day. Before, a person had to watch the
logged data by eye in STM32CubeIDE. The Verification System does this automatically: it
reads the real position with an **encoder**, estimates velocity and acceleration with a
**Kalman filter**, shows the data on a web UI, and checks the robot's performance against
set limits.

Back-end
~~~~~~~~

The back-end is split into **four ROS 2 nodes**.

micro_ros_agent (Teensy bridge)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Role.** It connects the Teensy 4.1 board to ROS 2. The Teensy firmware reads an
8192-count quadrature encoder and publishes the raw ticks. The agent carries these
messages from the board (over USB serial, baudrate 115200) into the ROS 2 network, on the
``/encoder_raw`` topic (``EncoderRaw``: ticks, raw position, ``dt_us``) at about 100 Hz.

**Features.** For hardware-free development, ``mock_ui.py`` (a Tkinter GUI) replaces both
the Teensy **and** the robot: it publishes ``/encoder_raw`` and ``/event_trigger`` and both
LSL streams, so the whole pipeline can run on any machine. (The older ``mock_encoder.py``
only replaces the Teensy and is now deprecated.)

Kalman Filter node
^^^^^^^^^^^^^^^^^^

**Role** (node name ``encoder_reader``). It turns the noisy raw ticks into smooth velocity
and acceleration, and a clean position.

**Model.** It uses a **constant-jerk Kalman filter**. The state is
``[position, velocity, acceleration, jerk]``. Each new tick reading is used to predict and
then correct the state. The **velocity and acceleration** come from the filter; the
published **position** is the raw encoder angle minus a zero offset (not the filter's
position state). The process- and measurement-noise values (``Q`` and ``R``) are tuned in
``params.yaml``. The node can re-zero its position on request (topic
``/zero_estimated_states``), so every subscriber shares the same zero point. It reads
``/encoder_raw`` and publishes ``/estimated_states`` (``EncoderState``: position, velocity,
acceleration, plus their variances and the raw tick count).

Web Visualizer node
^^^^^^^^^^^^^^^^^^^

**Role** (node name ``web_visualizer``). It is the bridge and the web server. It connects
three worlds — LSL, ROS 2, and the browser.

**Features:**

- Reads the data from the Base System's LSL streams (``ActualStates``, ``EventTrigger``)
  and brings it into ROS 2 topics.
- Sends the Kalman result (``/estimated_states``) back out as an LSL stream
  (``EstimatedStates``), so the robot side can read the filtered data.
- Serves the verifier web page over **HTTP (localhost:8000)** and streams live JSON to the
  browser over **WebSocket (localhost:9090)**.
- Handles browser commands: zero, stop experiment, skip iteration, update criteria, and
  time / position sync.

System Evaluator node
^^^^^^^^^^^^^^^^^^^^^

**Role** (node name ``experiment_evaluator``). It watches a run and evaluates it
automatically.

**Features.** It waits for a start event — one of ``point_to_point``, ``pick_place``,
``performance``, or ``precision`` — then evaluates the robot from the estimated states.
This node also provides the data of the experiments — including initial and target
position, current error, and so on — to the UI to help the visualization. The pass/fail
limits can be configured in two ways: (1) directly in the UI, and (2) in ``criteria.yaml``.

..  old version
    measures the robot from the estimated states. It
    reports live numbers during the run and a final **pass / fail** summary at the end. The
    pass/fail limits come from ``criteria.yaml`` (chosen by ``pair_id``) and can be changed
    during a run through the ``/update_criteria`` service. Positions are in radians; one hole
    index = 360 / 72 = 5°.

Typical metrics: settling time, overshoot (%), final error, peak speed and acceleration,
and precision (the mean and spread of repeated stops).

Front-end
~~~~~~~~~

**Role.** The front-end is the browser dashboard, served by the web visualizer node. It
shows the live data and the evaluation results, so the lecturer can watch the robot's
performance in real time.

**Features:**

- Live **uPlot** charts of the actual and estimated position, velocity, and acceleration
  (updated about 30 times per second).
- An **evaluation panel** that shows the live metrics and the final pass/fail result from
  the evaluator node.
- A **Criteria tab** to edit the pass/fail limits live (sent to the evaluator node).
- Controls for **Zero** (re-home the position), **Skip Iteration** (drop a stuck
  waypoint / trial), **Pos Sync**, and **CSV export** of the recorded data.
- It finds the right WebSocket port itself (by reading ``/config.json``), so it always
  connects back to whichever machine served the page.

Notation
--------

Words and symbols used in this documentation and in the code.

YA / HI
    Heartbeat "magic numbers". The robot writes **YA = 22881** into register ``0x00`` to
    ask "are you there?"; the back-end writes **HI = 18537** to answer. Each is a pair of
    ASCII letters read as one 16-bit number.

Modbus RTU register
    A 16-bit value stored at a numbered address (written in hex, ``0x00``–``0x31``). Signed
    values use **two's complement**. Some values are stored **×10** (for example, a raw
    ``1234`` means ``123.4``).

LSL (Lab Streaming Layer)
    A library for sending data streams over a network. A source publishes an **outlet**; a
    reader gets the data with an **inlet**. Streams are found by their **name** and
    **source_id**. **IRREGULAR_RATE** means samples arrive only when something happens
    (used for ``EventTrigger``).

ROS 2 / rclpy / topic / QoS
    **ROS 2 (Jazzy)** is the robotics middleware the Verification System is built on.
    **rclpy** is its Python API. Nodes exchange messages over named **topics**. **QoS**
    (Quality of Service) sets the delivery rules — here, RELIABLE and "keep the last 10".

micro-ROS
    A small version of ROS 2 that runs on a microcontroller (the Teensy board).
    ``micro_ros_agent`` connects it to the main ROS 2 network.

Constant-jerk Kalman filter
    A filter whose motion model assumes the **jerk** (the rate of change of acceleration)
    is roughly constant between samples. This gives smooth position, velocity, and
    acceleration estimates from noisy encoder ticks.

group_number / namespace
    Settings that keep robot setups apart on one network. A single ``group_number = N``
    puts the Verification System's ROS topics under the namespace ``/G<N>/``, adds a ``_N``
    suffix to the LSL stream names (e.g. ``ActualStates_5``), and selects the group's row
    in ``criteria.yaml``. The web ports stay fixed (HTTP 8000, WS 9090). ``group_number 0``
    is the default: namespace ``/G0/``, no LSL suffix, and the ``default`` criteria.

Units
    Position in **radians (rad)**, velocity in **rad/s**, acceleration in **rad/s²**.
    An **index** is a hole number on the plate: 1 index = 360 / 72 = **5°**. A **degree**
    is the plain angle. Sign convention: **+ = counter-clockwise (CCW)**, **− = clockwise
    (CW)**.

Settling band / settling time / overshoot
    The **settling band** is a small window around the target (a % of the travel distance).
    The **settling time** is how long the robot takes to enter and stay in that band. The
    **overshoot** is how far the robot passes the target before coming back, as a % of the
    travel distance.
