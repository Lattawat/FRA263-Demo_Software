System Architecture & Overview
==============================

This page is a **map of the whole project for the next developer**. It explains what
each part does, how the parts talk to each other, and the words you need to know. It
stays high-level on purpose. The deep detail (for example, the full Modbus register
map) lives in the :doc:`Base System` and :doc:`Verification System` pages.

The software has **two systems that work together**:

1. **Base System** — the part the *students* use. It gives their robot a ready-made
   user interface (UI) and a command layer, so they can drive the robot without
   building their own interface.
2. **Verification System** — the part the *lecturer* uses on demo day. It measures the
   robot with an encoder and scores its performance automatically.

**Tech stack at a glance:** Python (asyncio) for the Base System back-end · React web UI
in Docker for the front-end · ROS 2 Jazzy + micro-ROS for the Verification System ·
links between parts use Modbus RTU, WebSocket, and LSL.

**Repository map** — where each part lives in the code:

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
     - Verification System: a ROS 2 Jazzy workspace (four nodes + Teensy firmware).
   * - ``docs/``
     - This documentation (Sphinx / ReadTheDocs).

System Architecture
-------------------

The diagram below shows how the pieces connect. Each arrow is one link, with its
protocol written on it.

.. code-block:: text

   Robot (STM32)
      |
      |  Modbus RTU  (USB serial, 230400 8-E-1, slave address 21)
      v
   Base System Back-End  (Python, asyncio)
      |
      +-- WebSocket + JSON  (ws://localhost:8765) --> Base System Front-End
      |                                               (Web UI in a browser, port 3000)
      |
      +-- LSL streams over the LAN -----------------> Verification System (ROS 2 Jazzy)
              out:  ActualStates, EventTrigger
              in :  EstimatedStates


   Verification System pipeline (ROS 2 Jazzy):

   Encoder      micro_ros_agent      Kalman Filter        web_visualizer ---> Browser UI
   (Teensy) --> (/encoder_raw)  -->  (/estimated_states) -->    |          (HTTP port 8000,
                                                                |           WS   port 9090)
                                                                v
                                                     experiment_evaluator
                                                     (/eval_live, /eval_summary)

In words: the **robot** talks to the **Base System back-end** over a USB cable using
Modbus RTU. The back-end shows the robot's state in the **web UI** over a WebSocket, and
at the same time streams the same data to the **Verification System** over the network
using LSL. Inside the Verification System, an **encoder** measures the real motion; a
**Kalman filter** cleans it up; the **web visualizer** shows it in a browser; and the
**evaluator** scores the run.

Data Flow & Protocols
~~~~~~~~~~~~~~~~~~~~~~~

Every link in the system, with its protocol, address, and payload:

.. list-table::
   :header-rows: 1
   :widths: 26 22 26 26

   * - Link (from ↔ to)
     - Protocol
     - Address / Port
     - Payload
   * - Robot ↔ Base back-end
     - Modbus RTU over USB serial
     - ``/dev/ttyACM*``, 230400 8-E-1, slave 21
     - 16-bit registers ``0x00``–``0x31``
   * - Front-end ↔ Base back-end
     - WebSocket (JSON)
     - ``ws://localhost:8765``
     - ``STATS`` messages down; ``{mode, action}`` commands up
   * - Base back-end → Verification
     - LSL stream ``ActualStates``
     - LAN (found by stream name)
     - 3 numbers: position, speed, accel (25 Hz)
   * - Base back-end → Verification
     - LSL stream ``EventTrigger``
     - LAN
     - one JSON message per user action
   * - Verification → robot side
     - LSL stream ``EstimatedStates``
     - LAN
     - 3 numbers: filtered position, velocity, accel
   * - Inside Verification
     - ROS 2 topics/services (DDS)
     - ``ROS_DOMAIN_ID=156``
     - ``EncoderRaw``, ``EncoderState``, ``EventTrigger``, ``ExperimentEval``
   * - Teensy ↔ micro_ros_agent
     - micro-ROS over serial
     - ``/dev/ttyACM0``, baudrate 115200
     - ``/encoder_raw`` messages
   * - Browser ↔ web_visualizer
     - HTTP + WebSocket (JSON)
     - HTTP port 8000, WS port 9090
     - live JSON: states, events, evaluation

**Running many pairs on one network.** The ``pair_id`` setting keeps two robot setups
apart. For pair *N*, it adds a ``_N`` suffix to the LSL stream names, picks the web ports
(``9000+N`` and ``8000+N``), and sets the ``ROS_DOMAIN_ID`` so the ROS 2 traffic does not
mix. ``pair_id 0`` means the legacy single-pair defaults (no suffix, ports 9090 / 8000).

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
   slow task does not block the others.
2. **Heartbeat.** It reads register ``0x00`` at 5 Hz. When the robot writes **YA (22881)**,
   the back-end writes **HI (18537)** back. If YA stops arriving in time, the UI shows the
   link as "not alive", even when the cable is still plugged in.
3. **Status poll.** It reads the register block ``0x00``–``0x31`` at 25 Hz to get position,
   speed, gripper state, mode, and the emergency flag.
4. **WebSocket command handling.** It receives JSON commands from the UI — Home, Manual /
   Jog, Auto (pick-and-place or point-to-point), Test (performance or precision), and
   Stop — and writes the matching Modbus registers.
5. **Verification link.** It publishes two LSL streams: ``ActualStates`` (position, speed,
   accel) and ``EventTrigger`` (a JSON message for every user action), so the Verification
   System can score the run.
6. **Multi-pair support.** The ``--pair_id N`` option adds a ``_N`` suffix to the LSL
   stream names, so many robot pairs can run on one LAN without mixing their data.
7. **Set-home.** "Set home" zeroes the position shown in the UI only; the LSL data stays
   raw, so the Verification System keeps the true reading.

**Inputs:** JSON commands from the web UI; Modbus register reads from the robot.

**Outputs:** Modbus register writes to the robot; ``STATS`` messages to the web UI; LSL
samples to the Verification System.

Front-end
~~~~~~~~~

.. note::

   *Author to complete.* The front-end is shipped as a compiled Docker image and an
   ``.exe`` (no source in this repository), so this subsection is left as a skeleton.

.. Objective of the front-end (author to write).

.. Functions and features of the front-end (author to write).

.. Link the front-end manual here:
   - ``FRA263-264_BaseSystem_FrontEnd/README.md`` (how it works + register map)
   - ``FRA263-264_BaseSystem_FrontEnd/HowToUse.pdf``
   - "How to use Basesystem 101" (Canva): https://canva.link/9pr3jhzbh18pxbn

Internal communication protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The front-end and back-end talk over a **local WebSocket** using **JSON messages**.

- The UI sends **command envelopes** like
  ``{"mode": "Manual", "action": "jog", "value": 10, "direction": "CCW"}``.
- The back-end sends **status messages** like
  ``{"type": "STATS", "pos": 12.3, "speed": 4.5, "mode": "...", "heartbeat_alive": true}``.

The full Modbus register map — every address the back-end writes and reads — is
documented in the :doc:`Base System` page.

Verification System
-------------------

This part is used to evaluate the robot on demo day. Before, a person had to watch the
logged data by eye in STM32CubeIDE. The Verification System does this automatically: it
reads the real position with an **encoder**, estimates velocity and acceleration with a
**Kalman filter**, shows the data on a web UI, and checks the robot's performance against
set limits.

The system is split into **four ROS 2 nodes**.

micro_ros_agent (Teensy bridge)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Role.** It connects the Teensy 4.1 board to ROS 2. The Teensy firmware reads a
4096-count quadrature encoder and publishes the raw ticks. The agent carries these
messages from the board (over USB serial, baudrate 115200) into the ROS 2 network.

**Features.** In mock mode (no hardware), ``mock_encoder.py`` replaces the Teensy and
publishes synthetic ticks, so the pipeline can run on any machine.

- **Input:** encoder hardware (ticks), read by the Teensy firmware.
- **Output:** the ``/encoder_raw`` topic (``EncoderRaw``: ticks, raw position, ``dt_us``),
  at about 100 Hz.

Kalman Filter node
~~~~~~~~~~~~~~~~~~~

**Role** (node name ``encoder_reader``). It turns the noisy raw ticks into smooth
position, velocity, and acceleration.

**Model.** It uses a **constant-jerk Kalman filter**. The state is
``[position, velocity, acceleration, jerk]``. Each new tick reading is used to predict and
then correct the state. The process- and measurement-noise values (``Q`` and ``R``) are
tuned in ``params.yaml``. The node can also re-zero its position on request (topic
``/zero_estimated_states``), so every downstream consumer shares the same zero point.

- **Input:** the ``/encoder_raw`` topic.
- **Output:** the ``/estimated_states`` topic (``EncoderState``: position, velocity,
  acceleration, plus their variances and the raw tick count).

Web Visualizer node
~~~~~~~~~~~~~~~~~~~~

**Role** (node name ``web_visualizer``). It is the bridge and the web server. It connects
three worlds — LSL, ROS 2, and the browser.

**Features:**

- Brings the Base System's LSL streams (``ActualStates``, ``EventTrigger``) into ROS 2
  topics.
- Sends the Kalman result (``/estimated_states``) back out as an LSL stream
  (``EstimatedStates``), so the robot side can read the filtered data.
- Serves the verifier web page over **HTTP (port 8000)** and streams live JSON to the
  browser over **WebSocket (port 9090)**.
- Handles browser commands: zero, stop experiment, skip iteration, update criteria, and
  time / position sync.

- **Input:** LSL streams and ROS topics (``/estimated_states``, ``/actual_states``,
  ``/event_trigger``, ``/eval_live``, ``/eval_summary``).
- **Output:** JSON to the browser; the ``/actual_states`` and ``/event_trigger`` ROS
  topics; and the ``EstimatedStates`` LSL outlet.

System Evaluator node
~~~~~~~~~~~~~~~~~~~~~~

**Role** (node name ``experiment_evaluator``). It watches a run and scores it
automatically.

**Features.** It waits for a start event — one of ``point_to_point``, ``pick_place``,
``performance``, or ``precision`` — then measures the robot from the estimated states. It
reports live numbers during the run and a final **pass / fail** summary at the end. The
pass/fail limits come from ``criteria.yaml`` (chosen by ``pair_id``) and can be changed
during a run through the ``/update_criteria`` service. Positions are in radians; one hole
index = 360 / 72 = 5°.

Typical metrics: settling time, overshoot (%), final error, peak speed and acceleration,
and precision (the mean and spread of repeated stops).

- **Input:** the ``/event_trigger`` topic (start / stop) and the ``/estimated_states``
  topic.
- **Output:** the ``/eval_live`` topic (live metrics) and the ``/eval_summary`` topic
  (final result with pass/fail).

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

pair_id / session / ROS_DOMAIN_ID
    Settings that keep two robot setups apart on one network. ``pair_id N`` adds a ``_N``
    suffix to the LSL stream names, picks the web ports (``9000+N`` / ``8000+N``), and
    selects the ``ROS_DOMAIN_ID`` so the ROS 2 (DDS) traffic does not mix.

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
