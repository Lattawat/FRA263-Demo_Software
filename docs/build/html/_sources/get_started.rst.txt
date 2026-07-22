Getting Started
===============

.. Installation guide: fresh clone -> running system, Ubuntu only.

**From a fresh clone to a running system.** This page assumes you have just cloned the
repository onto a machine with nothing installed on it. It walks through the packages, the
Python environment, the micro-ROS agent, and the build — then shows the **two ways to run**
the project.

.. note::

   This project runs on **Ubuntu only**. The Docker and Windows options were removed. The
   one place Docker still appears is the Base System front-end UI, which is shipped as a
   container image and has no other distribution form.

Before you start
----------------

**What the machine needs.** These versions go together: ROS 2 Jazzy targets Ubuntu 24.04,
and both use Python 3.12. That shared Python version is what later lets one virtual
environment see the ROS packages.

.. list-table::
   :header-rows: 1
   :widths: 26 20 54

   * - Item
     - Version
     - Note
   * - Ubuntu
     - 24.04 LTS
     - The distribution ROS 2 Jazzy is built for.
   * - ROS 2
     - Jazzy
     - Installed at ``/opt/ros/jazzy``. Older notes saying *humble* are out of date.
   * - Python
     - 3.12
     - Comes with Ubuntu 24.04 and is the same version ROS 2 Jazzy uses.
   * - Docker
     - any recent
     - Only for the Base System front-end UI (Step 8). Not used anywhere else.

.. **What the hardware needs.** The two run methods do not need the same equipment. Method 1
.. runs on any laptop; only Method 2 needs the rig.

.. .. list-table::
..    :header-rows: 1
..    :widths: 26 37 37

..    * - Part
..      - Method 1 — no hardware
..      - Method 2 — full system
..    * - Teensy 4.1 + encoder
..      - Not needed. ``mock_ui`` publishes ``encoder_raw`` instead.
..      - Required. Flashed with the firmware (Step 7).
..    * - Robot (STM32)
..      - Not needed. ``mock_ui`` also sends the LSL streams.
..      - Required, connected over USB serial.
..    * - micro-ROS agent
..      - Not started.
..      - Required (Step 5).
..    * - Base System back-end + UI
..      - Not started.
..      - Required (Steps 4 and 8).

Step 1 — Install ROS 2 Jazzy
----------------------------

Follow the official Debian-package guide, which adds the apt key and the ROS 2 repository:

   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Then install the desktop variant:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-jazzy-desktop

Check that it is there:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   printenv ROS_DISTRO          # must print:  jazzy

.. note::

   Sourcing ``setup.bash`` only affects the terminal you run it in. Every new terminal needs
   it again — see :ref:`environment-prep`.

Step 2 — Install the system packages
------------------------------------

.. code-block:: bash

   # package list already refreshed in Step 1
   sudo apt install -y \
       git \
       cmake \
       python3-venv \
       python3-pip \
       python3-tk \
       python3-rosdep \
       python3-colcon-common-extensions

**Why these.** ``python3-tk`` is the Tkinter library — ``mock_ui`` is a Tkinter window, so
without it Method 1 cannot start. ``python3-colcon-common-extensions`` provides the
``colcon`` build tool. ``python3-rosdep`` resolves the micro-ROS dependencies in Step 5.
``cmake`` is used by the Teensy firmware cross-build in Step 7.

**USB permission.** Both the Teensy and the robot appear as USB serial devices. A normal
user cannot open them until it belongs to the ``dialout`` group:

.. code-block:: bash

   sudo usermod -aG dialout $USER

This only takes effect after you log out and back in.

.. warning::

   Logging out **closes every open application**. Save your work and close your programs
   first. Log out the session (the same method as shutdown, but select log out instead) and
   back in.

Skipping this gives ``Permission denied: '/dev/ttyACM0'`` later, with nothing else to explain
it.

Step 3 — Clone the repository
-----------------------------

Clone into any folder you like, then enter it:

.. code-block:: bash

   git clone https://github.com/Lattawat/FRA263-Demo_Software.git
   cd FRA263-Demo_Software

.. note::

   **Paths on this page are relative to the project root** — the ``FRA263-Demo_Software`` folder
   the clone just created. Open every new terminal at that root, or ``cd`` into it first, so the
   relative commands resolve.

**What you just cloned.**

.. code-block:: text

   FRA263-Demo_Software/
   ├── claude-visualizer-ws/                    Verification System — ROS 2 Jazzy workspace
   │   ├── src/claude_visualizer/               nodes, launch, config, mock UI
   │   ├── src/claude_visualizer_interface/     custom ROS 2 messages
   │   └── encoder_data_publisher/              Teensy firmware (PlatformIO)
   ├── FRA263-264_BaseSystem_BackEnd/           Base System back-end (server_111.py, protocol.py)
   ├── FRA263-264_BaseSystem_FrontEnd/          Base System front-end (Docker image + compose)
   ├── docs/                                    this documentation
   └── requirements.txt                         Python dependencies

See :doc:`system_overview` for what each part does. ``build/``, ``install/``, ``log/`` and
``.venv`` are not in the clone — you create them in the steps below.

Step 4 — Create the Python environment
--------------------------------------

The project keeps its Python packages in one virtual environment at the repository root. Run
this from the project root:

.. code-block:: bash

   python3 -m venv .venv
   source .venv/bin/activate

Make sure that the ``(.venv)`` prompt is showing. Then install the dependencies:

.. code-block:: bash

   pip install -r requirements.txt

``requirements.txt`` holds every pip package the project needs — the application packages
(``pylsl``, ``websockets``, ``pymodbus``, ``pyserial``, ``PyYAML``, ``numpy``) **and** the ROS 2
build tooling (``catkin_pkg``, ``empy``, ``lark``) that ``colcon build`` needs in Step 5 and
Step 6.

.. note::

   **Two sets of Python packages.** The virtual environment provides the pip packages
   (``pylsl``, ``websockets``, ``pymodbus``, ``pyserial``, ``PyYAML``). ROS 2 provides
   ``rclpy`` and the message packages.

   Sourcing ``/opt/ros/jazzy/setup.bash`` adds the ROS packages to the path Python searches,
   so they can be imported inside the virtual environment. This works because both use
   Python 3.12.

   Every terminal needs both. Activate the environment only and ``rclpy`` is missing; source
   ROS 2 only and ``pylsl`` is missing.

.. note::

   ``pylsl`` ships its own ``liblsl.so`` inside the pip package, so there is **no** separate
   LSL system library to install.

Step 5 — Install the micro-ROS agent
------------------------------------

*Needed only for Method 2 (with hardware). Skip it if you only want the no-hardware run.*

The agent is the program that carries messages from the Teensy into the ROS 2 network.

**micro-ROS is not part of this repository.** It is maintained by the micro-ROS project, so you
fetch and build it yourself with their official tool. The commands below add it into
``claude-visualizer-ws/src/`` and build it next to the project packages:

.. note::

   Keep the ``.venv`` from Step 4 active for this build. ``colcon build`` needs ``catkin_pkg``,
   ``empy`` and ``lark``, which Step 4 installed into the environment — the system does not
   provide them inside the environment.

.. code-block:: bash

   cd claude-visualizer-ws
   source /opt/ros/jazzy/setup.bash

   # 1. Get the setup tool. The branch must match your ROS 2 distribution.
   git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

   # 2. Resolve its dependencies.
   sudo rosdep init          # first time on this machine only; harmless error if already done
   rosdep update
   rosdep install --from-paths src --ignore-src -y

   # 3. Build the setup tool.
   colcon build
   source install/local_setup.bash

   # 4. Create and build the agent workspace. This fills src/uros/.
   ros2 run micro_ros_setup create_agent_ws.sh
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.bash

Check that the agent exists:

.. code-block:: bash

   ros2 pkg list | grep micro_ros_agent      # must print:  micro_ros_agent

Step 6 — Build the workspace
----------------------------

This builds the Verification System packages — ``claude_visualizer`` and the custom message
package ``claude_visualizer_interface``:

.. code-block:: bash

   cd claude-visualizer-ws
   source /opt/ros/jazzy/setup.bash
   colcon build
   source install/setup.bash

The first build takes several minutes, mostly generating the custom message types.

.. warning::

   **Do not use** ``--symlink-install``. Always build with plain ``colcon build``.
   The developer cannot remember the reason, but there are some error when ``install/`` 
   point back at the files in ``src/`` instead of copying them.

   If this workspace was ever built with ``--symlink-install``, delete ``build/``,
   ``install/`` and ``log/`` first, then build again.

Step 7 — Build and flash the Teensy firmware
--------------------------------------------

*Needed only for Method 2 (with hardware).*

The firmware is a PlatformIO project in ``claude-visualizer-ws/encoder_data_publisher/``. It
reads the encoder and publishes ``/G<N>/encoder_raw``. The recommended way to build and flash it
is **VS Code with the PlatformIO IDE extension**. For how the build actually works, the
command-line alternative, and how to add another message, see
:doc:`verification_system/teensy_firmware`.

Install the PlatformIO extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In VS Code open **Extensions**, search for *PlatformIO IDE*, and install it. (When you open the
firmware folder in the next step, VS Code also offers to install it, because the folder
recommends it.)

Open the firmware in VS Code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File → Open Folder** and choose ``claude-visualizer-ws/encoder_data_publisher``. Open that
folder itself, not the whole repository — PlatformIO looks for ``platformio.ini`` at the top of
the folder you open. Once it loads, the blue **PlatformIO toolbar** appears along the bottom of
the window.

Check the group number and domain before flashing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open ``src/main.cpp`` and confirm two values — the group number near the top, and the domain ID
inside ``setup()``:

.. code-block:: cpp

   #define GROUP_NUMBER 1                                  // set to your group N
   rcl_init_options_set_domain_id(&init_options, 156);     // must equal ROS_DOMAIN_ID

.. warning::

   **The defaults do not match out of the box.** The firmware ships with ``GROUP_NUMBER 1``,
   while the launch file and the Base System back-end default to ``0``. Set them all to the same
   ``N``, and keep the domain at ``156`` (the value used everywhere in this guide). If they
   disagree, the Teensy publishes on ``/G1/encoder_raw`` while the rest of the system listens on
   ``/G0/encoder_raw`` — everything starts, but no data lines up.

Allow the Teensy to be flashed (one time)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A USB device is owned by ``root`` by default, so the upload fails with a permission error until
your user is allowed to access Teensy boards:

.. code-block:: bash

   sudo curl -fsSL https://www.pjrc.com/teensy/00-teensy.rules -o /etc/udev/rules.d/00-teensy.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger

Build and upload
~~~~~~~~~~~~~~~~

With the Teensy plugged in, use the **PlatformIO toolbar** at the bottom of the VS Code window:

- **✓ (Build)** compiles the firmware. The first build takes 5–15 minutes.
- **→ (Upload)** flashes the board.

The same actions are under the PlatformIO sidebar → *Project Tasks → teensy41 → General → Build
/ Upload*. If the Teensy Loader window opens and waits, press the button on the board to start
the flash.

Step 8 — Load the front-end UI image
------------------------------------

*Needed only for Method 2 (with hardware).*

The Base System front-end is distributed as a Docker image:

.. code-block:: bash

   cd FRA263-264_BaseSystem_FrontEnd
   docker load -i frontend-image_v1_2.tar      # first time only
   docker compose up -d

The UI is then served at http://localhost:3000. To stop and remove the container:

.. code-block:: bash

   docker compose down

.. note::

   This is the only Docker in the project. The Verification System runs directly on the
   machine — do not use the compose files under ``claude-visualizer-ws/docker/``, which
   belong to the removed container option.

Step 9 — Check the installation
-------------------------------

These checks need no hardware. Run them from the project root; each line tells you which step
failed. The real functional test is Method 1 below — if it draws charts, the install is good.

.. code-block:: bash

   # from the project root
   source /opt/ros/jazzy/setup.bash
   source claude-visualizer-ws/install/setup.bash
   source .venv/bin/activate

   printenv ROS_DISTRO                                   # Step 1 -> jazzy
   ros2 pkg list | grep -E "claude|micro_ros_agent"      # Steps 5, 6
   python3 -c "import pylsl, websockets, pymodbus, serial, yaml, numpy; print('python deps OK')"

``ros2 pkg list`` should show ``claude_visualizer`` and ``claude_visualizer_interface``, plus
``micro_ros_agent`` if you did Step 5.

Running the system
------------------

.. _environment-prep:

The environment preparation
~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Every new terminal starts the same way.** A terminal that skips a line here will fail with
a missing command or a missing module, so run all four in each terminal before the commands
in the run methods below:

.. code-block:: bash

   # from the project root
   source /opt/ros/jazzy/setup.bash                       # ROS 2
   source claude-visualizer-ws/install/setup.bash         # this workspace
   source .venv/bin/activate                              # Python packages
   export ROS_DOMAIN_ID=156                               # must match the Teensy

``ROS_DOMAIN_ID`` is not decoration. The Teensy firmware joins domain **156**, and nodes on
different domains cannot see each other, so a wrong value here produces a system that starts
cleanly and shows nothing.

In the commands below, ``N`` is your group number. Use ``0`` if you have not been given one.

Method 1 — Run without hardware
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Use this for debugging and development.** ``mock_ui`` stands in for *both* the Teensy and
the robot: it publishes ``encoder_raw`` and it sends the two LSL streams. There is **no**
micro-ROS agent, **no** Base System back-end, and **no** front-end UI in this mode.

.. code-block:: text

   Terminal 1 - mock_ui.py                Terminal 2 - ros2 launch bringup
     (Tkinter window)                       encoder_reader
        |  /G<N>/encoder_raw  ------------>      |  /G<N>/estimated_states
        |                                        +--> experiment_evaluator
        |  LSL ActualStates_N               web_visualizer
        |      EventTrigger_N  ----------->      |
                                                 v
                                        Verifier UI  http://localhost:8000

**Terminal 1 — the mock UI.** Note it is started with ``python3``; it is not installed as a
ROS executable:

.. code-block:: bash

   cd claude-visualizer-ws
   python3 src/claude_visualizer/mock_UI/mock_ui.py --group-number N

**Terminal 2 — the ROS 2 pipeline:**

.. code-block:: bash

   cd claude-visualizer-ws
   ros2 launch claude_visualizer bringup.launch.py group_number:=N

Open http://localhost:8000. Drag the knobs in the mock UI window and the charts move.

The knobs, the sync toggle, the reset buttons and the command field are described on the
``mock_ui_script.rst`` page.

Method 2 — Run with hardware
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**This is the full system.** Plug in the Teensy and the robot, identify their serial ports
(below), then start the four terminals in order. Each one uses the environment preparation above.

Identify the serial ports
^^^^^^^^^^^^^^^^^^^^^^^^^

Both the Teensy and the robot appear as ``/dev/ttyACM*`` devices, and the numbering depends on
the order they were plugged in — so you have to find out which is which. The simplest way is to
plug them in **one at a time**:

.. code-block:: bash

   ls /dev/ttyACM*        # nothing plugged in: "No such file or directory"
   # plug in the Teensy, then:
   ls /dev/ttyACM*        # the new entry is the Teensy   (e.g. /dev/ttyACM0)
   # plug in the robot, then:
   ls /dev/ttyACM*        # the second entry is the robot (e.g. /dev/ttyACM1)

If both are already connected, ``udevadm`` tells them apart — the Teensy reports a
Teensyduino / PJRC model, the robot an STMicroelectronics one:

.. code-block:: bash

   udevadm info -q property -n /dev/ttyACM0 | grep -E "ID_VENDOR|ID_MODEL"

The two ports are used in two different places:

- The **micro-ROS agent** (Terminal 1) reads the **Teensy** — give it that port with ``--dev``.
- The **Base System back-end** takes **no** port on the command line. You connect the **robot**
  from the browser UI (Terminal 4): in the connect dialog, enter the robot's ``ttyACM``
  **number** and the slave address **21**. The back-end builds ``/dev/ttyACM{number}`` from it —
  so a robot at ``/dev/ttyACM1`` means you enter port **1**.

.. code-block:: text

   Terminal 1 - micro-ROS agent
     Teensy 4.1  --USB serial 115200-->  micro_ros_agent  -->  /G<N>/encoder_raw

   Terminal 2 - ROS 2 pipeline  (ros2 launch claude_visualizer bringup.launch.py)
     /G<N>/encoder_raw --> encoder_reader --> /G<N>/estimated_states
                                                  |
                                                  +--> experiment_evaluator --> /G<N>/eval_*
                                                  +--> web_visualizer --> Verifier UI :8000

   Terminal 3 - Base System back-end  (python3 server_111.py --group_number N)
     Robot (STM32) <--Modbus RTU 230400--> server_111.py
                                              +--WebSocket :8765--> Front-end UI
                                              +--LSL ActualStates_N / EventTrigger_N
                                                                    --> web_visualizer

   Terminal 4 - Front-end UI  (docker compose up -d)
     Browser :3000 <-- React UI container

**Terminal 1 — micro-ROS agent (Teensy bridge).** Use *your* Teensy port (``/dev/ttyACM0`` here):

.. code-block:: bash

   cd claude-visualizer-ws
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

**Terminal 2 — ROS 2 pipeline:**

.. code-block:: bash

   cd claude-visualizer-ws
   ros2 launch claude_visualizer bringup.launch.py group_number:=N

**Terminal 3 — Base System back-end:**

.. code-block:: bash

   cd FRA263-264_BaseSystem_BackEnd
   python3 server_111.py --group_number N

**Terminal 4 — Base System front-end:**

.. code-block:: bash

   cd FRA263-264_BaseSystem_FrontEnd
   docker compose up -d

Then open both pages: the verifier dashboard at http://localhost:8000 and the Base System UI at
http://localhost:3000. In the Base System UI, connect to the robot using its port **number** and
slave **21** — see "Identify the serial ports" above.

Ports and addresses
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 26 44

   * - Link
     - Address
     - Used by
   * - Verifier dashboard (HTTP)
     - ``http://localhost:8000``
     - ``web_visualizer`` serves the page.
   * - Verifier live data (WebSocket)
     - ``ws://localhost:9090``
     - The dashboard finds this itself from ``/config.json``.
   * - Base System UI (HTTP)
     - ``http://localhost:3000``
     - The front-end container.
   * - Base System back-end (WebSocket)
     - ``ws://localhost:8765``
     - Between the front-end UI and ``server_111.py``.
   * - Teensy (USB serial)
     - ``/dev/ttyACM*``, 115200
     - Between the Teensy and ``micro_ros_agent``.
   * - Robot (Modbus RTU)
     - ``/dev/ttyACM*``, 230400 8-E-1, slave 21
     - Between ``server_111.py`` and the STM32.

The web ports are fixed. Different machines have different IP addresses, so groups do not
need different ports.

Keeping one group consistent
----------------------------

**One number has to agree in four places.** The group number sets the ROS namespace
``/G<N>/``, the LSL stream suffix ``_N``, and the row read from ``criteria.yaml``. If two of
these disagree, the parts start normally but never find each other.

.. list-table::
   :header-rows: 1
   :widths: 34 46 20

   * - Where
     - How it is set
     - Method
   * - ROS 2 pipeline
     - ``ros2 launch ... group_number:=N``
     - 1 and 2
   * - Mock UI
     - ``python3 mock_ui.py --group-number N``
     - 1
   * - Base System back-end
     - ``python3 server_111.py --group_number N``
     - 2
   * - Teensy firmware
     - ``GROUP_NUMBER`` in the firmware source
     - 2

.. warning::

   **The three tools spell the option differently.** The launch file takes
   ``group_number:=N`` (a ROS launch argument), the Base System back-end takes
   ``--group_number N`` with an **underscore**, and the mock UI takes ``--group-number N``
   with a **hyphen**. Using the wrong one is not silently ignored — the mock UI exits with
   ``unrecognized arguments``. Copy the exact line from the run method above.

``N = 0`` is the default: namespace ``/G0/``, no LSL suffix, and the ``default`` criteria row.
On top of this, ``ROS_DOMAIN_ID`` must be **156** on every machine, matching the firmware.

Troubleshooting
---------------

.. list-table::
   :header-rows: 1
   :widths: 34 30 36

   * - Symptom
     - Cause
     - Fix
   * - ``ros2: command not found``
     - ROS 2 not sourced in this terminal.
     - ``source /opt/ros/jazzy/setup.bash``
   * - ``ModuleNotFoundError: No module named 'pylsl'``
     - The virtual environment is not active.
     - ``source .venv/bin/activate`` (from the project root)
   * - ``ModuleNotFoundError: No module named 'rclpy'``
     - ROS 2 not sourced (the environment alone does not provide it).
     - Source ROS 2 as well — see :ref:`environment-prep`.
   * - ``colcon build``: ``No module named 'catkin_pkg'`` / ``'em'`` / ``'lark'``
     - The build tooling is not in the active virtual environment.
     - Activate ``.venv`` and re-run ``pip install -r requirements.txt`` (Step 4).
   * - ``Package 'micro_ros_agent' not found``
     - Step 5 was skipped — micro-ROS is not part of the clone.
     - Run Step 5.
   * - ``Permission denied: '/dev/ttyACM0'``
     - The user is not in the ``dialout`` group.
     - ``sudo usermod -aG dialout $USER``, then log out and back in.
   * - Everything starts, but the charts stay empty
     - Group number or ``ROS_DOMAIN_ID`` does not match across the parts.
     - Check all four places above, and ``ROS_DOMAIN_ID=156`` everywhere.
   * - A node runs old code, or a renamed file is still loaded
     - The workspace was built with ``--symlink-install``.
     - Delete ``build/``, ``install/``, ``log/`` and run ``colcon build`` again.
   * - Port 3000 or 8765 already in use
     - Another copy of the UI or back-end is still running.
     - ``docker compose down``; stop the other ``server_111.py``.
   * - ``colcon: command not found``
     - The colcon package is missing.
     - ``sudo apt install python3-colcon-common-extensions``

Where to go next
----------------

- :doc:`system_overview` — the architecture and the overview of the system.
- :doc:`verification_system/verification_system` — the design of each node in Verification System.
- :doc:`verification_system/teensy_firmware` — the Teensy 4.1 firmware in detail.
