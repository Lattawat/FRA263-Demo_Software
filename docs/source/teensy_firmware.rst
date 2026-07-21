Teensy 4.1 Firmware (Encoder Publisher)
=======================================

.. Node description (the design idea, detail, and other crucial info)

This firmware is the real hardware source of the ``EncoderRaw`` message. It runs on a
**Teensy 4.1** microcontroller, reads the quadrature encoder mounted on the test rig, and
publishes the raw tick count as a micro-ROS node named ``encoder_data_publisher``. On a real
setup this is what replaces the ``mock_encoder`` and ``mock_ui`` publishers that stand in for
the encoder when the system is run without hardware.

A microcontroller cannot join a ROS 2 network on its own. It speaks **micro-ROS** — a cut-down
ROS 2 client library for embedded devices — and reaches the full ROS 2 graph through the
**micro-ROS agent**, a small program running on the host PC that bridges the Teensy's serial
link to normal ROS 2 topics.

.. code-block:: text

   encoder A/B        ┌──────────────────────────┐   USB serial   ┌────────────────────┐    /G<N>/encoder_raw
   channels   ───────▶│  Teensy 4.1              │───────────────▶│  micro-ROS agent   │───▶ (EncoderRaw)
   (pins 7, 6)        │  encoder_data_publisher  │  (XRCE-DDS)    │  (host PC)         │     ticks, raw_position,
                      │  micro-ROS node          │                └────────────────────┘     dt_us
                      └──────────────────────────┘

The Teensy and the agent talk over USB serial using **XRCE-DDS** (the lightweight transport
micro-ROS uses in place of full DDS). The agent is the only part that touches the real ROS 2
graph, so every other node — the State Estimator, the Web Visualizer — sees ``/G<N>/encoder_raw``
as an ordinary topic and never knows a microcontroller produced it.

**Key settings.** These are fixed in the firmware and the build config, and several of them
*must* match the host PC or the messages never arrive.

.. list-table::
   :header-rows: 1
   :widths: 26 18 56

   * - Setting
     - Value
     - Meaning
   * - ``EN_RES``
     - ``8192``
     - Encoder resolution in ticks per revolution. Used by ``ticks2rad`` to convert a tick
       count into radians. Matches ``ticks_per_rev`` on the State Estimator side.
   * - Encoder pins
     - A = 7, B = 6
     - The two quadrature channels read by the ``Encoder`` library.
   * - Publish rate
     - 100 Hz
     - Set by a 10 ms timer. One ``EncoderRaw`` message goes out every tick of that timer.
   * - ``dt_us``
     - ``10000``
     - The sample interval reported in each message. It is **hard-coded** to 10000 µs
       (10 ms), not measured, so it always agrees with the 100 Hz timer.
   * - ``GROUP_NUMBER``
     - ``1``
     - Sets the node namespace ``/G<N>``. Must match the ``group_number`` the rest of the
       system is launched with, or the topics will not line up.
   * - Domain ID
     - ``156``
     - The ROS 2 domain the node joins. Must equal ``ROS_DOMAIN_ID`` on the host PC —
       different domains cannot see each other.
   * - Transport
     - serial @ 115200
     - USB serial to the agent, 115200 baud. Set in both the firmware and ``platformio.ini``.

**Interfaces.** The firmware has one interface:

- **Publishes** ``encoder_raw`` (``EncoderRaw``: ``ticks``, ``raw_position``, ``dt_us``, and
  the standard ``header``) — the entry point of the whole measurement chain; everything
  downstream is derived from it. ``ticks`` is the cumulative signed count read straight from
  the encoder, ``raw_position`` is that count run through ``ticks2rad``, and ``dt_us`` is the
  fixed 10000 µs described above. The topic name is written **relative** (``encoder_raw``, no
  leading slash), so the node namespace ``/G<N>`` is added in front to give
  ``/G<N>/encoder_raw``.

Each message is time-stamped with ``rmw_uros_epoch_nanos()`` — the agent's clock, not the
Teensy's. A microcontroller has no real-time clock of its own; on power-up its time starts
from zero. The call ``rmw_uros_sync_session()`` in ``setup()`` syncs the node to the agent's
epoch once at start, so from then on every stamp is real wall-clock time. Without that sync
the stamps would be boot-relative and useless for lining this stream up against the others.

How the custom interface is used on the microcontroller
-------------------------------------------------------

The firmware publishes ``EncoderRaw``, which is one of *our* message types from the
``claude_visualizer_interface`` package — not a standard ROS 2 message. Getting a custom
message onto a microcontroller works differently from a normal ROS 2 package, and this is the
part most worth understanding.

On a desktop, adding a message package is just a build dependency: the message headers are
generated when the workspace is built and linked in at run time. In micro-ROS there is no run
time to link against. The **entire micro-ROS C library is cross-compiled into a single static
``.a`` file** when PlatformIO builds the firmware. A custom message therefore has to be part
of *that* build — it cannot be added afterwards. Three pieces make this happen.

**1. ``platformio.ini`` — the build and transport config.**

.. code-block:: ini

   lib_deps = https://github.com/micro-ROS/micro_ros_platformio
   board_microros_distro = jazzy
   board_microros_transport = serial
   board_microros_user_meta = colcon.meta

- ``lib_deps`` pulls the ``micro_ros_platformio`` library from GitHub; it is what runs the
  cross-compilation.
- ``board_microros_distro = jazzy`` picks which ROS 2 distribution's message definitions are
  generated. It **must match the ROS 2 distro on the host PC** running the agent, or the two
  sides disagree about the message layout.
- ``board_microros_transport = serial`` wires in the USB-serial transport.
- ``board_microros_user_meta = colcon.meta`` points the build at the extra config file
  described below.

**2. The ``extra_packages`` symlink — one definition, built twice.**

The build script always looks for custom packages in a folder named ``extra_packages`` next to
``platformio.ini``. Instead of copying the message package in, the project puts a **symlink**
there that points back to the real package in the ROS 2 workspace:

.. code-block:: text

   encoder_data_publisher/extra_packages/claude_visualizer_interface
       → ../../src/claude_visualizer_interface

Because it is a symlink, there is only ever **one** copy of the ``.msg`` files. The host-side
nodes and the firmware are both built from the same definition, so ``EncoderRaw`` can never
drift out of sync between the two. If the messages were copied instead, every edit would have
to be copied again by hand.

**3. ``colcon.meta`` — list the package so it is compiled in.**

.. code-block:: json

   {
       "names": {
           "micro_ros_msgs": {},
           "claude_visualizer_interface": {}
       }
   }

This is the file ``board_microros_user_meta`` pointed at. Listing
``claude_visualizer_interface`` here is what tells the cross-compile step to build our package
into the static library alongside the standard messages. The empty ``{}`` means "use the
default build settings, no overrides."

.. note::

   After changing ``colcon.meta`` you must clean the micro-ROS library before rebuilding
   (``pio run -t clean_microros`` then ``pio run``). An incremental build sees no changed
   ``.cpp`` source and skips recompiling the library, silently ignoring the new message. The
   full step-by-step, including the ``package.xml`` rules a micro-ROS interface package must
   follow, lives in ``MICROROS_NOTES.md`` in the firmware project.

**The C naming rule.** Once the package is compiled in, the message is used through C names,
not the ROS 2 CamelCase names. The generator flattens
``claude_visualizer_interface/msg/EncoderRaw`` into:

.. list-table::
   :header-rows: 1
   :widths: 36 64

   * - Use
     - C form
   * - Type
     - ``claude_visualizer_interface__msg__EncoderRaw``
   * - Include path
     - ``claude_visualizer_interface/msg/encoder_raw.h`` (snake_case file name)
   * - Type support
     - ``ROSIDL_GET_MSG_TYPE_SUPPORT(claude_visualizer_interface, msg, EncoderRaw)``

These three forms are exactly what you will see in the code below.

Node Workflow
-------------

.. The flow chart of this whole firmware

A micro-ROS program has the same two-function shape as any Arduino sketch: ``setup()`` runs
once to build everything, then ``loop()`` runs forever. All the real work happens inside a
timer callback that the executor fires every 10 ms.

.. code-block:: text

   setup()  (runs once)
     │
     ├─ Serial.begin + set_microros_serial_transports   (open the USB link to the agent)
     ├─ init_options + set_domain_id(156)               (join the same ROS domain as the host)
     ├─ rclc_support_init                               (start the micro-ROS session)
     ├─ rmw_uros_sync_session(1000)                     (sync clock to the agent's epoch)
     ├─ rclc_node_init_default(... ROS_NAMESPACE ...)   (create node under /G<N>)
     ├─ rclc_executor_init                              (create the callback scheduler)
     ├─ rclc_publisher_init_default(... "encoder_raw")  (publisher on relative topic)
     ├─ rclc_timer_init_default(10 ms → timer_cb)       (create the 10 ms timer)
     └─ rclc_executor_add_timer                         (hand the timer to the executor)

   loop()  (runs forever)
     │
     └─ rclc_executor_spin_some(20 ms)  ──────▶  every 10 ms the executor fires timer_cb
                                                    │
                                                    ▼
                                          ┌──────────────────────────────────────┐
                                          │ timer_cb()                            │
                                          │   stamp  = rmw_uros_epoch_nanos()     │
                                          │   ticks  = encoder.read()             │
                                          │   raw_position = ticks2rad(ticks)     │
                                          │   dt_us  = 10000                      │
                                          │   publish /G<N>/encoder_raw           │
                                          └──────────────────────────────────────┘

The one part that is easy to get wrong is why ``loop()`` calls ``rclc_executor_spin_some`` and
not the plain ``spin``. The **executor** is micro-ROS's callback scheduler: it holds the list
of registered callbacks (here, just the timer) and runs the ones that are ready. ``spin`` would
loop forever inside itself and never give control back, which would freeze Arduino's own
``loop()``. ``spin_some`` instead does one pass — run whatever is ready, wait up to a short
timeout, then return — so ``loop()`` keeps cycling and any other Arduino code could still run.
The 20 ms timeout is comfortably longer than the 10 ms timer period, so a firing is never
missed.

Examine the code
----------------

The full firmware lives in ``encoder_data_publisher/src/main.cpp``. This section walks through
it block by block.

**Pins and the tick-to-radian macro.**

.. code-block:: cpp

   #define LED_PIN 13
   #define EN_CH_A 7
   #define EN_CH_B 6
   #define EN_RES 8192.0
   #define ticks2rad(ticks) ((ticks/EN_RES)*2.0*PI)

These name the hardware pins and the encoder resolution. ``EN_RES`` is the number of ticks in
one full turn, so ``ticks2rad`` converts a tick count to radians the same way the State
Estimator does: one revolution is ``EN_RES`` ticks and ``2π`` radians, so the angle is
``ticks / EN_RES × 2π``. ``LED_PIN`` is the on-board LED, used later as an error light.

**The group namespace.**

.. code-block:: cpp

   #define GROUP_NUMBER 1
   #define STR2(x) #x
   #define STR(x)  STR2(x)
   #define ROS_NAMESPACE "G" STR(GROUP_NUMBER)     // → "G0", "G5", …

This turns the group number into the namespace string ``"G1"``. The two-step
``STR``/``STR2`` trick is needed because the C preprocessor's ``#`` stringize operator would
otherwise turn ``GROUP_NUMBER`` into the literal text ``"GROUP_NUMBER"`` instead of its value.
The inner macro expands ``GROUP_NUMBER`` to ``1`` first, then the outer one stringizes that to
``"1"``, and the adjacent string literals ``"G"`` and ``"1"`` join into ``"G1"`` at compile
time. Set ``GROUP_NUMBER`` per group and reflash.

**Includes and the transport guard.**

.. code-block:: cpp

   #include <micro_ros_platformio.h>

   #include <rcl/rcl.h>
   #include <rclc/rclc.h>
   #include <rclc/executor.h>

   #include <std_msgs/msg/float32_multi_array.h>
   #include <claude_visualizer_interface/msg/encoder_raw.h>
   #include <rmw_microros/rmw_microros.h>

   #include "Encoder.h"

   #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
   #error This script is only available for Arduino framework with serial transport.
   #endif

``rcl`` is the low-level ROS 2 client library in C, and ``rclc`` adds the executor and the
simpler ``_default`` init helpers on top of it. The custom message comes in through
``claude_visualizer_interface/msg/encoder_raw.h`` — the snake_case header from the naming rule
above. ``rmw_microros.h`` provides the clock-sync and epoch calls. The ``#error`` guard stops
the build early with a clear message if the project is ever configured for a transport other
than Arduino serial, instead of failing later with a confusing linker error.

**Global entities and the return-code checks.**

.. code-block:: cpp

   claude_visualizer_interface__msg__EncoderRaw encoder_raw;

   rclc_support_t support;
   rcl_allocator_t allocator = rcl_get_default_allocator();
   rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
   rcl_node_t node = rcl_get_zero_initialized_node();
   rcl_timer_t timer = rcl_get_zero_initialized_timer();
   rcl_publisher_t encoder_data_publisher = rcl_get_zero_initialized_publisher();

   #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){_error_handler();}}
   #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

``encoder_raw`` is the one message object, filled in and reused every cycle. Below it are the
micro-ROS entities. Three of them are the foundation objects every micro-ROS program needs: the
**allocator** (how micro-ROS asks for and frees memory — spelled out here because a
microcontroller has no garbage collector), the **support** object (the session, i.e. the
connection to ROS 2), and the **executor** (the callback scheduler). The node, timer, and
publisher are created from those. Each ROS call returns a status code, and the two macros
check it: ``RCCHECK`` treats a failure as fatal and jumps to the error handler, while
``RCSOFTCHECK`` notes it but carries on — used for calls like publishing where one dropped
sample is not worth halting the device. (``MICROROS_NOTES.md`` explains the foundation objects
and the zero-initialize pattern in more depth.)

.. code-block:: cpp

   void _error_handler(){
     uint32_t _error_handler_timestamp = millis();
     uint8_t _error_handler_timestep = 200;
     while(true){
       if (millis() - _error_handler_timestamp > _error_handler_timestep){
         _error_handler_timestamp = millis();
         digitalToggle(LED_PIN);
       }
     }
   }

If a fatal call fails, the firmware stops here and blinks the on-board LED forever, toggling it
every 200 ms. There is no screen on a microcontroller, so a steady blink is the signal that
setup failed — usually because the agent is not running or the domain ID does not match.

**The timer callback.** This is the heart of the firmware; it runs once every 10 ms.

.. code-block:: cpp

   void timer_cb(rcl_timer_t *timer, int64_t last_call_time){
     RCLC_UNUSED(last_call_time);

     if(timer != NULL){
       int64_t time_since_last_call;
       RCSOFTCHECK(rcl_timer_get_time_since_last_call(timer, &time_since_last_call));

       // Stamp using synced epoch time
       int64_t nanos = rmw_uros_epoch_nanos();
       encoder_raw.header.stamp.sec     = (int32_t)(nanos / 1000000000LL);
       encoder_raw.header.stamp.nanosec = (uint32_t)(nanos % 1000000000LL);

       ticks = test_station_encoder.read();
       encoder_raw.ticks        = ticks;
       encoder_raw.raw_position = ticks2rad(ticks);
       // encoder_raw.dt_us        = (time_since_last_call % 1000000000) / 1000;
       encoder_raw.dt_us        = 10000;

       RCSOFTCHECK(rcl_publish(&encoder_data_publisher, &encoder_raw, NULL));
     }
   }

Each firing stamps the message with the synced epoch time (split into whole seconds and
leftover nanoseconds, the shape the ``header`` expects), reads the current tick count from the
encoder, converts it to radians, and publishes. Note the commented-out line: the firmware
*could* report the real measured interval from ``rcl_timer_get_time_since_last_call``, but
instead ``dt_us`` is fixed at ``10000``. The 10 ms timer is steady enough that a constant value
is simpler and keeps the State Estimator's maths predictable, so the measured value is left out
on purpose.

**Setup — session, node, and time sync.**

.. code-block:: cpp

   void setup(){
     //config serial port
     Serial.begin(115200);
     set_microros_serial_transports(Serial);
     ...
     rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
     RCCHECK(rcl_init_options_init(&init_options, allocator));
     RCCHECK(rcl_init_options_set_domain_id(&init_options, 156)); // match ROS_DOMAIN_ID on host
     RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
     rmw_uros_sync_session(1000);

First the USB serial link to the agent is opened at 115200 baud. Then the ROS domain is set to
156 — this **must** equal ``ROS_DOMAIN_ID`` on the host, since nodes on different domains are
invisible to each other. ``rclc_support_init_with_options`` starts the micro-ROS session with
those options, and ``rmw_uros_sync_session`` performs the one-time clock sync so the timestamps
in every message are real wall-clock time.

**Setup — the node under the group namespace.**

.. code-block:: cpp

     RCCHECK(rclc_node_init_default(
         &node,
         "encoder_data_publisher",
         ROS_NAMESPACE,                      // now /G<GROUP_NUMBER>
         &support
       )
     );

This creates the node named ``encoder_data_publisher`` and places it under the namespace
``ROS_NAMESPACE`` (``/G1`` for group 1). Everything the node publishes is prefixed with that
namespace, which is how each group's firmware stays on its own set of topics.

**Setup — the publisher.**

.. code-block:: cpp

     RCCHECK(rclc_publisher_init_default(
         &encoder_data_publisher,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(claude_visualizer_interface, msg, EncoderRaw),
         // "/encoder_raw"                   // was absolute (ignored the namespace)
         "encoder_raw"                       // relative → /G<GROUP_NUMBER>/encoder_raw
       )
     );

The publisher is bound to the custom message through the type-support macro — the third C form
from the naming rule. The topic name is the **relative** ``"encoder_raw"``, so the node
namespace is added in front to give ``/G<N>/encoder_raw``. The commented-out ``"/encoder_raw"``
is the earlier absolute version: a leading slash makes the name absolute, which ignores the
namespace and would put every group on the same topic — exactly what the relative name avoids.

**Setup — the timer and executor.**

.. code-block:: cpp

     const unsigned int timestep = 10; //[ms]
     RCCHECK(rclc_timer_init_default(
         &timer,
         &support,
         RCL_MS_TO_NS(timestep),
         timer_cb
       )
     );

     RCCHECK(rclc_executor_add_timer(&executor, &timer));
   }

The timer is created with a 10 ms period (converted to nanoseconds by ``RCL_MS_TO_NS``) and
told to call ``timer_cb`` each time it fires. The last line registers the timer with the
executor, so from now on the executor knows to run ``timer_cb`` whenever 10 ms has passed. The
executor itself was created just above this block with room for one handle — the single timer.

**Loop.**

.. code-block:: cpp

   void loop(){
     RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
   }

The main loop does one thing: give the executor a chance to run any ready callback, waiting up
to 20 ms. Arduino calls ``loop()`` over and over on its own, so this repeated ``spin_some`` is
what keeps the 10 ms timer firing and the messages flowing. (The original file also has a
commented-out serial-debug branch that printed raw ticks to the serial monitor instead of
publishing — handy while bringing the board up, switched off in normal operation.)

Notation
--------

Words used on this page.

micro-ROS
    A cut-down version of ROS 2 for microcontrollers. It uses the ``rclc`` C library and a
    lightweight transport instead of full DDS.

micro-ROS agent
    A small program on the host PC that bridges a micro-ROS device to the real ROS 2 network.
    The Teensy talks only to the agent; the agent publishes the actual topics.

rcl vs rclc
    ``rcl`` is the low-level ROS 2 client library in C — it owns the node, publisher and timer
    types. ``rclc`` sits on top and adds the executor and the simpler ``_default`` init helpers.

executor / spin_some
    The **executor** is the scheduler that runs registered callbacks. ``spin_some`` runs the
    ready ones once and returns, so the Arduino ``loop()`` keeps cycling — unlike ``spin``,
    which never returns.

allocator / support
    The **allocator** is how micro-ROS asks for and frees memory (spelled out because there is
    no garbage collector on the chip). The **support** object is the session — the connection
    to ROS 2 that the node and timer are created from.

type support
    A description of a message's layout that a publisher needs in order to send it. Obtained
    with the ``ROSIDL_GET_MSG_TYPE_SUPPORT`` macro.

XRCE-DDS
    The lightweight transport micro-ROS uses over serial in place of full DDS. It is what the
    Teensy and the agent speak to each other.

domain ID
    A number that partitions a ROS 2 network. Only nodes on the same domain ID can see each
    other, so the firmware's ``156`` must match ``ROS_DOMAIN_ID`` on the host.

quadrature encoder / ticks
    A sensor that reports shaft rotation as a stream of counts (**ticks**) on two channels.
    The count is cumulative and signed; ``ticks2rad`` turns it into an angle.

extra package / colcon.meta
    ``extra_packages/`` is the folder where the micro-ROS build looks for custom message
    packages, and ``colcon.meta`` is the file that lists which of them to compile into the
    firmware's static library.

namespace ``/G<N>``
    The prefix added to every relative topic name, set from ``GROUP_NUMBER``. It keeps each
    group's topics separate on a shared network.
