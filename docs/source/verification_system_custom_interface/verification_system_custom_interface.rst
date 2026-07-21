Custom Interface
================

.. The layout of each message section follows the ROS message documentation format,
   for example "https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html"

The Verification System does not use only the standard ROS 2 message types. It defines
its own **interface package**, ``claude_visualizer_interface``, which holds five message
types and one service type. 

An interface package is a package that contains **only** definitions — no node code. It is
built with ``ament_cmake`` and the ``rosidl_generate_interfaces`` macro, which reads every
``.msg`` and ``.srv`` file listed in ``CMakeLists.txt`` and generates the matching C++ and
Python classes at build time:

.. code-block:: cmake

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/EncoderRaw.msg"
     "msg/EncoderState.msg"
     "msg/EventTrigger.msg"
     "msg/ActualStates.msg"
     "msg/ExperimentEval.msg"
     "srv/UpdateCriteria.srv"

     DEPENDENCIES builtin_interfaces std_msgs
   )

These custom messages are directly referenced by all of the ROS 2 nodes and referenced
by the firmware in the ``/claude-visualizer-ws/encoder_data_publisher`` directory to help
during the firmware building process. So a message built on the microcontroller can be understood
by a Python node without anyone writing a converter.

In code the types are referred to by package and kind, for example
``claude_visualizer_interface/msg/EncoderRaw`` or
``claude_visualizer_interface/srv/UpdateCriteria``.

Summary
-------

.. list-table::
   :header-rows: 1
   :widths: 18 8 20 22 32

   * - Interface
     - Kind
     - Topic / service
     - Published by
     - Purpose
   * - ``EncoderRaw``
     - msg
     - ``encoder_raw``
     - Teensy firmware, ``mock_ui``
     - Raw encoder ticks straight from the hardware.
   * - ``EncoderState``
     - msg
     - ``estimated_states``
     - ``encoder_reader``
     - Raw unit converted position and Kalman-filtered velocity and acceleration.
   * - ``ActualStates``
     - msg
     - ``actual_states``
     - ``web_visualizer``, ``mock_ui``
     - The actual states of the robot.
   * - ``EventTrigger``
     - msg
     - ``event_trigger``
     - ``web_visualizer``, ``mock_ui``
     - The command that starts, stops or skips a test.
   * - ``ExperimentEval``
     - msg
     - ``eval_live``, ``eval_summary``
     - ``experiment_evaluator``
     - Live metrics during a run, and the final pass/fail verdict.
   * - ``UpdateCriteria``
     - srv
     - ``update_criteria``
     - server: ``experiment_evaluator``
     - Read and change the pass/fail criteria while the system is running.

.. note::
  
  Every topic name above is written as a relative name (no leading slash). A relative
  name is expanded with the namespace specified during the launch process, so under the launch
  namespace ``/G7/`` the topic ``encoder_raw`` becomes ``/G7/encoder_raw``. This is how we handle the
  concurrent multi-machine running on one network without the topics colliding.

.. toctree::
  :maxdepth: 2

  EncoderRawMSG.rst
  EncoderStateMSG.rst
  ActualStatesMSG.rst
  EventTriggerMSG.rst
  ExperimentEvalMSG.rst
  UpdateCriteriaSRV.rst
  JSONpayloads.rst


Notation
--------

Words used on this page.

interface package
    A ROS 2 package that contains only message and service definitions, no runnable code.
    Other packages depend on it to share one common data layout.

rosidl
    The ROS 2 code generator. It reads ``.msg`` and ``.srv`` files and writes the C++ and
    Python classes that nodes actually use, so the definition is written once.

field
    One named entry inside a message — a type and a name, such as ``float64 velocity``.

message vs service
    A **message** travels one way on a topic, and the sender never learns who received it.
    A **service** is a request/response pair: the caller waits and gets an answer back.

request / response
    The two halves of a service definition, separated by the ``---`` line. Above it is what
    the client sends, below it is what the server sends back.

payload
    The actual content carried inside a field — here, the JSON object packed into a
    ``string``.

type discriminator
    The key (or keys) a receiver reads first to work out what kind of payload it is holding,
    so it knows how to read the rest. In this system it is the ``mode`` + ``action`` pair.

loopback
    When a node subscribes to a topic it also publishes on. It is done on purpose here, so
    that messages from *any* producer — not only the node's own bridge — still reach the
    browser.

relative vs absolute topic name
    An **absolute** name starts with a slash (``/encoder_raw``) and is used exactly as
    written. A **relative** name has no leading slash (``encoder_raw``) and gets the node's
    namespace added in front, becoming ``/G7/encoder_raw`` under the namespace ``/G7/``.
    This system uses relative names everywhere so that several groups can run at once.

variance
    The filter's own measure of how uncertain it is about a state. A small variance means
    the estimate has converged; it is useful while tuning, but nothing in the system makes
    a decision from it.
