# Actual robot state telemetry.
# Phase 2: produced by mock_robot_controller.py via LSL and bridged into ROS 2 by
# web_visualizer. Fields can be extended as the robot controller scope grows.

std_msgs/Header header          # timestamp assigned by the source / bridge

# Actual states (measured / computed by the controller)
float64 actual_position         # actual position     [rad or m]
float64 actual_velocity         # actual velocity     [rad/s or m/s]
float64 actual_acceleration     # actual acceleration [rad/s² or m/s²]
