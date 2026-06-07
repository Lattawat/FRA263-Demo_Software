# Trigger signal from robot_controller.py via LSL marker stream,
# bridged into ROS 2 by web_visualizer node.

std_msgs/Header header          # timestamp assigned by the bridge on receipt
string  event                   # Full JSON payload string. Must contain an "event" key
                                # (the type discriminator). All other keys are event-specific.
