# Kalman-filtered kinematic estimates published by encoder_reader node.
# Consumers: web_visualizer node, robot_controller.py (via LSL re-pub).

std_msgs/Header header          # ROS 2 standard header (timestamp + frame_id)

float64 position                # estimated position  [rad or m depending on setup]
float64 velocity                # estimated velocity  [rad/s or m/s]
float64 acceleration            # estimated acceleration [rad/s² or m/s²]

# Kalman filter diagnostics (optional but useful during tuning)
float64 pos_variance            # posterior variance — position
float64 vel_variance            # posterior variance — velocity
float64 acc_variance            # posterior variance — acceleration

# Raw tick count passed through for logging / sanity checks
int32   raw_ticks
