# Raw quadrature encoder ticks published by micro-ROS on Teensy 4.1.
# In Phase 1 this is published by mock_encoder.py instead.

std_msgs/Header header          # ROS 2 standard header (timestamp + frame_id)
int32   ticks                   # cumulative tick count (signed)
float64 raw_position            # position converted from ticks [rad or m depending on setup]
uint32  dt_us                   # microseconds since last sample
