# What the adaptive controller has learned the vehicle can do, for planners.
# Published by ackermann_adaptive_controller at ~1 Hz and on change.
std_msgs/Header header
# Tightest turn the car can drive, the worse side (m). Until turning_confirmed
# this is the model's derated (larger, conservative) extrapolation.
float32 min_turning_radius
bool turning_confirmed
# Slowest speed the current surface lets the car hold (m/s), capped at the
# operating speed; 0 = unknown.
float32 creep_speed
# Time budget for one start from rest on the current surface (s): the learned
# breakaway time plus half the command-to-motion delay, so that two starts
# budget one full delay; 0 = unknown.
float32 launch_time
