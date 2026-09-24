# What the adaptive controller has learned the vehicle can do, for planners.
# Published by ackermann_adaptive_controller at ~1 Hz and on change.
std_msgs/Header header
# Tightest turn the car can drive, the worst of the four below (m).
float32 min_turning_radius
# Tightest turn per travel direction and side (m). Left means curvature
# (yaw rate / speed) > 0, whichever way the car travels. A side not yet
# driven at full lock is the steering model's prediction.
float32 min_turning_radius_forward_left
float32 min_turning_radius_forward_right
float32 min_turning_radius_reverse_left
float32 min_turning_radius_reverse_right
# Both forward sides have been driven at full lock (measured, not predicted).
bool turning_confirmed
# Slowest speed the current surface lets the car hold (m/s), capped at the
# operating speed; 0 = unknown.
float32 creep_speed
# Time budget for one start from rest on the current surface (s): the learned
# breakaway time plus half the command-to-motion delay, so that two starts
# budget one full delay; 0 = unknown.
float32 launch_time
