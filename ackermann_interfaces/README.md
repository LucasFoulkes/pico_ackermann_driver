# Ackermann interfaces

`ActuatorState` reports the timestamp and normalized steering/throttle values
successfully written by the Pico driver, connectivity (`connected`: the port is
open and the Pico runs its firmware), channel activity, and conflicting ROS
command ownership. The adaptive controller consumes it.

This is serial-delivery evidence, not wheel/servo feedback or a firmware ACK.

`VehicleCapability` reports what the adaptive controller has learned the
vehicle can do (turning radii per direction and side, creep speed, launch
time), for planners. The controller publishes it.

The package defines messages only and launches no node.
