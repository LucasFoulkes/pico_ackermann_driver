# Robot interfaces

`ActuatorState` reports the timestamp and normalized steering/throttle values
successfully written by the Pico driver, serial connectivity, channel activity,
and conflicting ROS command ownership. The adaptive controller consumes it.

This is serial-delivery evidence, not wheel/servo feedback or a firmware ACK.
The package defines messages only and launches no node.
