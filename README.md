# Pico Ackermann driver and interfaces

This repository contains two ROS 2 packages:

- [`pico_ackermann_driver`](pico_ackermann_driver/README.md): USB serial driver, Pico firmware, configuration, and driver tests.
- [`ackermann_interfaces`](ackermann_interfaces/README.md): shared messages: `ActuatorState` (the driver's delivery status, read by the adaptive controller) and `VehicleCapability` (what the adaptive controller learned, for planners).

Clone this repository into your workspace `src` directory. Colcon discovers both packages automatically.

```sh
cd ~/ros2_ws
colcon build --symlink-install --packages-select ackermann_interfaces pico_ackermann_driver
source install/setup.bash
cd src/pico_ackermann_driver/pico_ackermann_driver
python3 -m pytest test
```

Firmware and udev instructions in the driver README use paths relative to the `pico_ackermann_driver` package directory above.

The interface package was imported with its original Git history; it no longer needs a separate checkout.
