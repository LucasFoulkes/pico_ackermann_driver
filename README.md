# pico_ackermann_driver

ROS 2 Jazzy USB serial driver and MicroPython firmware for normalized
Ackermann actuator commands. The ROS node owns the serial port; the Pico
owns GPIO assignments, PWM timing, actuator limits, and the final watchdog.

## Interface

- Subscribes: `/actuators/steering/command` (`std_msgs/msg/Float32`)
- Input range: `[-1.0, 1.0]`
- Serial protocol: `steering <value>` or `steering stop`
- Pico steering output: GP2, 50 Hz servo pulses
- Servo calibration: Hitec HS-645MG, 900/1500/2100 microseconds

Commands must arrive continuously. After 250 ms without a ROS command, the
driver sends `steering stop`. After approximately 520 ms without a valid
serial command, the Pico independently stops servo pulses.

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select pico_ackermann_driver
source install/setup.bash
```

## Host setup

The user must belong to `dialout`. Install the included ModemManager rule:

```bash
sudo cp udev/99-pico-ignore-mm.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Deploy the Pico firmware

Stop the ROS driver before using `mpremote`; both require exclusive access
to the same USB serial device.

```bash
mpremote cp :main.py ~/pico-main-before-ackermann.py
mpremote cp firmware/main.py :main.py
mpremote reset
```

The first command backs up the Pico's existing LED-blink program.

## Run in its own terminal

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pico_ackermann_driver pico_ackermann_driver
```

Run `dualshock4_teleop` in a separate terminal. There is deliberately no
combined launch file.

## Servo power

Power the servo from a supply sized for its stall current, not the Pico 3V3
pin. Connect the servo supply ground and Pico ground. Begin powered tests
with the teleop mapper's `scale` set to `0.1`.

## Future traction motor

GP2 shares RP2040 PWM slice 1 with GP3. A future motor PWM output must use
another slice, such as GP4/GP5, because the servo uses 50 Hz while a motor
driver normally uses a kHz-range PWM frequency.
