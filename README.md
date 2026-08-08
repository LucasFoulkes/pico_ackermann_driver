# pico_ackermann_driver

ROS 2 Jazzy USB serial driver and MicroPython firmware for normalized
Ackermann actuator commands. The ROS node owns the serial port; the Pico
owns GPIO assignments, PWM timing, actuator limits, and the final watchdog.

## Interface

- Subscribes: `/actuators/steering/command` and
  `/actuators/throttle/command` (`std_msgs/msg/Float32`)
- Input range: `[-1.0, 1.0]`
- Serial protocol: `<channel> <value>` or `<channel> stop`
- Pico steering output: GP2, 50 Hz servo pulses
- Servo calibration: Hitec HS-645MG, 900/1500/2100 microseconds
- Pico throttle outputs: RPWM GP4, LPWM GP5, enable GP6, 20 kHz
- Throttle bridge: HW-039/BTS7960; positive is forward, negative reverse

Commands must arrive continuously. After 250 ms without a ROS command, the
driver sends that channel's `stop`. After approximately 520 ms without a
valid serial command, the Pico independently disables the affected output.
Steering and throttle use independent watchdog timestamps.

## Unloaded servo calibration

Normal operation clamps commands to `[-1.0, 1.0]`. With the steering linkage
disconnected, the driver can explicitly enable calibration commands up to
`[-2.0, 2.0]`:

```bash
ros2 run pico_ackermann_driver pico_ackermann_driver \
  --ros-args -p command_limit:=2.0
```

The HS-645MG's specified range remains `900-2100 us` at commands `-1.0` and
`1.0`. Calibration values extrapolate that mapping, with an absolute firmware
guard of `500-2500 us`. Values outside `[-2.0, 2.0]` are rejected/clamped.
This mode is outside the servo's published specification and must never be
used with the steering linkage connected.

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

Run `dualshock4_teleop` in a separate terminal, or use the application-level
`robot` package to start the complete system.

## Servo power

Power the servo from a supply sized for its stall current, not the Pico 3V3
pin. Connect the servo supply ground and Pico ground. Begin powered tests
with the teleop mapper's `scale` set to `0.1`.

## Traction motor wiring

| HW-039 pin | Pico connection |
|---|---|
| RPWM | GP4 |
| LPWM | GP5 |
| R_EN and L_EN tied together | GP6 |
| VCC | Pico 3V3 |
| GND | Pico/battery common ground |
| R_IS, L_IS | Not connected |

Connect the motor battery only to `B+`/`B-` and the motor to `M+`/`M-`.
Never connect the traction battery to a Pico power pin. GP4/GP5 share PWM
slice 2 at 20 kHz; the steering servo remains on GP2/slice 1 at 50 Hz.

Firmware drives GP6 low before configuring the PWM outputs and on every stop.
Because RP2040 GPIO is high-impedance before MicroPython starts, fit an
external pull-down (for example 10 kOhm) from the tied enable input to ground
if motion during boot must be prevented electrically as well as in software.
