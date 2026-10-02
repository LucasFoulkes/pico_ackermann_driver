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
- Servo calibration: installed linkage, 1000/1500/2000 microseconds
- Pico throttle outputs: RPWM GP4, LPWM GP5, enable GP6, 20 kHz
- Throttle bridge: HW-039/BTS7960; positive commands drive the installed
  motor forward through the firmware's explicit polarity setting

Each command is written to the Pico as it arrives and reported on `/actuators/state`;
the driver resends the latest one at `send_hz` (10 Hz) to keep the Pico's watchdog fed.
Commands must arrive continuously. After 250 ms without a ROS command, the
driver sends that channel's `stop`. After approximately 520 ms without a
valid serial command, the Pico independently stops the affected output.
Steering and throttle use independent watchdog timestamps. A throttle `stop`
brakes (bridge enabled, both sides low) so the car holds on a slope; a
steering `stop` ends the servo pulses.

## Command limits and servo calibration

The firmware clamps both channels to `[-1.0, 1.0]`, whatever it receives. The
installed steering linkage uses `1000-2000 us` at commands `-1.0` and `1.0`
(`STEERING_MIN_US`, `STEERING_CENTER_US`, `STEERING_MAX_US` in
`firmware/main.py`); to recalibrate, change those with the linkage
disconnected and flash the firmware again. The driver's `command_limit`
(default `1.0`, at most `1.0`) can only narrow the range further.

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
to the same USB serial device. `mpremote` commands such as `cp` stop
`main.py` and leave the Pico at the REPL; `mpremote reset` starts it again,
and so does the driver when it connects (see Delivery status).

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
| VCC | Pico VBUS 5 V (physical pin 40) |
| GND | Pico/battery common ground |
| R_IS, L_IS | Not connected |

Connect the motor battery only to `B+`/`B-` and the motor to `M+`/`M-`.
Never connect the traction battery to a Pico power pin. GP4/GP5 share PWM
slice 2 at 20 kHz; the steering servo remains on GP2/slice 1 at 50 Hz.
The module logic supply is 5 V, while its RPWM, LPWM, and enable inputs accept
the Pico's 3.3 V GPIO levels. Never connect VBUS 5 V to a Pico GPIO.

At start the firmware drives GP6 low, sets both PWM outputs to 0 and only
then drives GP6 high: the motor brakes from boot, so a Pico reset (for
example a brown-out) does not let the car roll on a slope. The steering
servo gets no pulses until the first steering command.

Between power-up and the start of `main.py` the bridge is disabled
(the motor freewheels): RP2040 pads come out of reset as inputs with weak
internal pull-downs (roughly 50 kOhm), which hold the enable low. Fit an
external pull-down (for example 10 kOhm) from the tied enable input to
ground if that low must not depend on the internal pull-down.

## Delivery status

`/actuators/state` (`ackermann_interfaces/msg/ActuatorState`) reports normalized
commands successfully written to the serial link, connection status, channel
activation and a ROS timestamp. This lets the adaptive learner align inputs to
odometry and suspend updates on a disconnect. It is not measured wheel speed,
steering angle or a firmware acknowledgment. A reconnect clears stop bookkeeping
so the driver sends explicit stops for inactive channels again.

`connected` means the port is open and the Pico runs its firmware. The
firmware never writes to the port, so the driver reads it every tick: any
byte comes from MicroPython's REPL (a traceback, a banner, an echo), which
means something stopped `main.py` (typically a Ctrl-C from `mpremote`) and
no command is applied. The driver then reports `connected: false`, logs
what it received and sends Ctrl-B, CR, Ctrl-D (leave the raw REPL, end the
typed line, soft reset, which runs `main.py` again), at most every 2 s. It
sends the same line on every connect, since a Pico in the raw REPL echoes
nothing; a running `main.py` rejects it as one invalid line. The Pico counts
as running again once it stayed silent for 1 s after such a restart.

The driver stops both channels when it detects multiple publishers on an
actuator command topic (checked once a second). It resumes only after ownership is
unambiguous and fresh commands arrive. Raw manual and autonomous launches are
mutually exclusive; this is an ownership guard, not a velocity multiplexer.
