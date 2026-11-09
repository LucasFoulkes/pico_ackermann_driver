# pico_ackermann_driver

ROS 2 Jazzy USB serial driver and MicroPython firmware for normalized
Ackermann actuator commands. The ROS node owns the serial port; the Pico
owns GPIO assignments, PWM timing, actuator limits, and the final watchdog.

## Interface

- Subscribes: `/actuators/steering/command` and
  `/actuators/throttle/command` (`std_msgs/msg/Float32`)
- Publishes: `/actuators/state` (`ackermann_interfaces/msg/ActuatorState`,
  see Delivery status)
- Input range: `[-1.0, 1.0]`
- Serial protocol: `<channel> <value>` or `<channel> stop`
- Pico steering output: GP2, 50 Hz servo pulses
- Servo calibration: installed linkage, 1000/1500/2000 microseconds at
  commands -1/0/1 (positive turns left)
- Pico throttle output: GP8, 50 Hz pulses to the truck's brushed ESC,
  1500 us neutral, 2000-1000 us at commands -1 and 1 (inverted: this ESC
  drives forward on the short pulses)
- Throttle ramp: speeding up and reversing at most 2 units/s (0 to full in
  0.5 s); slowing towards neutral is immediate

Each command is written to the Pico as it arrives and reported on `/actuators/state`;
the driver resends the latest one at `send_hz` (10 Hz) to keep the Pico's watchdog fed.
Commands must arrive continuously. After 250 ms without a ROS command, the
driver sends that channel's `stop`, and repeats it every second while the
channel stays idle; a non-finite command (NaN, inf) sends it at once. The
value reported delivered is the one written, after `command_limit` and
rounding to 5 decimals. After approximately 520 ms without a
valid serial command, the Pico independently stops the affected output.
Steering and throttle use independent watchdog timestamps. A throttle `stop`
holds neutral at once (pulses continue, so the ESC stays armed; whether the
car holds on a slope depends on the ESC's drag brake); a steering `stop` ends
the servo pulses.

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
colcon build --symlink-install --packages-select ackermann_interfaces pico_ackermann_driver
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

Parameters (defaults; override with `--ros-args -p name:=value`):

| Parameter | Default | Meaning |
|---|---|---|
| `channels` | `[steering, throttle]` | Channels to drive; each subscribes to `/actuators/<channel>/command` |
| `port` | `/dev/pico-ackermann` | Serial device (alias from the udev rule) |
| `baud` | `115200` | Serial baud rate (USB CDC ignores it) |
| `send_hz` | `10.0` | Keep-alive, port read and `/actuators/state` rate |
| `command_limit` | `1.0` | Clamp on commands, in `(0, 1]` |
| `input_timeout` | `0.25` | Seconds without a ROS command before a channel is stopped |

On the robot the complete system is started by the `lukuma` package's
`drive.launch.py`, which passes its own parameter file.

## Servo power

Power the servo from a supply sized for its stall current, not the Pico 3V3
pin. Connect the servo supply ground and Pico ground. Begin powered tests
with small commands, for example
`ros2 topic pub -r 10 /actuators/steering/command std_msgs/msg/Float32 "{data: 0.1}"`
with only the driver running.

## Traction motor wiring

The motor runs from the truck's own brushed ESC, set to forward/reverse (F/R)
mode so a negative command reverses without the brake, neutral, reverse
sequence. It replaced a HW-039/BTS7960 bridge on 2026-10-10 after one of that
module's half-bridges shorted.

| ESC lead | Connection |
|---|---|
| Signal (white/yellow) | GP8 (physical pin 11) |
| Ground (black/brown) | Pico ground |
| Red (ESC BEC output) | Not connected to the Pico |

Connect the battery and motor to the ESC as in the truck. Never connect the
traction battery or the ESC's red lead to a Pico pin. GP8 (slice 4) and the
steering servo on GP2 (slice 1) both run at 50 Hz.

From boot the firmware sends neutral, which the ESC needs before it arms; it
never sees throttle until a command arrives. Before `main.py` starts the ESC
gets no pulses and stays disarmed or in its own failsafe. Flip
`THROTTLE_INVERT` in `firmware/main.py` (now `True`) if a positive command
drives the car backwards.

The firmware raises the throttle by at most 2 units per second (lowering is
immediate), so the motor can lag a rising command by up to 0.5 s.

## Delivery status

`/actuators/state` (`ackermann_interfaces/msg/ActuatorState`) reports normalized
commands successfully written to the serial link, connection status, channel
activation and a ROS timestamp. This lets the adaptive learner align inputs to
odometry and suspend updates on a disconnect. It is the command as written,
before the firmware's throttle ramp. It is not measured wheel speed,
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
