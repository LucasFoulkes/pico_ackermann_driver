# Copyright 2026 Lucas Foulkes
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE file or at https://opensource.org/licenses/MIT.
"""MicroPython firmware for normalized Ackermann actuator commands."""

from math import isfinite
import select
import sys
from time import sleep_ms, ticks_diff, ticks_ms

from machine import Pin, PWM
import micropython


WATCHDOG_MS = 500
MAX_CHARS_PER_PASS = 256
# Both channels clamp here, whatever the driver sends: -1/1 are full throttle
# and the ends of the installed steering linkage.
COMMAND_LIMIT = 1.0

STEERING_PIN = 2
STEERING_FREQ_HZ = 50
# Installed linkage travel at commands -1, 0 and 1. To recalibrate, edit these
# with the linkage disconnected and flash again. Command 1 is full left (ROS:
# positive steering turns left): this servo turns left on the long pulses. (A
# swap on 2026-10-14 made 1 turn right; 648 turning samples showed it, undone.)
STEERING_MIN_US = 1000
STEERING_CENTER_US = 1500
STEERING_MAX_US = 2000

# Brushed RC car ESC (the truck's own, set to F/R mode) on GP8, physical pin 11:
# 50 Hz servo pulses, neutral at the center. Replaced a HW-039/BTS7960 bridge on
# 2026-10-10 after one of its half-bridges shorted.
THROTTLE_PIN = 8
THROTTLE_FREQ_HZ = 50
THROTTLE_NEUTRAL_US = 1500
THROTTLE_SPAN_US = 500  # pulse at command 1 is NEUTRAL + SPAN, at -1 NEUTRAL - SPAN
THROTTLE_INVERT = True  # this ESC drives forward on the short pulses (2026-10-14)
# Speeding up and reversing are limited to this many command units per second,
# so 0 to full takes 0.5 s; slowing towards neutral is immediate.
THROTTLE_RAMP_PER_S = 2.0


class Servo:
    """Drive a calibrated hobby servo from normalized position commands."""

    def __init__(self, pin, min_us, center_us, max_us, freq):
        self.min_us = min_us
        self.center_us = center_us
        self.max_us = max_us
        self.pwm = PWM(Pin(pin), freq=freq, duty_u16=0)

    def set_command(self, value):
        """Set a finite position command, clamped to -1..1 (min_us..max_us)."""
        value = max(-COMMAND_LIMIT, min(COMMAND_LIMIT, value))
        if value >= 0:
            pulse_us = (
                self.center_us
                + value * (self.max_us - self.center_us)
            )
        else:
            pulse_us = (
                self.center_us
                + value * (self.center_us - self.min_us)
            )
        self.pwm.duty_ns(int(pulse_us * 1000))

    def stop(self):
        """Stop servo pulses by holding the PWM output low."""
        self.pwm.duty_u16(0)


class Esc:
    """Drive a brushed RC ESC in forward/reverse mode from ramped commands."""

    def __init__(self, pin, neutral_us, span_us, freq, ramp_per_s, invert=False):
        # Neutral pulses from boot: the ESC arms only after it sees neutral,
        # and a Pico reset must not leave it holding the last throttle.
        self.neutral_us = neutral_us
        self.span_us = span_us
        self.ramp_per_s = ramp_per_s
        self.invert = invert
        self.target = 0.0
        self.value = 0.0
        self.last_ms = ticks_ms()
        self.pwm = PWM(Pin(pin), freq=freq, duty_u16=0)
        self.write()

    def set_command(self, value):
        """Set the target the output ramps to, clamped to the command limit."""
        value = max(-COMMAND_LIMIT, min(COMMAND_LIMIT, value))
        self.target = -value if self.invert else value
        self.update(ticks_ms())

    def update(self, now):
        """Move the output towards the target; call it every loop pass."""
        elapsed = ticks_diff(now, self.last_ms)
        self.last_ms = now
        if self.target * self.value < 0:
            self.value = 0.0  # a reversal first drops to neutral
        if abs(self.target) <= abs(self.value):
            self.value = self.target
        else:
            step = self.ramp_per_s * elapsed / 1000
            self.value += max(-step, min(step, self.target - self.value))
        self.write()

    def write(self):
        self.pwm.duty_ns(int((self.neutral_us + self.value * self.span_us) * 1000))

    def stop(self):
        """Hold neutral at once; pulses continue so the ESC stays armed."""
        self.target = 0.0
        self.value = 0.0
        self.write()


CHANNELS = {
    'steering': Servo(
        STEERING_PIN,
        STEERING_MIN_US,
        STEERING_CENTER_US,
        STEERING_MAX_US,
        STEERING_FREQ_HZ,
    ),
    'throttle': Esc(
        THROTTLE_PIN,
        THROTTLE_NEUTRAL_US,
        THROTTLE_SPAN_US,
        THROTTLE_FREQ_HZ,
        THROTTLE_RAMP_PER_S,
        THROTTLE_INVERT,
    ),
}
throttle = CHANNELS['throttle']

last_ok = {}
enabled = {}
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)
# Bytes, not text: the text stream blocks waiting for the rest of a UTF-8
# sequence after a stray byte >= 0x80, and the watchdog below would not run.
stdin = sys.stdin.buffer
buffer = b''
overlong = False  # drop the rest of a line that was too long to be a command


try:
    while True:
        poller.poll(20)

        chars_read = 0
        while chars_read < MAX_CHARS_PER_PASS and poller.poll(0):
            char = stdin.read(1)
            if not char:
                break
            chars_read += 1

            if char != b'\n':
                if not overlong:
                    buffer += char
                    if len(buffer) > 64:
                        buffer, overlong = b'', True
                continue

            if overlong:
                buffer, overlong = b'', False
                continue
            try:
                line = buffer.decode().strip()
            except UnicodeError:
                line = ''
            buffer = b''
            parts = line.split()
            if len(parts) != 2 or parts[0] not in CHANNELS:
                continue

            name, argument = parts
            device = CHANNELS[name]
            if argument == 'stop':
                device.stop()
                enabled[name] = False
                continue

            try:
                value = float(argument)
            except ValueError:
                continue
            if not isfinite(value):
                continue

            device.set_command(value)
            enabled[name] = True
            last_ok[name] = ticks_ms()

        now = ticks_ms()
        for name, device in CHANNELS.items():
            if (
                enabled.get(name)
                and ticks_diff(now, last_ok[name]) >= WATCHDOG_MS
            ):
                device.stop()
                enabled[name] = False
        throttle.update(now)

        sleep_ms(1)
finally:
    # Whatever ended the loop (Ctrl-C, an error): throttle to neutral first,
    # then end the servo pulses. The PWM hardware keeps sending neutral after
    # main.py ends, so the ESC stays armed and stopped. From here on Ctrl-C is
    # an ordinary byte, so a second one cannot cut the stops short; the REPL
    # enables it again for whatever it runs next.
    micropython.kbd_intr(-1)
    CHANNELS['throttle'].stop()
    CHANNELS['steering'].stop()
