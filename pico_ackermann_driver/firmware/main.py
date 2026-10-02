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
# with the linkage disconnected and flash again.
STEERING_MIN_US = 1000
STEERING_CENTER_US = 1500
STEERING_MAX_US = 2000

THROTTLE_RPWM_PIN = 4
THROTTLE_LPWM_PIN = 5
THROTTLE_EN_PIN = 6
THROTTLE_FREQ_HZ = 20_000
THROTTLE_INVERT = True


class Servo:
    """Drive a calibrated hobby servo from normalized position commands."""

    def __init__(self, pin, min_us, center_us, max_us, freq):
        self.min_us = min_us
        self.center_us = center_us
        self.max_us = max_us
        self.pwm = PWM(Pin(pin), freq=freq, duty_u16=0)

    def set_command(self, value):
        """Set a finite position command, clamped to min_us..max_us."""
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


class Motor:
    """Drive an HW-039/BTS7960 using signed normalized commands."""

    def __init__(self, rpwm_pin, lpwm_pin, en_pin, freq, invert=False):
        # The shared enable goes low before either PWM output is touched (after
        # a soft reset the pins hold what the previous run left on them), and
        # high only once both duties are 0: braked from boot, so a car reset by
        # a brown-out on a slope holds instead of rolling.
        self.en = Pin(en_pin, Pin.OUT, value=0)
        self.rpwm = PWM(Pin(rpwm_pin), freq=freq, duty_u16=0)
        self.lpwm = PWM(Pin(lpwm_pin), freq=freq, duty_u16=0)
        self.invert = invert
        self.stop()

    def set_command(self, value):
        """Set direction from the sign and duty from the magnitude."""
        value = max(-COMMAND_LIMIT, min(COMMAND_LIMIT, value))
        if self.invert:
            value = -value
        duty = int(abs(value) * 65_535)

        if value >= 0:
            self.lpwm.duty_u16(0)
            self.rpwm.duty_u16(duty)
        else:
            self.rpwm.duty_u16(0)
            self.lpwm.duty_u16(duty)
        self.en.value(1)

    def stop(self):
        """Brake: both half-bridges low with the bridge enabled.

        Disabling the bridge would let the motor freewheel, and a car parked
        on a hill would roll away whenever the Pi, USB or the driver drops.
        The bridge is disabled only from power-up until this firmware starts.
        """
        self.rpwm.duty_u16(0)
        self.lpwm.duty_u16(0)
        self.en.value(1)


CHANNELS = {
    'steering': Servo(
        STEERING_PIN,
        STEERING_MIN_US,
        STEERING_CENTER_US,
        STEERING_MAX_US,
        STEERING_FREQ_HZ,
    ),
    'throttle': Motor(
        THROTTLE_RPWM_PIN,
        THROTTLE_LPWM_PIN,
        THROTTLE_EN_PIN,
        THROTTLE_FREQ_HZ,
        THROTTLE_INVERT,
    ),
}

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

        sleep_ms(1)
finally:
    # Whatever ended the loop (Ctrl-C, an error): brake the motor first, then
    # end the servo pulses. From here on Ctrl-C is an ordinary byte, so a
    # second one cannot cut the stops short; the REPL enables it again for
    # whatever it runs next.
    micropython.kbd_intr(-1)
    CHANNELS['throttle'].stop()
    CHANNELS['steering'].stop()
