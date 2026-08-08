# Copyright 2026 Lucas Foulkes
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE file or at https://opensource.org/licenses/MIT.
"""MicroPython firmware for normalized Ackermann actuator commands."""

from math import isfinite
import select
import sys
from time import sleep_ms, ticks_diff, ticks_ms

from machine import Pin, PWM


WATCHDOG_MS = 500
MAX_CHARS_PER_PASS = 256

STEERING_PIN = 2
STEERING_FREQ_HZ = 50
STEERING_MIN_US = 1000
STEERING_CENTER_US = 1500
STEERING_MAX_US = 2000


class Servo:
    """Drive a calibrated hobby servo from normalized position commands."""

    def __init__(self, pin, min_us, center_us, max_us, freq):
        self.min_us = min_us
        self.center_us = center_us
        self.max_us = max_us
        self.pwm = PWM(Pin(pin), freq=freq, duty_u16=0)

    def set_position(self, value):
        """Set a finite normalized position command in [-1, 1]."""
        value = max(-1.0, min(1.0, value))
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


CHANNELS = {
    'steering': Servo(
        STEERING_PIN,
        STEERING_MIN_US,
        STEERING_CENTER_US,
        STEERING_MAX_US,
        STEERING_FREQ_HZ,
    ),
}

last_ok = {}
enabled = {}
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)
buffer = ''


try:
    while True:
        poller.poll(20)

        chars_read = 0
        while chars_read < MAX_CHARS_PER_PASS and poller.poll(0):
            char = sys.stdin.read(1)
            if not char:
                break
            chars_read += 1

            if char != '\n':
                buffer += char
                if len(buffer) > 64:
                    buffer = ''
                continue

            line, buffer = buffer.strip(), ''
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

            device.set_position(value)
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
    for device in CHANNELS.values():
        device.stop()
