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
COMMAND_LIMIT = 2.0

STEERING_PIN = 2
STEERING_FREQ_HZ = 50
STEERING_MIN_US = 900
STEERING_CENTER_US = 1500
STEERING_MAX_US = 2100
STEERING_HARD_MIN_US = 500
STEERING_HARD_MAX_US = 2500

THROTTLE_RPWM_PIN = 4
THROTTLE_LPWM_PIN = 5
THROTTLE_EN_PIN = 6
THROTTLE_FREQ_HZ = 20_000


class Servo:
    """Drive a calibrated hobby servo from normalized position commands."""

    def __init__(self, pin, min_us, center_us, max_us, freq):
        self.min_us = min_us
        self.center_us = center_us
        self.max_us = max_us
        self.pwm = PWM(Pin(pin), freq=freq, duty_u16=0)

    def set_command(self, value):
        """Set a finite position command, including calibration extension."""
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
        pulse_us = max(
            STEERING_HARD_MIN_US,
            min(STEERING_HARD_MAX_US, pulse_us),
        )
        self.pwm.duty_ns(int(pulse_us * 1000))

    def stop(self):
        """Stop servo pulses by holding the PWM output low."""
        self.pwm.duty_u16(0)


class Motor:
    """Drive an HW-039/BTS7960 using signed normalized commands."""

    def __init__(self, rpwm_pin, lpwm_pin, en_pin, freq):
        # Force the shared enable low before configuring either PWM output.
        self.en = Pin(en_pin, Pin.OUT, value=0)
        self.rpwm = PWM(Pin(rpwm_pin), freq=freq, duty_u16=0)
        self.lpwm = PWM(Pin(lpwm_pin), freq=freq, duty_u16=0)

    def set_command(self, value):
        """Set direction from the sign and duty from the magnitude."""
        value = max(-1.0, min(1.0, value))
        duty = int(abs(value) * 65_535)

        if value >= 0:
            self.lpwm.duty_u16(0)
            self.rpwm.duty_u16(duty)
        else:
            self.rpwm.duty_u16(0)
            self.lpwm.duty_u16(duty)
        self.en.value(1)

    def stop(self):
        """Disable the bridge and clear both PWM outputs for coast."""
        self.en.value(0)
        self.rpwm.duty_u16(0)
        self.lpwm.duty_u16(0)


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
    for device in CHANNELS.values():
        device.stop()
