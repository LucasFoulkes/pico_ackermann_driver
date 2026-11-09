# Copyright 2026 Lucas Foulkes
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE file or at https://opensource.org/licenses/MIT.
"""Run firmware/main.py under CPython against fake machine, select and time modules.

The fake pins log every level and duty change. The fake stdin delivers the given
bytes, or each (ms, bytes) chunk once the fake clock reaches ms; after the last
chunk and idle_ms it raises KeyboardInterrupt (a Ctrl-C) in the main loop's wait.
"""

from pathlib import Path
import re
import sys
from types import ModuleType, SimpleNamespace as NS

FIRMWARE = Path(__file__).resolve().parents[1] / 'firmware' / 'main.py'
STEERING, THROTTLE = 2, 8
NEUTRAL = 1_500_000  # throttle pulse in ns
STOPPED = {STEERING: 0, THROTTLE: NEUTRAL}
LOOP = ('loop', None, None)  # logged when the main loop first waits for input
CTRL_C = ('ctrl-c', None, None)  # logged when the input runs out and Ctrl-C ends the loop


def run_firmware(monkeypatch, data=b'', idle_ms=0, invert=False):
    """The tests read throttle pulses uninverted (forward is above neutral); invert=None
    keeps the installed THROTTLE_INVERT."""
    chunks = sorted(data) if isinstance(data, list) else [(0, data)]
    events, pending, clock = [], bytearray(), [0]

    class Pin:
        OUT = 1

        def __init__(self, number, mode=None, value=None):
            self.number = number
            if value is not None:
                events.append(('pin', number, value))

        def value(self, level):
            events.append(('pin', self.number, level))

    class PWM:
        def __init__(self, pin, freq, duty_u16):
            self.number = pin.number
            events.append(('duty', self.number, duty_u16))

        def duty_u16(self, duty):
            events.append(('duty', self.number, duty))

        def duty_ns(self, ns):
            events.append(('ns', self.number, ns))

    class Poll:
        def register(self, *_):
            pass

        def poll(self, timeout):
            if LOOP not in events:
                events.append(LOOP)
            while chunks and chunks[0][0] <= clock[0]:
                pending.extend(chunks.pop(0)[1])
            if pending:
                return [True]
            if timeout and (chunks or clock[0] < idle_ms):
                clock[0] += timeout
                return []
            if timeout:
                events.append(CTRL_C)
                raise KeyboardInterrupt
            return []

    def read(_):
        return bytes([pending.pop(0)]) if pending else b''

    def sleep_ms(ms):
        clock[0] += ms

    modules = {
        'machine': NS(Pin=Pin, PWM=PWM),
        'micropython': NS(kbd_intr=lambda char: events.append(('kbd_intr', None, char))),
        'select': NS(poll=Poll, POLLIN=1),
        'time': NS(sleep_ms=sleep_ms, ticks_ms=lambda: clock[0], ticks_diff=lambda a, b: a - b),
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(vars(attributes))
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(sys, 'stdin', NS(buffer=NS(read=read)))
    try:
        source = FIRMWARE.read_text()
        if invert is not None:
            source = re.sub(r'^THROTTLE_INVERT = \w+', f'THROTTLE_INVERT = {invert}', source, flags=re.M)
        exec(compile(source, str(FIRMWARE), 'exec'), {'__name__': '__main__'})
    except KeyboardInterrupt:
        pass
    return events


def levels(events):
    """Each pin's level, duty or pulse width after every output change."""
    state, history = {}, []
    for kind, number, value in events:
        if kind in ('pin', 'duty', 'ns'):
            state[number] = value
            history.append(dict(state))
    return history


def throttle_ns(events):
    """Throttle pulse widths in order, up to the Ctrl-C if there is one."""
    if CTRL_C in events:
        events = events[:events.index(CTRL_C)]
    return [value for kind, number, value in events if kind == 'ns' and number == THROTTLE]


def every_100_ms(line, start, end):
    """Keep-alive commands within the watchdog, as the driver sends them."""
    return [(ms, line) for ms in range(start, end, 100)]


# One loop pass is at most about 21 fake ms; at 2 units/s and 500 us per unit
# a ramp step is then at most about 21 us.
MAX_RAMP_STEP_NS = 25_000


def test_the_esc_gets_neutral_from_boot_and_the_servo_waits(monkeypatch):
    events = run_firmware(monkeypatch)
    boot = events[:events.index(LOOP)]
    assert set(throttle_ns(boot)) == {NEUTRAL}  # the ESC arms on neutral, never sees throttle
    assert levels(boot)[-1] == STOPPED


def test_commands_are_clamped_to_the_installed_range(monkeypatch):
    events = run_firmware(monkeypatch, b'steering 1.5\nsteering -2\n')
    assert ('ns', STEERING, 2_000_000) in events
    assert ('ns', STEERING, 1_000_000) in events
    for command, full in ((b'throttle 2\n', 2_000_000), (b'throttle -2\n', 1_000_000)):
        pulses = throttle_ns(run_firmware(monkeypatch, every_100_ms(command, 0, 900)))
        assert full in pulses
        assert all(1_000_000 <= pulse <= 2_000_000 for pulse in pulses)


def test_positive_steering_is_the_long_pulse_left(monkeypatch):
    events = run_firmware(monkeypatch, b'steering 1\nsteering -0.5\n')
    pulses = [value for kind, number, value in events if kind == 'ns' and number == STEERING]
    assert pulses == [2_000_000, 1_250_000]


def test_throttle_ramps_up(monkeypatch):
    pulses = throttle_ns(run_firmware(monkeypatch, every_100_ms(b'throttle 1\n', 0, 300)))
    assert pulses == sorted(pulses)
    assert max(b - a for a, b in zip(pulses, pulses[1:])) <= MAX_RAMP_STEP_NS
    assert NEUTRAL < max(pulses) < 2_000_000  # 300 ms is not enough for full


def test_slowing_is_immediate_and_a_reversal_ramps_from_neutral(monkeypatch):
    slow = every_100_ms(b'throttle 1\n', 0, 400) + [(450, b'throttle 0.1\n')]
    pulses = throttle_ns(run_firmware(monkeypatch, slow))
    slowed = pulses.index(NEUTRAL + 50_000)
    assert pulses[slowed - 1] > NEUTRAL + 300_000 and set(pulses[slowed:]) == {NEUTRAL + 50_000}
    reverse = every_100_ms(b'throttle 1\n', 0, 400) + every_100_ms(b'throttle -1\n', 400, 700)
    pulses = throttle_ns(run_firmware(monkeypatch, reverse))
    first_back = next(i for i, pulse in enumerate(pulses) if pulse < NEUTRAL)
    assert pulses[first_back - 1] > NEUTRAL + 300_000
    assert pulses[first_back] >= NEUTRAL - MAX_RAMP_STEP_NS  # no jump into reverse
    assert min(pulses) < NEUTRAL - 100_000  # then it ramps on


def test_invert_mirrors_the_throttle_about_neutral(monkeypatch):
    forward = every_100_ms(b'throttle 1\n', 0, 800)
    plain = throttle_ns(run_firmware(monkeypatch, forward))
    mirrored = throttle_ns(run_firmware(monkeypatch, forward, invert=True))
    assert len(mirrored) == len(plain)  # 1 ns apart at most: int() truncates the float
    assert all(abs(m + p - 2 * NEUTRAL) <= 1 for m, p in zip(mirrored, plain))
    assert min(mirrored) == 1_000_000


def test_the_watchdog_returns_the_throttle_to_neutral(monkeypatch):
    pulses = throttle_ns(run_firmware(monkeypatch, b'throttle 0.2\n', idle_ms=1_000))
    assert max(pulses) == NEUTRAL + 100_000
    assert pulses[-1] == NEUTRAL


def test_ctrl_c_neutrals_the_throttle_first_and_cannot_cut_the_stops_short(monkeypatch):
    events = run_firmware(monkeypatch, b'steering 0.5\nthrottle 0.5\n', idle_ms=100)
    assert levels(events[:events.index(CTRL_C)])[-1][THROTTLE] > NEUTRAL  # driving when Ctrl-C comes
    after = events[events.index(CTRL_C) + 1:]
    assert after[0] == ('kbd_intr', None, -1)
    throttle_done = max(i for i, event in enumerate(after) if event[1] == THROTTLE)
    assert all(event[1] != STEERING for event in after[:throttle_done])
    assert levels(events)[-1] == STOPPED


def test_the_restart_line_never_completes_a_stale_partial_line(monkeypatch):
    from pico_ackermann_driver.driver_node import PICO_RESTART
    restart=PICO_RESTART.encode('ascii')
    for stale in (b'', b'throttle', b'throttle ', b'throttle 0.5', b'steering 1', b'steering 0.5 '):
        events=run_firmware(monkeypatch, stale+restart, idle_ms=100)
        assert not any(kind == 'ns' and number == STEERING for kind, number, _ in events)
        assert set(throttle_ns(events)) == {NEUTRAL}
    # The bare newline the driver used to send turned the stale line into a command.
    events=run_firmware(monkeypatch, b'throttle 0.5' + b'\n', idle_ms=100)
    assert max(throttle_ns(events)) > NEUTRAL
    # The command after the restart line is applied.
    events=run_firmware(monkeypatch, b'throttle 0.5' + restart + b'throttle 0.5\n', idle_ms=100)
    assert max(throttle_ns(events)) > NEUTRAL
