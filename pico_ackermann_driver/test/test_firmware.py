# Copyright 2026 Lucas Foulkes
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE file or at https://opensource.org/licenses/MIT.
"""Run firmware/main.py under CPython against fake machine, select and time modules.

The fake pins log every level and duty change. The fake stdin delivers the given
bytes, then raises KeyboardInterrupt (a Ctrl-C) in the main loop's wait.
"""

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

FIRMWARE = Path(__file__).resolve().parents[1] / 'firmware' / 'main.py'
STEERING, RPWM, LPWM, EN = 2, 4, 5, 6
LOOP = ('loop', None, None)  # logged when the main loop first waits for input
CTRL_C = ('ctrl-c', None, None)  # logged when the input runs out and Ctrl-C ends the loop


def run_firmware(monkeypatch, data=b''):
    events, pending, clock = [], bytearray(data), [0]

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
            if pending:
                return [True]
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
        exec(compile(FIRMWARE.read_text(), str(FIRMWARE), 'exec'), {'__name__': '__main__'})
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


def test_the_motor_brakes_from_boot_and_the_servo_waits(monkeypatch):
    events = run_firmware(monkeypatch)
    boot = events[:events.index(LOOP)]
    motor = [event for event in boot if event[1] in (RPWM, LPWM, EN)]
    assert motor[0] == ('pin', EN, 0)  # bridge off before either PWM output is touched
    for state in levels(boot):
        assert not (state.get(EN) == 1 and (state.get(RPWM) or state.get(LPWM)))
    assert levels(boot)[-1] == {STEERING: 0, RPWM: 0, LPWM: 0, EN: 1}


def test_commands_are_clamped_to_the_installed_range(monkeypatch):
    events = run_firmware(monkeypatch, b'steering 1.5\nsteering -2\nthrottle 2\n')
    assert ('ns', STEERING, 2_000_000) in events
    assert ('ns', STEERING, 1_000_000) in events
    assert max(value for kind, number, value in events if kind == 'ns') == 2_000_000
    assert min(value for kind, number, value in events if kind == 'ns') == 1_000_000
    assert max(value for kind, number, value in events if kind == 'duty') == 65_535


def test_ctrl_c_brakes_the_motor_first_and_cannot_cut_the_stops_short(monkeypatch):
    events = run_firmware(monkeypatch, b'steering 0.5\nthrottle 0.5\n')
    assert levels(events[:events.index(CTRL_C)])[-1][LPWM] > 0  # driving when Ctrl-C comes
    after = events[events.index(CTRL_C) + 1:]
    assert after[0] == ('kbd_intr', None, -1)
    motor_done = max(i for i, event in enumerate(after) if event[1] in (RPWM, LPWM, EN))
    assert all(event[1] != STEERING for event in after[:motor_done])
    assert levels(events)[-1] == {STEERING: 0, RPWM: 0, LPWM: 0, EN: 1}
