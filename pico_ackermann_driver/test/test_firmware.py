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
            if pending:
                return [True]
            if timeout:
                raise KeyboardInterrupt
            return []

    def read(_):
        return bytes([pending.pop(0)]) if pending else b''

    def sleep_ms(ms):
        clock[0] += ms

    modules = {
        'machine': NS(Pin=Pin, PWM=PWM),
        'micropython': NS(kbd_intr=lambda char: events.append(('kbd_intr', char))),
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
    """Each pin's last level or duty after every event."""
    state, history = {}, []
    for kind, number, value in events:
        if kind != 'kbd_intr':
            state[number] = value
            history.append(dict(state))
    return history


def test_commands_are_clamped_to_the_installed_range(monkeypatch):
    events = run_firmware(monkeypatch, b'steering 1.5\nsteering -2\nthrottle 2\n')
    assert ('ns', STEERING, 2_000_000) in events
    assert ('ns', STEERING, 1_000_000) in events
    assert max(value for kind, number, value in events if kind == 'ns') == 2_000_000
    assert min(value for kind, number, value in events if kind == 'ns') == 1_000_000
    assert max(value for kind, number, value in events if kind == 'duty') == 65_535
