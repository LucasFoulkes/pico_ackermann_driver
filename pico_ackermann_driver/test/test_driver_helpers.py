# Copyright 2026 Lucas Foulkes
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE file or at https://opensource.org/licenses/MIT.

from pico_ackermann_driver.driver_node import channel_topic
from pico_ackermann_driver.driver_node import clamp_command


class FakePort:
    """A serial port that records what the driver does with it."""

    def __init__(self, incoming=b''):
        self.incoming=bytearray(incoming)
        self.written=[]
        self.events=[]
        self.out_waiting=0

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, size):
        data=bytes(self.incoming[:size])
        del self.incoming[:size]
        return data

    def write(self, data):
        self.written.append(data)
        return len(data)

    def reset_input_buffer(self):
        self.events.append('reset_input')
        self.incoming.clear()

    def reset_output_buffer(self):
        self.events.append('reset_output')

    def close(self):
        self.events.append('close')


def make_driver(**attributes):
    """A driver with its real methods but no ROS node behind it."""
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    node=PicoAckermannDriver.__new__(PicoAckermannDriver)
    log, published = [], []
    logger=NS(info=log.append,warning=log.append)
    vars(node).update(
        port='/dev/pico',baud=115200,serial_port=None,connect_error=None,delivered={},
        channels={'steering':[0.,None,None],'throttle':[0.,None,None]},command_conflict=False,
        command_limit=1.,input_timeout=.25,pico_running=False,pico_restart_at=0.,
        pico_answered=False,pico_output=b'',log=log,published=published,
        get_logger=lambda:logger,check_owners=lambda:None,
        get_clock=lambda:NS(now=lambda:NS(to_msg=Time)),state_pub=NS(publish=published.append))
    vars(node).update(attributes)
    return node


def use_clock(monkeypatch, start):
    """Replace the driver's monotonic clock with one the test sets (clock.t, seconds)."""
    from types import SimpleNamespace as NS
    from pico_ackermann_driver import driver_node
    clock=NS(t=start)
    monkeypatch.setattr(driver_node,'time',NS(
        monotonic=lambda:clock.t,monotonic_ns=lambda:round(clock.t*1e9),sleep=lambda _:None))
    return clock


def test_clamp_command():
    assert clamp_command(0.25) == 0.25
    assert clamp_command(2.0) == 1.0
    assert clamp_command(-2.0) == -1.0
    assert clamp_command(1.5, 2.0) == 1.5
    assert clamp_command(3.0, 2.0) == 2.0


def test_channel_topic():
    assert channel_topic('steering') == '/actuators/steering/command'
    assert channel_topic('throttle') == '/actuators/throttle/command'


def test_invalid_input_stops_the_channel_on_the_pico_at_once(monkeypatch):
    from types import SimpleNamespace as NS
    use_clock(monkeypatch,100.)
    for value in (float('nan'),float('inf'),-float('inf')):
        port=FakePort()
        n=make_driver(serial_port=port,pico_running=True)
        n.channels['throttle']=[.5,123,None]
        n.delivered['throttle']=(.5,True)
        n.on_command('throttle',NS(data=value))
        assert port.written==[b'throttle stop\n']
        assert n.channels['throttle']==[0.,None,100_000_000_000]
        assert not n.published[-1].throttle_active and n.published[-1].throttle==0.


def test_an_idle_channel_is_stopped_again_every_second(monkeypatch):
    # A Pico that reset or was unplugged while idle is noticed, and a rebooted one is told again.
    clock=use_clock(monkeypatch,100.)
    port=FakePort()
    n=make_driver(serial_port=port,pico_running=True,watch_pico=lambda:None,
                  channels={'throttle':[.3,round(99.5e9),None]})
    n.tick()
    assert port.written==[b'throttle stop\n']
    clock.t=100.9
    n.tick()
    assert len(port.written)==1
    clock.t=101.
    n.tick()
    assert port.written==[b'throttle stop\n']*2


def test_clean_shutdown_sends_channel_stops_before_closing():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    port=FakePort()
    n=NS(channels={'steering':None,'throttle':None},serial_port=port,
         send=lambda line:port.events.append(line))
    PicoAckermannDriver.close(n)
    assert port.events==['steering stop\n','throttle stop\n','reset_output','close']
    assert n.serial_port is None


def test_shutdown_waits_briefly_for_the_stops_then_discards_them():
    # A stalled Pico takes nothing; close() alone would block up to 30 s (closing_wait).
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    import time
    port=FakePort()
    port.out_waiting=17
    n=NS(channels={'throttle':None},serial_port=port,send=lambda line:True)
    start=time.monotonic()
    PicoAckermannDriver.close(n)
    assert time.monotonic()-start<1.
    assert port.events==['reset_output','close']


def test_disconnect_revokes_delivered_state_and_rearms_stop_messages():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    port=FakePort()
    n=NS(serial_port=port,delivered={'throttle':(.3,True)},
         channels={'throttle':[0.,None,True]},get_logger=lambda:NS(warning=lambda *_:None))
    PicoAckermannDriver.disconnect(n,'test disconnect')
    assert n.serial_port is None
    assert port.events==['reset_output','close']  # queued output discarded: close() cannot block
    assert n.delivered=={}
    assert not n.channels['throttle'][2]


def test_a_failed_write_on_connect_releases_the_port(monkeypatch):
    from pico_ackermann_driver import driver_node
    import serial
    port=FakePort()
    def stalled(_):
        raise serial.SerialTimeoutException('Write timeout')
    port.write=stalled
    monkeypatch.setattr(driver_node.serial,'Serial',lambda *args,**kwargs:port)
    n=make_driver()
    n.connect()
    assert n.serial_port is None
    assert port.events==['reset_output','close']


def test_failed_write_is_never_reported_as_delivered_command():
    import time
    n=make_driver(serial_port=object(),watch_pico=lambda:None,send=lambda _:False,
                  channels={'throttle':[.4,time.monotonic_ns(),None]})
    n.tick()
    assert n.delivered=={}


def test_conflicting_publishers_revoke_both_actuator_channels():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    n=NS(owner_check_at=0.,command_conflict=False,
         channels={'steering':[.4,1,None],'throttle':[.3,1,None]},
         get_publishers_info_by_topic=lambda _:[object(),object()],
         get_logger=lambda:NS(warning=lambda *_:None))
    PicoAckermannDriver.check_owners(n)
    assert n.command_conflict
    assert all(state==[0.,None,None] for state in n.channels.values())


def test_a_command_goes_to_the_pico_at_once_and_is_reported_delivered():
    from types import SimpleNamespace as NS
    port=FakePort()
    n=make_driver(serial_port=port,pico_running=True)
    n.on_command('throttle',NS(data=.3))
    assert port.written==[b'throttle 0.30000\n']
    assert n.delivered['throttle']==(.3,True)
    assert n.published[-1].throttle_active and n.published[-1].throttle==.3


def test_the_value_reported_is_the_one_sent():
    from types import SimpleNamespace as NS
    port=FakePort()
    n=make_driver(serial_port=port,pico_running=True,command_limit=.8)
    n.on_command('steering',NS(data=-.123456789))
    n.on_command('throttle',NS(data=3.))
    assert port.written==[b'steering -0.12346\n',b'throttle 0.80000\n']
    assert n.delivered=={'steering':(-.12346,True),'throttle':(.8,True)}


def test_no_command_goes_out_while_ownership_is_in_conflict():
    from types import SimpleNamespace as NS
    port=FakePort()
    n=make_driver(serial_port=port,command_conflict=True)
    n.on_command('throttle',NS(data=.3))
    assert port.written==[] and n.delivered=={}


def test_a_failed_write_on_arrival_is_not_reported_delivered():
    from types import SimpleNamespace as NS
    n=make_driver(serial_port=object(),send=lambda _:False)
    n.on_command('throttle',NS(data=.3))
    assert n.delivered=={}


def test_repl_output_revokes_delivery_and_restarts_the_pico(monkeypatch):
    # The firmware never writes: anything received means main.py stopped, here with a traceback.
    use_clock(monkeypatch,100.)
    port=FakePort(b'Traceback (most recent call last):\r\nKeyboardInterrupt: \r\n>>> ')
    n=make_driver(serial_port=port,pico_running=True,pico_restart_at=50.)
    n.delivered['throttle']=(.4,True)
    n.watch_pico()
    assert not n.pico_running
    assert port.written==[b'\x02\r\x04\n']  # leave raw REPL, end the line, soft reset: main.py again
    assert port.events==['reset_input']
    assert any('KeyboardInterrupt' in line for line in n.log)
    n.publish_state()
    assert not n.published[-1].connected
    assert not n.published[-1].throttle_active and n.published[-1].throttle==0.


def test_restarts_are_rate_limited_and_a_quiet_pico_counts_as_running_again(monkeypatch):
    clock=use_clock(monkeypatch,100.)
    port=FakePort(b'>>> ')
    n=make_driver(serial_port=port,pico_restart_at=50.)
    n.watch_pico()
    assert len(port.written)==1
    port.incoming+=b'MPY: soft reboot\r\n'  # its answer to the restart
    clock.t=100.1
    n.watch_pico()
    assert len(port.written)==1 and not n.pico_running  # no second restart within 2 s
    clock.t=102.
    n.watch_pico()
    assert len(port.written)==2  # a running main.py rejects the line; nothing comes back
    clock.t=102.9
    n.watch_pico()
    assert not n.pico_running
    clock.t=103.
    n.watch_pico()
    assert n.pico_running
    n.delivered['throttle']=(.4,True)
    n.publish_state()
    assert n.published[-1].connected and n.published[-1].throttle_active


def test_connect_restarts_the_pico_and_waits_for_a_quiet_second(monkeypatch):
    # A Pico in the raw REPL echoes nothing, so connecting sends the restart unasked; it also
    # ends a partial line left by a previous driver without completing it (see test_firmware).
    from pico_ackermann_driver import driver_node
    clock=use_clock(monkeypatch,100.)
    port=FakePort()
    monkeypatch.setattr(driver_node.serial,'Serial',lambda *args,**kwargs:port)
    n=make_driver()
    n.connect()
    assert n.serial_port is port and port.written==[b'\x02\r\x04\n']
    n.watch_pico()
    assert not n.pico_running
    clock.t=101.
    n.watch_pico()
    assert n.pico_running
