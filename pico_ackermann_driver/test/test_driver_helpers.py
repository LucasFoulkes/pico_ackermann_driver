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


def test_clamp_command():
    assert clamp_command(0.25) == 0.25
    assert clamp_command(2.0) == 1.0
    assert clamp_command(-2.0) == -1.0
    assert clamp_command(1.5, 2.0) == 1.5
    assert clamp_command(3.0, 2.0) == 2.0


def test_channel_topic():
    assert channel_topic('steering') == '/actuators/steering/command'
    assert channel_topic('throttle') == '/actuators/throttle/command'


def test_invalid_input_revokes_the_previous_drive_command():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    for value in (float('nan'),float('inf'),-float('inf')):
        n=NS(channels={'throttle':[.5,123,False]},command_limit=1.)
        PicoAckermannDriver.on_command(n,'throttle',NS(data=value))
        assert n.channels['throttle']==[0.,None,False]


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
    from types import SimpleNamespace as NS
    from pico_ackermann_driver import driver_node
    import serial
    port=FakePort()
    def stalled(_):
        raise serial.SerialTimeoutException('Write timeout')
    port.write=stalled
    monkeypatch.setattr(driver_node.serial,'Serial',lambda *args,**kwargs:port)
    logger=NS(info=lambda *_:None,warning=lambda *_:None)
    n=NS(port='/dev/pico',baud=115200,serial_port=None,connect_error=None,delivered={},channels={},
         get_logger=lambda:logger)
    n.send=lambda line:driver_node.PicoAckermannDriver.send(n,line)
    n.disconnect=lambda reason:driver_node.PicoAckermannDriver.disconnect(n,reason)
    driver_node.PicoAckermannDriver.connect(n)
    assert n.serial_port is None
    assert port.events==['reset_output','close']


def test_failed_write_is_never_reported_as_delivered_command():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    import time
    n=NS(serial_port=object(),input_timeout=.25,command_conflict=False,check_owners=lambda:None,
         channels={'throttle':[.4,time.monotonic_ns(),False]},delivered={},
         send=lambda _:False,publish_state=lambda:None)
    PicoAckermannDriver.tick(n)
    assert n.delivered=={}


def test_conflicting_publishers_revoke_both_actuator_channels():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    n=NS(owner_check_at=0.,command_conflict=False,
         channels={'steering':[.4,1,False],'throttle':[.3,1,False]},
         get_publishers_info_by_topic=lambda _:[object(),object()],
         get_logger=lambda:NS(warning=lambda *_:None))
    PicoAckermannDriver.check_owners(n)
    assert n.command_conflict
    assert all(state==[0.,None,False] for state in n.channels.values())


def test_a_command_goes_to_the_pico_at_once_and_is_reported_delivered():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    sent, reported = [], []
    n=NS(channels={'throttle':[0.,None,False]},command_limit=1.,command_conflict=False,delivered={},
         send=lambda line:sent.append(line) or True,publish_state=lambda:reported.append(True))
    PicoAckermannDriver.on_command(n,'throttle',NS(data=.3))
    assert sent==['throttle 0.30000\n']
    assert n.delivered['throttle']==(.3,True) and reported


def test_no_command_goes_out_while_ownership_is_in_conflict():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    sent=[]
    n=NS(channels={'throttle':[0.,None,False]},command_limit=1.,command_conflict=True,delivered={},
         send=lambda line:sent.append(line) or True,publish_state=lambda:None)
    PicoAckermannDriver.on_command(n,'throttle',NS(data=.3))
    assert sent==[] and n.delivered=={}


def test_a_failed_write_on_arrival_is_not_reported_delivered():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    n=NS(channels={'throttle':[0.,None,False]},command_limit=1.,command_conflict=False,delivered={},
         send=lambda _:False,publish_state=lambda:None)
    PicoAckermannDriver.on_command(n,'throttle',NS(data=.3))
    assert n.delivered=={}
