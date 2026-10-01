# Copyright 2026 Lucas Foulkes
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE file or at https://opensource.org/licenses/MIT.

from pico_ackermann_driver.driver_node import channel_topic
from pico_ackermann_driver.driver_node import clamp_command


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
    events=[]
    n=NS(channels={'steering':None,'throttle':None},
         serial_port=NS(close=lambda:events.append('closed')),
         send=lambda line:events.append(line))
    PicoAckermannDriver.close(n)
    assert events==['steering stop\n','throttle stop\n','closed']
    assert n.serial_port is None


def test_disconnect_revokes_delivered_state_and_rearms_stop_messages():
    from types import SimpleNamespace as NS
    from pico_ackermann_driver.driver_node import PicoAckermannDriver
    n=NS(serial_port=NS(close=lambda:None),delivered={'throttle':(.3,True)},
         channels={'throttle':[0.,None,True]},get_logger=lambda:NS(warning=lambda *_:None))
    PicoAckermannDriver.disconnect(n,'test disconnect')
    assert n.serial_port is None
    assert n.delivered=={}
    assert not n.channels['throttle'][2]


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
