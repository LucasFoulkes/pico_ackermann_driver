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
