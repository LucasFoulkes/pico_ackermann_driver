# Copyright 2026 Lucas Foulkes
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE file or at https://opensource.org/licenses/MIT.

from pico_ackermann_driver.driver_node import channel_topic
from pico_ackermann_driver.driver_node import clamp_command


def test_clamp_command():
    assert clamp_command(0.25) == 0.25
    assert clamp_command(2.0) == 1.0
    assert clamp_command(-2.0) == -1.0


def test_channel_topic():
    assert channel_topic('steering') == '/actuators/steering/command'
