#!/usr/bin/env python3
"""Bridge normalized ROS actuator commands to a Pico over USB serial."""

import math
import re
import time

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32
from ackermann_interfaces.msg import ActuatorState


CHANNEL_PATTERN = re.compile(r'^[A-Za-z0-9_]+$')
TOPIC_PREFIX = '/actuators'
# The firmware never writes to the port, so whatever the Pico sends comes from
# MicroPython's REPL (a traceback, a banner, an echo): something stopped
# main.py, typically a Ctrl-C byte from mpremote, and no command is applied.
# This restarts it: Ctrl-B leaves the raw REPL (mpremote's), CR ends whatever
# was typed into the friendly REPL, and Ctrl-D on the then empty line
# soft-resets, which runs main.py again. A running main.py reads all of it as
# one line it rejects (the CR splits it into at least two words, none of them a
# channel followed by a value), so it is also safe to send when nothing is
# wrong, and it ends a partial line a previous driver left without completing
# it into a command.
PICO_RESTART = '\x02\r\x04\n'
PICO_RESTART_PERIOD = 2.0  # s between restart attempts
PICO_QUIET = 1.0  # s of silence after a restart before the Pico counts as running main.py
PICO_OUTPUT_LOGGED = 200  # last bytes the Pico sent that a warning shows
# An idle channel's stop is repeated this often, so an unplugged Pico (the
# write fails) or one at the REPL (it answers) is noticed while idle too.
STOP_REPEAT_NS = 1_000_000_000


def clamp_command(value, limit=1.0):
    """Clamp a finite command to a configured symmetric limit."""
    return max(-limit, min(limit, value))


def channel_topic(channel):
    """Return the ROS command topic for a protocol channel."""
    return f'{TOPIC_PREFIX}/{channel}/command'


def release_port(port, drain_s=0.0):
    """Close a serial port without waiting on output a stalled Pico never takes.

    Linux close() waits up to 30 s (closing_wait) for queued output to go out,
    so wait at most drain_s for it, then discard the rest before closing.
    """
    deadline = time.monotonic() + drain_s
    try:
        while port.out_waiting and time.monotonic() < deadline:
            time.sleep(0.01)
    except (serial.SerialException, OSError):
        pass
    for step in (port.reset_output_buffer, port.close):
        try:
            step()
        except (serial.SerialException, OSError):
            pass


class PicoAckermannDriver(Node):
    """Own the Pico serial port and enforce per-channel input timeouts."""

    def __init__(self):
        super().__init__('pico_ackermann_driver')

        channels = list(
            self.declare_parameter('channels', ['steering', 'throttle']).value)
        self.port = self.declare_parameter(
            'port', '/dev/pico-ackermann').value
        self.baud = self.declare_parameter('baud', 115200).value
        # Commands go to the Pico as they arrive; this rate only resends them (the Pico stops a
        # channel after ~0.5 s without one) and stops stale channels. At 50 Hz the wake-ups of this
        # Python node, not the serial writes, were its CPU (2026-09-30).
        send_hz = self.declare_parameter('send_hz', 10.0).value
        self.command_limit = self.declare_parameter(
            'command_limit', 1.0).value
        self.input_timeout = self.declare_parameter(
            'input_timeout', 0.25).value

        if not channels:
            raise ValueError('channels must not be empty')
        if any(not CHANNEL_PATTERN.fullmatch(name) for name in channels):
            raise ValueError('channel names must be alphanumeric/underscore')
        if send_hz <= 0:
            raise ValueError('send_hz must be greater than zero')
        if not 0 < self.command_limit <= 1.0:  # the firmware clamps to 1 as well
            raise ValueError('command_limit must be in (0, 1]')
        if self.input_timeout <= 0:
            raise ValueError('input_timeout must be greater than zero')
        if self.baud <= 0:
            raise ValueError('baud must be greater than zero')

        self.serial_port = None
        self.delivered = {}
        self.command_conflict = False
        self.connect_error = None  # last reason the port would not open, logged once
        self.owner_check_at = 0.0
        self.pico_running = False  # silent since the last restart: main.py applies the commands
        self.pico_restart_at = 0.0  # monotonic time PICO_RESTART was last sent
        self.pico_answered = False  # the Pico sent something since then
        self.pico_output = b''  # what it sent, for the next warning
        self.state_pub = self.create_publisher(ActuatorState, '/actuators/state', 5)
        # name -> [command, its arrival (monotonic ns), when its stop was last sent (monotonic ns)]
        self.channels = {
            name: [0.0, None, None]
            for name in channels
        }
        self._subscriptions = [
            self.create_subscription(
                Float32,
                channel_topic(name),
                lambda message, name=name: self.on_command(name, message),
                1,
            )
            for name in channels
        ]
        self.timer = self.create_timer(1.0 / send_hz, self.tick)

        self.get_logger().info(
            f'channels={channels} port={self.port} '
            f'command_limit={self.command_limit}')

    def on_command(self, channel, message):
        """Send a valid command at once; stop the channel at once on an invalid one."""
        state = self.channels[channel]
        now = time.monotonic_ns()
        if not math.isfinite(message.data):
            # Invalid input revokes the previous command, on the Pico too.
            state[:] = [0.0, None, None]
            if self.send_stop(channel, now):
                self.publish_state()
            return
        state[:] = [clamp_command(message.data, self.command_limit), now, None]
        # Out now, not at the next keep-alive tick, and reported as delivered at once.
        if not self.command_conflict and self.send_command(channel, state[0]):
            self.publish_state()

    def send_command(self, channel, value):
        """Write a command; on success record the value the Pico parses as delivered."""
        text = f'{value:.5f}'
        if not self.send(f'{channel} {text}\n'):
            return False
        self.delivered[channel] = (float(text), True)
        return True

    def send_stop(self, channel, now):
        """Write a channel's stop; on success record it and when it was sent."""
        if not self.send(f'{channel} stop\n'):
            return False
        self.channels[channel][2] = now
        self.delivered[channel] = (0.0, False)
        return True

    def tick(self):
        """Refresh fresh channels; stop the others, repeating it for a Pico that reset."""
        self.check_owners()
        if self.serial_port is None:
            self.connect()
        else:
            self.watch_pico()

        now = time.monotonic_ns()
        timeout_ns = int(self.input_timeout * 1_000_000_000)
        for channel, (value, updated_ns, stopped_ns) in self.channels.items():
            fresh = (
                not self.command_conflict
                and updated_ns is not None
                and now - updated_ns < timeout_ns
            )
            if fresh:
                self.send_command(channel, value)
            elif stopped_ns is None or now - stopped_ns >= STOP_REPEAT_NS:
                self.send_stop(channel, now)
        self.publish_state()

    def check_owners(self):
        now = time.monotonic()
        if now-self.owner_check_at < 1.0:   # a graph query: costly in Python, so once a second
            return
        self.owner_check_at = now
        conflict = any(len(self.get_publishers_info_by_topic(channel_topic(channel))) > 1
                       for channel in self.channels)
        if conflict != self.command_conflict:
            self.get_logger().warning('multiple actuator publishers: stopping' if conflict else 'actuator ownership restored; waiting for fresh commands')
            for state in self.channels.values():
                state[:] = [0.0, None, None]
        self.command_conflict = conflict

    def watch_pico(self):
        """Read what the Pico sent; restart it if it is at the REPL."""
        now = time.monotonic()
        try:
            received = self.serial_port.read(self.serial_port.in_waiting)
        except (serial.SerialException, OSError) as error:
            self.disconnect(f'lost {self.port} ({type(error).__name__}: {error})')
            return
        if received:
            self.pico_answered = True
            self.pico_output = (self.pico_output + received)[-PICO_OUTPUT_LOGGED:]
        if self.pico_answered and now - self.pico_restart_at >= PICO_RESTART_PERIOD:
            self.get_logger().warning(
                f'the Pico is not running its firmware, it sent {self.pico_output!r}; '
                'soft-resetting it')
            self.restart_pico(now)
        running = (self.serial_port is not None and not self.pico_answered
                   and now - self.pico_restart_at >= PICO_QUIET)
        if running and not self.pico_running:
            self.get_logger().info('the Pico runs its firmware')
        self.pico_running = running

    def restart_pico(self, now):
        """Send PICO_RESTART and drop what the Pico sent before it."""
        self.pico_running = False
        self.pico_restart_at, self.pico_answered, self.pico_output = now, False, b''
        if not self.send(PICO_RESTART):
            return
        try:
            self.serial_port.reset_input_buffer()
        except (serial.SerialException, OSError) as error:
            self.disconnect(f'lost {self.port} ({type(error).__name__}: {error})')

    def publish_state(self):
        # Only a Pico running its firmware applies what was written.
        connected = self.serial_port is not None and self.pico_running
        steer, steer_on = self.delivered.get('steering', (0.0, False))
        drive, drive_on = self.delivered.get('throttle', (0.0, False))
        msg = ActuatorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = connected
        msg.command_conflict = self.command_conflict
        msg.steering_active = connected and steer_on
        msg.throttle_active = connected and drive_on
        msg.steering = float(steer) if connected else 0.0
        msg.throttle = float(drive) if connected else 0.0
        self.state_pub.publish(msg)

    def connect(self):
        """Open the serial device and restart the Pico's firmware if it is not running."""
        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=0.1,
                exclusive=True,
            )
        except (serial.SerialException, OSError) as error:
            if str(error) != self.connect_error:
                self.get_logger().warning(f'cannot open {self.port}: {error}')
            self.connect_error = str(error)
            return
        self.connect_error = None
        self.get_logger().info(f'connected to {self.port}')
        # A Pico left in the raw REPL echoes nothing, so this is sent unasked.
        self.restart_pico(time.monotonic())

    def send(self, line):
        """Write one protocol line; on failure drop the port and return false."""
        if self.serial_port is None:
            return False
        try:
            # pyserial writes all of it or raises (SerialTimeoutException after write_timeout).
            self.serial_port.write(line.encode('ascii'))
            return True
        except (serial.SerialException, OSError) as error:
            self.disconnect(f'lost {self.port} ({type(error).__name__}: {error})')
            return False

    def disconnect(self, reason):
        """Close the current serial handle and log one transition."""
        self.get_logger().warning(f'{reason}; reconnecting')
        if self.serial_port is not None:
            release_port(self.serial_port)
        self.serial_port = None
        self.pico_running = False
        self.delivered.clear()
        for state in self.channels.values():
            state[2] = None

    def close(self):
        """Stop every channel and release the port during clean shutdown."""
        if self.serial_port is None:
            return
        for channel in self.channels:
            self.send(f'{channel} stop\n')
        if self.serial_port is not None:
            release_port(self.serial_port, drain_s=0.2)
            self.serial_port = None


def main(args=None):
    """Run the Pico Ackermann driver node."""
    rclpy.init(args=args)
    node = PicoAckermannDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
