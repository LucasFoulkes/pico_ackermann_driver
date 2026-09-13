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


def clamp_command(value, limit=1.0):
    """Clamp a finite command to a configured symmetric limit."""
    return max(-limit, min(limit, value))


def channel_topic(channel):
    """Return the ROS command topic for a protocol channel."""
    return f'{TOPIC_PREFIX}/{channel}/command'


class PicoAckermannDriver(Node):
    """Own the Pico serial port and enforce per-channel input timeouts."""

    def __init__(self):
        super().__init__('pico_ackermann_driver')

        channels = list(
            self.declare_parameter('channels', ['steering']).value)
        self.port = self.declare_parameter(
            'port', '/dev/pico-ackermann').value
        self.baud = self.declare_parameter('baud', 115200).value
        send_hz = self.declare_parameter('send_hz', 50.0).value
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
        if not 0 < self.command_limit <= 2.0:
            raise ValueError('command_limit must be in (0, 2]')
        if self.input_timeout <= 0:
            raise ValueError('input_timeout must be greater than zero')
        if self.baud <= 0:
            raise ValueError('baud must be greater than zero')

        self.serial_port = None
        self.delivered = {}
        self.command_conflict = False
        self.owner_check_at = 0.0
        self.state_pub = self.create_publisher(ActuatorState, '/actuators/state', 5)
        self.channels = {
            name: [0.0, None, False]
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
        """Store a valid command and its monotonic arrival time."""
        state = self.channels[channel]
        if not math.isfinite(message.data):
            # Invalid input revokes the previous command immediately.
            state[:] = [0.0, None, False]
            return
        state[0] = clamp_command(message.data, self.command_limit)
        state[1] = time.monotonic_ns()
        state[2] = False

    def tick(self):
        """Refresh active channels and stop channels that become stale."""
        self.check_owners()
        if self.serial_port is None:
            self.connect()

        now = time.monotonic_ns()
        timeout_ns = int(self.input_timeout * 1_000_000_000)
        for channel, state in self.channels.items():
            value, updated_ns, stop_sent = state
            fresh = (
                not self.command_conflict
                and updated_ns is not None
                and now - updated_ns < timeout_ns
            )
            if fresh:
                if self.send(f'{channel} {value:.5f}\n'):
                    self.delivered[channel] = (float(f'{value:.5f}'), True)
            elif not stop_sent and self.send(f'{channel} stop\n'):
                state[2] = True
                self.delivered[channel] = (0.0, False)
        self.publish_state()

    def check_owners(self):
        now = time.monotonic()
        if now-self.owner_check_at < .2:
            return
        self.owner_check_at = now
        conflict = any(len(self.get_publishers_info_by_topic(channel_topic(channel))) > 1
                       for channel in self.channels)
        if conflict != self.command_conflict:
            self.get_logger().warning('multiple actuator publishers: stopping' if conflict else 'actuator ownership restored; waiting for fresh commands')
            for state in self.channels.values():
                state[:] = [0.0, None, False]
        self.command_conflict = conflict

    def publish_state(self):
        connected = self.serial_port is not None
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
        """Open the configured serial device and clear a partial line."""
        try:
            candidate = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=0.1,
                exclusive=True,
            )
            if candidate.write(b'\n') != 1:
                candidate.close()
                return
            self.serial_port = candidate
            self.get_logger().info(f'connected to {self.port}')
        except (serial.SerialException, OSError):
            self.serial_port = None

    def send(self, line):
        """Return true only after writing one complete protocol line."""
        if self.serial_port is None:
            return False

        payload = line.encode('ascii')
        try:
            written = self.serial_port.write(payload)
            if written == len(payload):
                return True
            self.disconnect('partial serial write')
        except (serial.SerialException, OSError):
            self.disconnect(f'lost {self.port}')
        return False

    def disconnect(self, reason):
        """Close the current serial handle and log one transition."""
        self.get_logger().warning(f'{reason}; reconnecting')
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (serial.SerialException, OSError):
                pass
        self.serial_port = None
        self.delivered.clear()
        for state in self.channels.values():
            state[2] = False

    def close(self):
        """Release the serial device during clean process shutdown."""
        if self.serial_port is not None:
            for channel in self.channels:
                self.send(f'{channel} stop\n')
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except (serial.SerialException, OSError):
                    pass
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
