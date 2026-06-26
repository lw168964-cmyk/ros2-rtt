#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from uart_msgs.msg import Recive, Send


HELP_TEXT = """
Keyboard control for RTT UART
-----------------------------
w/s : increase/decrease target speed
a/left  : increase angular velocity
d/right : decrease angular velocity
x   : zero speed
space : zero speed and angular velocity
q   : quit
"""


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('uart_keyboard_control')

        self.declare_parameter('speed_step', 0.05)
        self.declare_parameter('angular_step', 0.05)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_angular_velocity', 0.45)
        self.declare_parameter('repeat_rate', 10.0)

        self.speed_step = float(self.get_parameter('speed_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value)
        repeat_rate = float(self.get_parameter('repeat_rate').value)

        self.target_speed = 0.0
        self.angular_velocity = 0.0
        self.latest_feedback = None

        self.publisher = self.create_publisher(Send, '/send_cmd', 10)
        self.subscription = self.create_subscription(
            Recive, '/recive_data', self.feedback_callback, 10)
        self.timer = self.create_timer(1.0 / repeat_rate, self.publish_command)

        self.get_logger().info(HELP_TEXT)

    def feedback_callback(self, msg):
        self.latest_feedback = msg

    def clamp_speed(self, value):
        return self.clamp_symmetric(value, self.max_speed)

    def clamp_symmetric(self, value, limit):
        return max(-limit, min(limit, value))

    def handle_key(self, key):
        if key == 'w':
            self.target_speed += self.speed_step
        elif key == 's':
            self.target_speed -= self.speed_step
        elif key == 'a' or key == '\x1b[D':
            self.angular_velocity += self.angular_step
        elif key == 'd' or key == '\x1b[C':
            self.angular_velocity -= self.angular_step
        elif key == 'x':
            self.target_speed = 0.0
        elif key == ' ':
            self.target_speed = 0.0
            self.angular_velocity = 0.0
        elif key == 'q':
            self.target_speed = 0.0
            self.angular_velocity = 0.0
            self.publish_command()
            return False

        self.target_speed = self.clamp_speed(self.target_speed)
        self.angular_velocity = self.clamp_symmetric(
            self.angular_velocity,
            self.max_angular_velocity,
        )
        self.print_status()
        return True

    def publish_command(self):
        msg = Send()
        msg.target_speed = float(self.target_speed)
        msg.steer_angle = float(self.angular_velocity)
        self.publisher.publish(msg)

    def print_status(self):
        feedback = ''
        if self.latest_feedback is not None:
            feedback = (
                f" | x={self.latest_feedback.x:.2f} y={self.latest_feedback.y:.2f}"
                f" vx={self.latest_feedback.vx:.2f} vy={self.latest_feedback.vy:.2f}"
                f" yaw={self.latest_feedback.yaw_deg:.2f}"
                f" gyro_z={self.latest_feedback.gyro_z_dps:.2f} dps"
            )
        self.get_logger().info(
            f"send target_speed={self.target_speed:.3f}, "
            f"angular_velocity={self.angular_velocity:.3f} rad/s{feedback}"
        )


def read_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
    if key == '\x1b':
        ready, _, _ = select.select([sys.stdin], [], [], 0.01)
        if ready:
            key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(settings)
            if key:
                running = node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.target_speed = 0.0
        node.angular_velocity = 0.0
        node.publish_command()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
