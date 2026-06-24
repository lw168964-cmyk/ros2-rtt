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
a/d : increase/decrease steer angle
x   : zero speed
space : zero speed and steer angle
q   : quit
"""


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('uart_keyboard_control')

        self.declare_parameter('speed_step', 100.0)
        self.declare_parameter('angle_step', 2.0)
        self.declare_parameter('max_speed', 4000.0)
        self.declare_parameter('max_angle', 30.0)
        self.declare_parameter('repeat_rate', 10.0)

        self.speed_step = float(self.get_parameter('speed_step').value)
        self.angle_step = float(self.get_parameter('angle_step').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        repeat_rate = float(self.get_parameter('repeat_rate').value)

        self.target_speed = 0.0
        self.steer_angle = 0.0
        self.latest_feedback = None

        self.publisher = self.create_publisher(Send, '/send_cmd', 10)
        self.subscription = self.create_subscription(
            Recive, '/recive_data', self.feedback_callback, 10)
        self.timer = self.create_timer(1.0 / repeat_rate, self.publish_command)

        self.get_logger().info(HELP_TEXT)

    def feedback_callback(self, msg):
        self.latest_feedback = msg

    def clamp_speed(self, value):
        return max(0.0, min(self.max_speed, value))

    def clamp_symmetric(self, value, limit):
        return max(-limit, min(limit, value))

    def handle_key(self, key):
        if key == 'w':
            self.target_speed += self.speed_step
        elif key == 's':
            self.target_speed -= self.speed_step
        elif key == 'a':
            self.steer_angle += self.angle_step
        elif key == 'd':
            self.steer_angle -= self.angle_step
        elif key == 'x':
            self.target_speed = 0.0
        elif key == ' ':
            self.target_speed = 0.0
            self.steer_angle = 0.0
        elif key == 'q':
            self.target_speed = 0.0
            self.steer_angle = 0.0
            self.publish_command()
            return False

        self.target_speed = self.clamp_speed(self.target_speed)
        self.steer_angle = self.clamp_symmetric(self.steer_angle, self.max_angle)
        self.print_status()
        return True

    def publish_command(self):
        msg = Send()
        msg.target_speed = float(self.target_speed)
        msg.steer_angle = float(self.steer_angle)
        self.publisher.publish(msg)

    def print_status(self):
        feedback = ''
        if self.latest_feedback is not None:
            feedback = (
                f" | yaw={self.latest_feedback.yaw_deg:.2f} deg"
                f" gyro_z={self.latest_feedback.gyro_z_dps:.2f} dps"
            )
        self.get_logger().info(
            f"send target_speed={self.target_speed:.3f}, "
            f"steer_angle={self.steer_angle:.2f}{feedback}"
        )


def read_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ''
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
        node.steer_angle = 0.0
        node.publish_command()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
