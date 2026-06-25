import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from uart_msgs.msg import Recive, Send


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class CmdVelToRttNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rtt')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav_smoothed')
        self.declare_parameter('send_topic', '/send_cmd')
        self.declare_parameter('feedback_topic', '/recive_data')
        self.declare_parameter('max_speed', 0.1)
        self.declare_parameter('max_angle', 180.0)
        self.declare_parameter('speed_deadband', 0.003)
        self.declare_parameter('angular_deadband', 0.01)
        self.declare_parameter('angular_lookahead', 1.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.send_topic = self.get_parameter('send_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.max_speed = abs(float(self.get_parameter('max_speed').value))
        self.max_angle = abs(float(self.get_parameter('max_angle').value))
        self.speed_deadband = abs(float(self.get_parameter('speed_deadband').value))
        self.angular_deadband = abs(float(self.get_parameter('angular_deadband').value))
        self.angular_lookahead = float(self.get_parameter('angular_lookahead').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.current_speed = 0.0
        self.feedback_yaw_deg = None
        self.target_angle_deg = 0.0
        self.last_cmd_time = None

        self.publisher = self.create_publisher(Send, self.send_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )
        self.feedback_sub = self.create_subscription(
            Recive,
            self.feedback_topic,
            self.feedback_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(publish_rate, 1.0),
            self.publish_command,
        )

        self.get_logger().info(
            'convert %s to %s, target_speed limited to %.3f m/s'
            % (self.cmd_vel_topic, self.send_topic, self.max_speed)
        )

    def feedback_callback(self, msg):
        self.feedback_yaw_deg = normalize_degrees(float(msg.yaw_deg))

    def cmd_callback(self, msg):
        speed = clamp(float(msg.linear.x), -self.max_speed, self.max_speed)
        if abs(speed) < self.speed_deadband:
            speed = 0.0

        angular_z = float(msg.angular.z)
        if abs(angular_z) < self.angular_deadband:
            angular_z = 0.0

        base_angle = self.target_angle_deg
        if self.feedback_yaw_deg is not None:
            base_angle = self.feedback_yaw_deg

        if angular_z == 0.0:
            if self.feedback_yaw_deg is not None:
                self.target_angle_deg = self.feedback_yaw_deg
        else:
            angle_delta = math.degrees(angular_z * self.angular_lookahead)
            self.target_angle_deg = normalize_degrees(base_angle + angle_delta)

        self.target_angle_deg = clamp(
            self.target_angle_deg,
            -self.max_angle,
            self.max_angle,
        )
        self.current_speed = speed
        self.last_cmd_time = self.get_clock().now()

    def command_is_stale(self):
        if self.last_cmd_time is None:
            return True
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age > self.command_timeout

    def publish_command(self):
        msg = Send()
        if self.command_is_stale():
            msg.target_speed = 0.0
            if self.feedback_yaw_deg is not None:
                self.target_angle_deg = self.feedback_yaw_deg
        else:
            msg.target_speed = float(self.current_speed)

        msg.steer_angle = float(self.target_angle_deg)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Send()
        stop.target_speed = 0.0
        stop.steer_angle = float(node.target_angle_deg)
        node.publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
