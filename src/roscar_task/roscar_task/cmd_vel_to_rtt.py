import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from uart_msgs.msg import Send


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class CmdVelToRttNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rtt')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav_smoothed')
        self.declare_parameter('send_topic', '/send_cmd')
        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('max_angular_velocity', 0.45)
        self.declare_parameter('speed_deadband', 0.003)
        self.declare_parameter('angular_deadband', 0.0)
        self.declare_parameter('straight_speed_threshold', 0.05)
        self.declare_parameter('straight_angular_deadband', 0.05)
        self.declare_parameter('speed_filter_alpha', 0.35)
        self.declare_parameter('angular_filter_alpha', 0.30)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.send_topic = self.get_parameter('send_topic').value
        self.max_speed = abs(float(self.get_parameter('max_speed').value))
        self.max_angular_velocity = abs(
            float(self.get_parameter('max_angular_velocity').value))
        self.speed_deadband = abs(float(self.get_parameter('speed_deadband').value))
        self.angular_deadband = abs(float(self.get_parameter('angular_deadband').value))
        self.straight_speed_threshold = abs(
            float(self.get_parameter('straight_speed_threshold').value))
        self.straight_angular_deadband = abs(
            float(self.get_parameter('straight_angular_deadband').value))
        self.speed_filter_alpha = clamp(
            float(self.get_parameter('speed_filter_alpha').value), 0.0, 1.0)
        self.angular_filter_alpha = clamp(
            float(self.get_parameter('angular_filter_alpha').value), 0.0, 1.0)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.target_speed = 0.0
        self.target_angular_velocity = 0.0
        self.filtered_speed = 0.0
        self.filtered_angular_velocity = 0.0
        self.last_cmd_time = None

        self.publisher = self.create_publisher(Send, self.send_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(publish_rate, 1.0),
            self.publish_command,
        )

        self.get_logger().info(
            'convert %s to %s: speed +/-%.3f m/s, angular +/-%.3f rad/s'
            % (
                self.cmd_vel_topic,
                self.send_topic,
                self.max_speed,
                self.max_angular_velocity,
            )
        )

    def cmd_callback(self, msg):
        speed = clamp(float(msg.linear.x), -self.max_speed, self.max_speed)
        angular_velocity = clamp(
            float(msg.angular.z) * self.angular_scale,
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )

        if abs(speed) < self.speed_deadband:
            speed = 0.0
        if abs(angular_velocity) < self.angular_deadband:
            angular_velocity = 0.0

        self.target_speed = speed
        self.target_angular_velocity = angular_velocity
        self.last_cmd_time = self.get_clock().now()

    def command_is_stale(self):
        if self.last_cmd_time is None:
            return True
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age > self.command_timeout

    def low_pass(self, current, target, alpha):
        return current + alpha * (target - current)

    def publish_command(self):
        msg = Send()
        if self.command_is_stale():
            self.target_speed = 0.0
            self.target_angular_velocity = 0.0

        self.filtered_speed = self.low_pass(
            self.filtered_speed,
            self.target_speed,
            self.speed_filter_alpha,
        )
        self.filtered_angular_velocity = self.low_pass(
            self.filtered_angular_velocity,
            self.target_angular_velocity,
            self.angular_filter_alpha,
        )

        if abs(self.filtered_speed) < self.speed_deadband:
            self.filtered_speed = 0.0
        if (
            abs(self.filtered_speed) >= self.straight_speed_threshold
            and self.filtered_angular_velocity < 0.0
            and abs(self.filtered_angular_velocity) < self.straight_angular_deadband
        ):
            self.filtered_angular_velocity = 0.0
        if abs(self.filtered_angular_velocity) < self.angular_deadband:
            self.filtered_angular_velocity = 0.0

        msg.target_speed = float(self.filtered_speed)
        msg.steer_angle = float(self.filtered_angular_velocity)
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
        stop.steer_angle = 0.0
        node.publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
