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
        self.declare_parameter('max_speed', 0.1)
        self.declare_parameter('max_angular_velocity', 0.45)
        self.declare_parameter('speed_deadband', 0.003)
        self.declare_parameter('angular_deadband', 0.03)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.send_topic = self.get_parameter('send_topic').value
        self.max_speed = abs(float(self.get_parameter('max_speed').value))
        self.max_angular_velocity = abs(
            float(self.get_parameter('max_angular_velocity').value))
        self.speed_deadband = abs(float(self.get_parameter('speed_deadband').value))
        self.angular_deadband = abs(float(self.get_parameter('angular_deadband').value))
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.current_speed = 0.0
        self.current_angular_velocity = 0.0
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
            float(msg.angular.z),
            -self.max_angular_velocity,
            self.max_angular_velocity,
        )

        if abs(speed) < self.speed_deadband:
            speed = 0.0
        if abs(angular_velocity) < self.angular_deadband:
            angular_velocity = 0.0

        self.current_speed = speed
        self.current_angular_velocity = angular_velocity
        self.last_cmd_time = self.get_clock().now()

    def command_is_stale(self):
        if self.last_cmd_time is None:
            return True
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return age > self.command_timeout

    def publish_command(self):
        msg = Send()
        if not self.command_is_stale():
            msg.target_speed = float(self.current_speed)
            msg.steer_angle = float(self.current_angular_velocity)
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
