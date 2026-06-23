import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotPoseListener(Node):
    def __init__(self):
        super().__init__('get_robot_pose')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 1.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        publish_rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / publish_rate, self.print_robot_pose)

        self.get_logger().info(
            f'Listening robot pose from {self.map_frame} to {self.base_frame}')

    def print_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.map_frame} -> {self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(rotation)

        self.get_logger().info(
            f'robot_pose: x={translation.x:.3f}, '
            f'y={translation.y:.3f}, yaw={yaw:.3f} rad')


def main():
    rclpy.init()
    node = RobotPoseListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
