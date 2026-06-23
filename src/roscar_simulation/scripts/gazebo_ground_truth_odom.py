#!/usr/bin/env python3

import math

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class GazeboGroundTruthOdom(Node):
    def __init__(self):
        super().__init__('gazebo_ground_truth_odom')
        self.model_name = self.declare_parameter('model_name', 'roscar').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.relative_to_start = self.declare_parameter('relative_to_start', True).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.odom_pub = self.create_publisher(Odometry, 'ground_truth/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(ModelStates, 'model_states', self.on_model_states, 10)

    def on_model_states(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[index]
        twist = msg.twist[index]
        stamp = self.get_clock().now().to_msg()
        yaw = yaw_from_quaternion(pose.orientation)

        odom_x = pose.position.x
        odom_y = pose.position.y
        odom_yaw = yaw
        if self.relative_to_start:
            if self.initial_x is None:
                self.initial_x = pose.position.x
                self.initial_y = pose.position.y
                self.initial_yaw = yaw
                self.get_logger().info('Using current Gazebo pose as the odom origin.')

            dx = pose.position.x - self.initial_x
            dy = pose.position.y - self.initial_y
            cos_yaw = math.cos(self.initial_yaw)
            sin_yaw = math.sin(self.initial_yaw)
            odom_x = cos_yaw * dx + sin_yaw * dy
            odom_y = -sin_yaw * dx + cos_yaw * dy
            odom_yaw = math.atan2(
                math.sin(yaw - self.initial_yaw),
                math.cos(yaw - self.initial_yaw),
            )

        qx, qy, qz, qw = quaternion_from_yaw(odom_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist = twist
        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom_x
        transform.transform.translation.y = odom_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GazeboGroundTruthOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
