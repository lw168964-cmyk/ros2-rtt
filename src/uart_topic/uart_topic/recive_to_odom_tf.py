#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from uart_msgs.msg import Recive


class ReciveToOdomTfNode(Node):
    def __init__(self):
        super().__init__('recive_to_odom_tf')

        self.declare_parameter('recive_topic', '/recive_data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 50.0)

        self.recive_topic = self.get_parameter('recive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.latest_msg = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.recive_sub = self.create_subscription(
            Recive,
            self.recive_topic,
            self.recive_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(publish_rate, 1.0),
            self.publish_latest_odom_tf,
        )

        self.get_logger().info(
            'convert %s to %s and TF %s -> %s at %.1f Hz'
            % (
                self.recive_topic,
                self.odom_topic,
                self.odom_frame,
                self.base_frame,
                publish_rate,
            )
        )

    @staticmethod
    def yaw_to_quaternion(yaw_rad):
        half_yaw = yaw_rad * 0.5
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)

    def recive_callback(self, msg):
        self.latest_msg = msg

    def publish_latest_odom_tf(self):
        now = self.get_clock().now().to_msg()

        if self.latest_msg is None:
            x = 0.0
            y = 0.0
            vx = 0.0
            vy = 0.0
            yaw_deg = 0.0
            gyro_z_dps = 0.0
        else:
            x = float(self.latest_msg.x)
            y = float(self.latest_msg.y)
            vx = float(self.latest_msg.vx)
            vy = float(self.latest_msg.vy)
            yaw_deg = float(self.latest_msg.yaw_deg)
            gyro_z_dps = float(self.latest_msg.gyro_z_dps)

        yaw_rad = math.radians(yaw_deg)
        gyro_z_rad = math.radians(gyro_z_dps)
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw_rad)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = gyro_z_rad

        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = 99999.0
        odom.pose.covariance[21] = 99999.0
        odom.pose.covariance[28] = 99999.0
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[7] = 0.1
        odom.twist.covariance[14] = 99999.0
        odom.twist.covariance[21] = 99999.0
        odom.twist.covariance[28] = 99999.0
        odom.twist.covariance[35] = 0.2

        self.odom_pub.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = ReciveToOdomTfNode()
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
