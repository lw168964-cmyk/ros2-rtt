import argparse
import math

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Publish the initial AMCL pose for roscar.')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    navigator = BasicNavigator()  # 节点
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = quaternion_from_yaw(args.yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    navigator.setInitialPose(pose)
    navigator.get_logger().info(
        f'Set initial pose: x={args.x:.3f}, y={args.y:.3f}, yaw={args.yaw:.3f}')
    navigator.waitUntilNav2Active()  # 等待导航系统激活
    rclpy.spin(navigator)
    rclpy.shutdown()
