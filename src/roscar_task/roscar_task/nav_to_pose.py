import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
import rclpy


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def parse_args():
    parser = argparse.ArgumentParser(description='Navigate roscar to a map pose.')
    parser.add_argument('--x', type=float, default=2.0)
    parser.add_argument('--y', type=float, default=1.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    navigator = BasicNavigator()  # 节点
    navigator.waitUntilNav2Active()  # 等待导航系统激活

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = args.x
    goal_pose.pose.position.y = args.y
    goal_pose.pose.position.z = 0.0
    qx, qy, qz, qw = quaternion_from_yaw(args.yaw)
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    navigator.get_logger().info(
        f'发送导航目标: x={args.x:.3f}, y={args.y:.3f}, yaw={args.yaw:.3f}')
    navigator.goToPose(goal_pose)
    last_feedback_time = 0.0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        now = time.monotonic()
        if feedback is not None and now - last_feedback_time >= 1.0:
            navigator.get_logger().info(
                f'剩余距离: {feedback.distance_remaining:.3f} m')  # 打印导航反馈
            last_feedback_time = now

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航成功！')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('导航被取消。')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('导航失败。')

    navigator.destroy_node()
    rclpy.shutdown()
