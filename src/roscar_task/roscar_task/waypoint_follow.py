import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
import rclpy


DEFAULT_WAYPOINTS = [
    (0.8, 0.0, 0.0),
    (1.2, 0.6, 0.7),
    (1.6, 1.0, 1.57),
]


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def parse_waypoints(text):
    waypoints = []
    for raw_item in text.split(';'):
        item = raw_item.strip()
        if not item:
            continue

        values = [value.strip() for value in item.split(',')]
        if len(values) not in (2, 3):
            raise argparse.ArgumentTypeError(
                '每个路点格式应为 x,y 或 x,y,yaw，例如 "1.0,2.0,0.0;2.0,2.0,1.57"')

        try:
            x = float(values[0])
            y = float(values[1])
            yaw = float(values[2]) if len(values) == 3 else 0.0
        except ValueError as exc:
            raise argparse.ArgumentTypeError('路点坐标必须是数字') from exc

        waypoints.append((x, y, yaw))

    if not waypoints:
        raise argparse.ArgumentTypeError('至少需要一个路点')

    return waypoints


def parse_args():
    parser = argparse.ArgumentParser(description='Follow a sequence of map waypoints.')
    parser.add_argument(
        '--waypoints',
        type=parse_waypoints,
        default=DEFAULT_WAYPOINTS,
        help='路点列表，格式: "x,y,yaw;x,y,yaw"。yaw 单位是弧度，可省略。',
    )
    return parser.parse_args()


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main():
    args = parse_args()

    rclpy.init()
    navigator = BasicNavigator()

    try:
        navigator.waitUntilNav2Active()

        navigator.get_logger().info(f'开始路点导航，共 {len(args.waypoints)} 个路点')
        for index, (x, y, yaw) in enumerate(args.waypoints, start=1):
            navigator.get_logger().info(
                f'路点 {index}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')

        for index, (x, y, yaw) in enumerate(args.waypoints, start=1):
            goal_pose = make_pose(navigator, x, y, yaw)
            navigator.get_logger().info(
                f'发送路点 {index}/{len(args.waypoints)}: '
                f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
            navigator.goToPose(goal_pose)

            last_feedback_time = 0.0
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                now = time.monotonic()
                if feedback is not None and now - last_feedback_time >= 1.0:
                    navigator.get_logger().info(
                        f'路点 {index}/{len(args.waypoints)} '
                        f'剩余距离: {feedback.distance_remaining:.3f} m')
                    last_feedback_time = now

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info(f'路点 {index} 到达')
                continue

            if result == TaskResult.CANCELED:
                navigator.get_logger().info(f'路点 {index} 导航被取消。')
            elif result == TaskResult.FAILED:
                navigator.get_logger().info(f'路点 {index} 导航失败。')
            else:
                navigator.get_logger().info(f'路点 {index} 导航结束，结果: {result}')
            return

        navigator.get_logger().info('全部路点导航完成！')
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
