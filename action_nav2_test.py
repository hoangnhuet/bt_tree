#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion


def main(args=None):
    rclpy.init(args=args)
    node = Node('simple_bt_nav_client')

    action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    node.get_logger().info('Waiting for action server...')
    action_client.wait_for_server()

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position = Point(x=1.0, y=1.0, z=0.0)
    pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = pose

    node.get_logger().info('Sending goal...')
    send_goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().info('FAILURE - goal rejected')
        rclpy.shutdown()
        return

    # If accepted, get the result asynchronously
    get_result_future = goal_handle.get_result_async()

    # Print "RUNNING" while waiting for the result
    while not get_result_future.done():
        node.get_logger().info('RUNNING')
        rclpy.spin_once(node, timeout_sec=1.0)

    # Determine final state
    result = get_result_future.result()
    print(result)
    if result.status == 4:  # 4 -> SUCCESS
        node.get_logger().info('SUCCESS')
    else:
        node.get_logger().info('FAILURE')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
