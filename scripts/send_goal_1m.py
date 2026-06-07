#!/usr/bin/env python3
"""Send a single Nav2 goal 1 metre straight ahead in the odom frame."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send(self):
        self._client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'odom'
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0   # facing forward, no rotation
        self._client.send_goal_async(goal)
        self.get_logger().info('Goal sent: 1 m ahead in odom frame')


def main():
    rclpy.init()
    node = GoalSender()
    node.send()
    rclpy.spin_once(node, timeout_sec=2.0)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
