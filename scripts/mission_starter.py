#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ExecuteBehaviorTree

class RobustMissionStarter(Node):
    """
    This node robustly waits until the BT action server is available and then
    sends the initial goal to start the mission.
    """
    def __init__(self):
        super().__init__('robust_mission_starter')
        self._action_client = ActionClient(self, ExecuteBehaviorTree, '/execute_behavior_tree')

    def start_mission(self):
        self.get_logger().info('Waiting for the BT action server to be available...')
        self._action_client.wait_for_server()

        self.get_logger().info('BT action server available. Sending goal...')
        
        goal_msg = ExecuteBehaviorTree.Goal()
        goal_msg.behavior_tree = "MissionTree"

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MISSION GOAL REJECTED')
        else:
            self.get_logger().info('MISSION GOAL ACCEPTED. Robot is now autonomous.')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    starter = RobustMissionStarter()
    starter.start_mission()
    rclpy.spin(starter)

if __name__ == '__main__':
    main()
