#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class RobustMissionStarter(Node):
    """
    This node robustly waits for a Nav2 action server and then sends a goal
    that specifies a custom behavior tree to execute the mission.
    """
    def __init__(self):
        super().__init__('robust_mission_starter')
        # Use the standard NavigateToPose action client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def start_mission(self):
        self.get_logger().info('Waiting for the Nav2 action server (/navigate_to_pose) to be available...')
        self._action_client.wait_for_server()

        self.get_logger().info('Nav2 action server available. Sending goal to trigger MissionTree...')
        
        goal_msg = NavigateToPose.Goal()
        
        # We must provide a target pose, but since our BT overrides the logic,
        # this pose is just a formality to initiate the action. We can use the origin.
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map' # Or 'odom' depending on your setup
        
        # This is the crucial part: specify the ID of the Behavior Tree to run.
        # This must match the ID in your mission.xml file.
        goal_msg.behavior_tree = "MissionTree"

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MISSION GOAL REJECTED by Nav2.')
        else:
            self.get_logger().info('MISSION GOAL ACCEPTED by Nav2. The robot is now autonomous.')
        
        # We don't shut down here, as we might want to monitor the result.
        # For this simple starter, we will just exit after sending the goal.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    starter = RobustMissionStarter()
    starter.start_mission()
    rclpy.spin(starter)

if __name__ == '__main__':
    main()