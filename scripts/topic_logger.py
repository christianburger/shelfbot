#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

class TopicLoggerNode(Node):
    def __init__(self):
        super().__init__('topic_logger')
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.get_logger().info("Topic logger node started.")

    def image_callback(self, msg):
        self.get_logger().info(
            f'Received Image: stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
            f'height={msg.height}, width={msg.width}, encoding={msg.encoding}'
        )

    def odom_callback(self, msg):
        self.get_logger().info(
            f'Received Odometry: stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, '
            f'position.x={msg.pose.pose.position.x:.4f}, '
            f'orientation.z={msg.pose.pose.orientation.z:.4f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TopicLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
