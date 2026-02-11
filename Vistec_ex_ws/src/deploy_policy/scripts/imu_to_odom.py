#!/usr/bin/env python3
"""
Simple IMU-based odometry publisher for data collection.
Publishes minimal odometry from IMU data.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

class ImuToOdomNode(Node):
    def __init__(self):
        super().__init__('imu_to_odom')

        # State (assume robot stays near origin for data collection)
        self.position = np.array([0.0, 0.0, 0.32])  # Initial height

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.get_logger().info("IMU to Odometry converter started")
        self.get_logger().warn("Using IMU-only odometry (position fixed at origin)")

    def imu_callback(self, msg: Imu):
        """Convert IMU to odometry."""
        # Create odometry message
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base'

        # Pose (fixed position, orientation from IMU)
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        odom.pose.pose.orientation = msg.orientation

        # Twist (assume zero linear velocity, angular from IMU)
        # For now, set zero linear velocity (robot movement assumed small)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = msg.angular_velocity.x
        odom.twist.twist.angular.y = msg.angular_velocity.y
        odom.twist.twist.angular.z = msg.angular_velocity.z

        # Publish
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
