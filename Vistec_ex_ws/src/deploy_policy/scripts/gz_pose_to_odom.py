#!/usr/bin/env python3
"""
Convert Gazebo model pose to proper odometry messages.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

class GzPoseToOdom(Node):
    def __init__(self):
        super().__init__('gz_pose_to_odom')
        
        # State
        self.last_pose = None
        self.last_time = None
        
        # Subscribe to Gazebo model state (this topic depends on Gazebo configuration)
        # Try common Gazebo topics
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/go2/pose',  # Will be remapped from Gazebo topic
            self.pose_callback,
            10
        )
        
        # Publish odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.get_logger().info("Gazebo Pose to Odometry converter started")
    
    def pose_callback(self, msg: PoseStamped):
        """Convert pose to odometry with velocity estimation."""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        
        # Copy pose
        odom.pose.pose = msg.pose
        
        # Estimate velocity if we have previous pose
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt > 0:
                # Linear velocity
                dx = msg.pose.position.x - self.last_pose.position.x
                dy = msg.pose.position.y - self.last_pose.position.y
                dz = msg.pose.position.z - self.last_pose.position.z
                
                odom.twist.twist.linear.x = dx / dt
                odom.twist.twist.linear.y = dy / dt
                odom.twist.twist.linear.z = dz / dt
                
                # Angular velocity (simplified - just use IMU if available)
                # For now, set to zero (would need proper quaternion differentiation)
                odom.twist.twist.angular.x = 0.0
                odom.twist.twist.angular.y = 0.0
                odom.twist.twist.angular.z = 0.0
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Update history
        self.last_pose = msg.pose
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = GzPoseToOdom()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
