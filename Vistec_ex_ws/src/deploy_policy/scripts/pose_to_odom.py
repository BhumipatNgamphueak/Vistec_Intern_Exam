#!/usr/bin/env python3
"""
Converts Gazebo pose to odometry by computing velocity from pose changes.
Publishes to /model/go2_robot/odometry for data logger compatibility.
Uses /world/default/dynamic_pose/info from Gazebo.
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench  # Placeholder, we'll use Entity
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation

# Try to import Gazebo message types
try:
    from ros_gz_interfaces.msg import Entity
except ImportError:
    Entity = None

class PoseToOdomNode(Node):
    def __init__(self):
        super().__init__('pose_to_odom')

        # State
        self.last_pose = None
        self.last_time = None
        self.last_quat = np.array([0, 0, 0, 1])
        self.ang_vel = np.array([0.0, 0.0, 0.0])
        self.current_pose = None
        self.current_quat = None

        # Subscribers - use TF instead of pose topic
        # We'll create a simple TF listener or use joint_states + IMU
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/model/go2_robot/pose',  # Keep this for now, will add fallback
            self.pose_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/model/go2_robot/odometry',
            10
        )

        self.get_logger().info("Pose to Odometry converter started")

    def imu_callback(self, msg: Imu):
        """Get angular velocity from IMU."""
        self.ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

    def pose_callback(self, msg: PoseStamped):
        """Convert pose to odometry."""
        current_time = self.get_clock().now()

        # Extract pose
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        # Compute linear velocity
        if self.last_pose is not None and self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                # Velocity in world frame
                vel_world = (pos - self.last_pose) / dt

                # Transform to body frame
                rot = Rotation.from_quat(quat)
                vel_body = rot.inv().apply(vel_world)
            else:
                vel_body = np.zeros(3)
        else:
            vel_body = np.zeros(3)

        # Create odometry message
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base'

        # Pose
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation = msg.pose.orientation

        # Twist (velocity in body frame)
        odom.twist.twist.linear.x = vel_body[0]
        odom.twist.twist.linear.y = vel_body[1]
        odom.twist.twist.linear.z = vel_body[2]
        odom.twist.twist.angular.x = self.ang_vel[0]
        odom.twist.twist.angular.y = self.ang_vel[1]
        odom.twist.twist.angular.z = self.ang_vel[2]

        # Publish
        self.odom_pub.publish(odom)

        # Update state
        self.last_pose = pos
        self.last_time = current_time
        self.last_quat = quat

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
