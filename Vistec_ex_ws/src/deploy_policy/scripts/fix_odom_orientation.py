#!/usr/bin/env python3
"""
Fix odometry by merging position from /odom and orientation from /imu/data.

Gazebo's OdometryPublisher plugin doesn't include orientation, so we fix it here.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from threading import Lock


class OdomOrientationFixer(Node):
    def __init__(self):
        super().__init__('odom_orientation_fixer')

        # State
        self.latest_imu = None
        self.latest_pose_z = 0.0  # Latest z position from Gazebo pose
        self.prev_pose_z = 0.0     # Previous z position for velocity computation
        self.prev_z_time = None    # Previous z timestamp
        self.vel_z = 0.0           # Computed z velocity
        self.lock = Lock()

        # QoS profile for Gazebo topics (BEST_EFFORT required)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for output (RELIABLE for subscribers)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to broken odometry (Gazebo topic - needs BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_best_effort
        )

        # Subscribe to IMU for orientation (Gazebo topic - needs BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos_best_effort
        )

        # Subscribe to robot z position from Gazebo bridge
        self.z_sub = self.create_subscription(
            Float64,
            '/robot_z_position',
            self.z_callback,
            10
        )

        # Publish fixed odometry (RELIABLE for subscribers)
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom_fixed',
            qos_reliable
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("Odometry Orientation Fixer Started")
        self.get_logger().info("Subscribing to:")
        self.get_logger().info("  - /odom (x, y position)")
        self.get_logger().info("  - /imu/data (orientation)")
        self.get_logger().info("  - /robot_z_position (z from Gazebo)")
        self.get_logger().info("Publishing to: /odom_fixed (complete odometry)")
        self.get_logger().info("=" * 60)

    def imu_callback(self, msg: Imu):
        """Store latest IMU data."""
        with self.lock:
            self.latest_imu = msg

    def z_callback(self, msg: Float64):
        """Store latest z position from Gazebo and compute z velocity."""
        with self.lock:
            current_time = self.get_clock().now()

            # Compute z velocity if we have a previous measurement
            if self.prev_z_time is not None:
                dt = (current_time - self.prev_z_time).nanoseconds / 1e9
                if dt > 0:
                    self.vel_z = (msg.data - self.prev_pose_z) / dt

            # Update state
            self.prev_pose_z = self.latest_pose_z
            self.latest_pose_z = msg.data
            self.prev_z_time = current_time

    def odom_callback(self, msg: Odometry):
        """Fix odometry by adding IMU orientation and Gazebo pose z-position."""
        with self.lock:
            if self.latest_imu is None:
                self.get_logger().warn("No IMU data yet, skipping...", throttle_duration_sec=5.0)
                return

            # Create fixed odometry message
            fixed_odom = Odometry()

            # Copy header and frames
            fixed_odom.header = msg.header
            fixed_odom.child_frame_id = msg.child_frame_id

            # Copy position from original odom (x, y from /odom)
            fixed_odom.pose.pose.position = msg.pose.pose.position

            # Override z position with latest from Gazebo pose
            # (Gazebo's /odom has z=0, so we use /robot_pose instead)
            fixed_odom.pose.pose.position.z = self.latest_pose_z

            # Copy velocity from original odom
            fixed_odom.twist.twist = msg.twist.twist

            # Override z velocity with computed value
            # (Gazebo's /odom has vz=0, so we compute it from position)
            fixed_odom.twist.twist.linear.z = self.vel_z

            # Use orientation from IMU
            fixed_odom.pose.pose.orientation = self.latest_imu.orientation

            # Use angular velocity from IMU (more accurate)
            fixed_odom.twist.twist.angular = self.latest_imu.angular_velocity

            # Copy covariances
            fixed_odom.pose.covariance = msg.pose.covariance
            fixed_odom.twist.covariance = msg.twist.covariance

            # Publish fixed odometry
            self.odom_pub.publish(fixed_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomOrientationFixer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
