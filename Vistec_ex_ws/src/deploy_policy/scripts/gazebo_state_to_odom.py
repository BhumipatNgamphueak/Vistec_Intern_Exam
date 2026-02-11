#!/usr/bin/env python3
"""
Convert Gazebo model state to odometry using tf2.
Works by listening to tf transforms published by robot_state_publisher.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import numpy as np

class GazeboStateToOdom(Node):
    def __init__(self):
        super().__init__('gazebo_state_to_odom')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.last_position = None
        self.last_time = None
        self.parent_frame = None
        self.child_frame = 'base'

        # Possible parent frames to try (in order of preference)
        self.candidate_frames = ['world', 'odom', 'map', 'base_footprint']

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer to publish odometry at 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("Gazebo State to Odometry converter started")
        self.get_logger().info(f"Will auto-detect parent frame for TF: <parent> -> {self.child_frame}")

    def timer_callback(self):
        """Get transform and publish odometry."""
        try:
            # Auto-detect parent frame on first successful lookup
            if self.parent_frame is None:
                for candidate in self.candidate_frames:
                    try:
                        self.tf_buffer.lookup_transform(
                            candidate,
                            self.child_frame,
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.05)
                        )
                        self.parent_frame = candidate
                        self.get_logger().info(f"✅ Found TF: {self.parent_frame} -> {self.child_frame}")
                        break
                    except:
                        continue

                if self.parent_frame is None:
                    # No frame found yet, skip this iteration
                    return

            # Get transform from parent to base
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            current_time = self.get_clock().now()

            # Create odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'base'

            # Copy position from TF
            odom.pose.pose.position.x = trans.transform.translation.x
            odom.pose.pose.position.y = trans.transform.translation.y
            odom.pose.pose.position.z = trans.transform.translation.z

            # Copy orientation from TF
            odom.pose.pose.orientation = trans.transform.rotation

            # Compute velocity by differentiation
            current_pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])

            if self.last_position is not None and self.last_time is not None:
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt > 0.001:  # At least 1ms
                    vel = (current_pos - self.last_position) / dt
                    odom.twist.twist.linear.x = float(vel[0])
                    odom.twist.twist.linear.y = float(vel[1])
                    odom.twist.twist.linear.z = float(vel[2])
                else:
                    # dt too small, use previous velocity or zero
                    odom.twist.twist.linear.x = 0.0
                    odom.twist.twist.linear.y = 0.0
                    odom.twist.twist.linear.z = 0.0

            # Publish
            self.odom_pub.publish(odom)

            # Update state for next iteration
            self.last_position = current_pos.copy()
            self.last_time = current_time

        except Exception as e:
            # Log once every 100 failures to avoid spam
            if not hasattr(self, 'error_count'):
                self.error_count = 0
            self.error_count += 1
            if self.error_count % 100 == 1:
                self.get_logger().warn(f"TF lookup failed (count: {self.error_count}): {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = GazeboStateToOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
