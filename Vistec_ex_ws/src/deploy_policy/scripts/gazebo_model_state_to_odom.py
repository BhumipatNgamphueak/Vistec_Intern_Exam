#!/usr/bin/env python3
"""
Get robot state directly from Gazebo's model state
This is more reliable than TF or plugins
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState
import numpy as np


class GazeboModelStateToOdom(Node):
    def __init__(self):
        super().__init__('gazebo_model_state_to_odom')

        # Client to get model state from Gazebo
        self.get_state_client = self.create_client(
            GetEntityState,
            '/gazebo/get_entity_state'
        )

        # Wait for service
        self.get_logger().info("Waiting for /gazebo/get_entity_state service...")
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        self.get_logger().info("Connected to Gazebo entity state service")

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # State for velocity computation
        self.last_position = None
        self.last_time = None

        # Timer to query Gazebo at 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("Gazebo Model State to Odometry converter started")
        self.get_logger().info("Querying model: go2_robot")

    def timer_callback(self):
        """Query Gazebo for robot state and publish odometry."""
        # Create request
        request = GetEntityState.Request()
        request.name = 'go2_robot'  # Model name in Gazebo
        request.reference_frame = 'world'

        # Call service asynchronously
        future = self.get_state_client.call_async(request)
        future.add_done_callback(self.state_callback)

    def state_callback(self, future):
        """Process Gazebo state response."""
        try:
            response = future.result()

            if not response.success:
                # Model not found yet (still spawning)
                return

            current_time = self.get_clock().now()

            # Create odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'base'

            # Position from Gazebo
            odom.pose.pose.position = response.state.pose.position
            odom.pose.pose.orientation = response.state.pose.orientation

            # Velocity from Gazebo (ground truth!)
            odom.twist.twist.linear = response.state.twist.linear
            odom.twist.twist.angular = response.state.twist.angular

            # Publish
            self.odom_pub.publish(odom)

        except Exception as e:
            # Silently skip errors (model might not be spawned yet)
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GazeboModelStateToOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
