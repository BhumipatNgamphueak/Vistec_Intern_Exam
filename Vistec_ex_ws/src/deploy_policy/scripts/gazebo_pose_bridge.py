#!/usr/bin/env python3
"""
Bridge Gazebo pose to ROS for extracting z position.
Subscribes to Gazebo's /world/default/pose/info and publishes z position to ROS.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import subprocess
import re
import threading
import time


class GazeboPoseBridge(Node):
    def __init__(self):
        super().__init__('gazebo_pose_bridge')

        # Publisher for z position
        self.z_pub = self.create_publisher(Float64, '/robot_z_position', 10)

        # Start background thread to read Gazebo topic
        self.running = True
        self.thread = threading.Thread(target=self._read_gazebo_pose, daemon=True)
        self.thread.start()

        self.get_logger().info("Gazebo Pose Bridge Started")
        self.get_logger().info("Publishing robot z position to /robot_z_position")

    def _read_gazebo_pose(self):
        """Read Gazebo pose topic in background thread."""
        while self.running:
            try:
                # Read one message from Gazebo topic
                result = subprocess.run(
                    ['ign', 'topic', '-e', '-t', '/world/default/pose/info', '-n', '1'],
                    capture_output=True,
                    text=True,
                    timeout=1.0
                )

                if result.returncode == 0:
                    # Parse the output to find go2_robot's z position
                    output = result.stdout

                    # Find the go2_robot pose block
                    if 'name: "go2_robot"' in output:
                        # Extract z position using regex
                        z_match = re.search(r'z:\s*([-\d.]+)', output[output.find('name: "go2_robot"'):])
                        if z_match:
                            z = float(z_match.group(1))

                            # Publish z position
                            msg = Float64()
                            msg.data = z
                            self.z_pub.publish(msg)

            except subprocess.TimeoutExpired:
                pass
            except Exception as e:
                self.get_logger().warn(f"Error reading Gazebo pose: {e}", throttle_duration_sec=5.0)

            time.sleep(0.02)  # 50 Hz

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
