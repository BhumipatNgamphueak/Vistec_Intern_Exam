#!/usr/bin/env python3
"""
ROS2 Node for deploying IsaacLab trained RL policy to Gazebo for Go2 quadruped.
Subscribes to robot state and publishes joint effort commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np
from scipy.spatial.transform import Rotation
import threading

from deploy_policy.go2_policy_runner import Go2PolicyRunner
from deploy_policy.actuator_network import ActuatorNetworkRunner


class Go2PolicyDeployNode(Node):
    """ROS2 Node for deploying trained RL policy to Go2 quadruped."""

    def __init__(self):
        super().__init__("go2_policy_deploy_node")

        # Declare parameters
        self.declare_parameter("policy_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("control_frequency", 50.0)  # Hz (matches IsaacLab: dt=0.005, decimation=4)
        self.declare_parameter("use_actuator_network", True)  # Use MLP actuator network
        self.declare_parameter("actuator_model_path", "/home/drl-68/actuator_net/app/resources/actuator.pth")
        self.declare_parameter("actuator_scaler_path", "/home/drl-68/actuator_net/app/resources/scaler.pkl")

        # Get parameters
        policy_path = self.get_parameter("policy_path").value
        device = self.get_parameter("device").value
        control_freq = self.get_parameter("control_frequency").value
        use_actuator_network = self.get_parameter("use_actuator_network").value
        actuator_model_path = self.get_parameter("actuator_model_path").value
        actuator_scaler_path = self.get_parameter("actuator_scaler_path").value

        if not policy_path:
            self.get_logger().error("No policy_path specified!")
            raise ValueError("policy_path parameter is required")

        # Initialize policy runner
        self.policy_runner = Go2PolicyRunner(
            policy_path=policy_path,
            device=device,
            history_length=1,  # Go2 uses single frame (no history)
            action_scale=0.25,  # From IsaacLab ActionsCfg
        )

        # Initialize actuator network if enabled
        self.use_actuator_network = use_actuator_network
        self.actuator_network = None
        if self.use_actuator_network:
            try:
                self.actuator_network = ActuatorNetworkRunner(
                    model_path=actuator_model_path,
                    scaler_path=actuator_scaler_path,
                    num_joints=12,
                    device=device
                )
                self.get_logger().info("✓ MLP Actuator Network loaded successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to load actuator network: {e}")
                self.get_logger().warn("Falling back to simple PD control")
                self.use_actuator_network = False

        # Get joint names from policy
        self.joint_names = self.policy_runner.get_joint_names()
        self.num_joints = len(self.joint_names)

        # State variables (protected by lock)
        self.lock = threading.Lock()
        self.joint_positions = np.zeros(self.num_joints)
        self.joint_velocities = np.zeros(self.num_joints)
        self.angular_velocity = np.zeros(3)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion (x, y, z, w)
        self.joint_state_received = False
        self.imu_received = False

        # Joint name to index mapping for incoming messages
        self.joint_name_to_idx = {name: i for i, name in enumerate(self.joint_names)}

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            qos
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            qos
        )

        # Publishers - Individual joint EFFORT command publishers for Ignition Gazebo
        # Each joint has its own topic: /model/go2_robot/joint/<joint_name>/cmd_force
        self.joint_cmd_pubs = {}
        for joint_name in self.joint_names:
            topic = f"/model/go2_robot/joint/{joint_name}/cmd_force"
            self.joint_cmd_pubs[joint_name] = self.create_publisher(Float64, topic, qos)
            self.get_logger().debug(f"Created publisher for {topic}")

        # PD gains for effort control - MUST match IsaacLab training config
        # From unitree.py: GO2HV actuator with stiffness=25.0, damping=0.5
        self.kp_gains = np.full(self.num_joints, 25.0)  # Stiffness for all joints
        self.kd_gains = np.full(self.num_joints, 0.5)   # Damping for all joints

        # Control timer
        self.control_period = 1.0 / control_freq
        self.control_timer = self.create_timer(self.control_period, self.control_loop)

        # Status timer
        self.status_timer = self.create_timer(5.0, self.status_callback)

        # Get default standing positions
        self.default_positions = self.policy_runner.get_default_positions()

        # Send initial standing commands immediately and repeatedly until joint states arrive
        self.init_timer = self.create_timer(0.02, self.send_initial_pose)  # 50Hz
        self.init_sent_count = 0

        control_mode = "MLP Actuator Network" if self.use_actuator_network else "Simple PD Control"
        self.get_logger().info("="*70)
        self.get_logger().info(f"Go2 Policy Deploy Node initialized")
        self.get_logger().info(f"Policy path: {policy_path}")
        self.get_logger().info(f"Control mode: {control_mode}")
        self.get_logger().info(f"Control frequency: {control_freq} Hz")
        self.get_logger().info(f"Number of joints: {self.num_joints}")
        if not self.use_actuator_network:
            self.get_logger().info(f"PD gains: Kp={self.kp_gains[0]}, Kd={self.kd_gains[0]}")
        self.get_logger().info("="*70)
        self.get_logger().info(f"Sending initial standing pose...")

    def joint_state_callback(self, msg: JointState):
        """Handle incoming joint state messages."""
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_name_to_idx:
                    idx = self.joint_name_to_idx[name]
                    if i < len(msg.position):
                        self.joint_positions[idx] = msg.position[i]
                    if i < len(msg.velocity):
                        self.joint_velocities[idx] = msg.velocity[i]
            self.joint_state_received = True

    def imu_callback(self, msg: Imu):
        """Handle incoming IMU messages."""
        with self.lock:
            # Angular velocity
            self.angular_velocity = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])
            # Orientation quaternion
            self.orientation = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])
            self.imu_received = True

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands."""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.policy_runner.set_velocity_command(vx, vy, wz)
        self.get_logger().debug(f"Velocity command: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")

    def compute_projected_gravity(self) -> np.ndarray:
        """Compute gravity vector in robot body frame."""
        # Gravity in world frame
        gravity_world = np.array([0.0, 0.0, -1.0])

        # Convert quaternion to rotation
        # scipy uses (x, y, z, w) format
        try:
            rot = Rotation.from_quat(self.orientation)
            # Rotate gravity into body frame (inverse rotation)
            gravity_body = rot.inv().apply(gravity_world)
        except Exception as e:
            self.get_logger().warn(f"Rotation error: {e}, using default gravity")
            gravity_body = np.array([0.0, 0.0, -1.0])

        return gravity_body

    def send_initial_pose(self):
        """Send initial standing pose until joint states are received."""
        if self.joint_state_received:
            # Stop init timer once we have joint states
            self.init_timer.cancel()
            self.get_logger().info("Joint states received, switching to policy control")
            return

        # Send default standing positions
        self.publish_joint_commands(self.default_positions)
        self.init_sent_count += 1

        if self.init_sent_count % 50 == 0:  # Log every second
            self.get_logger().info(f"Sending initial pose... (count: {self.init_sent_count})")

    def control_loop(self):
        """Main control loop - runs at control_frequency."""
        # Check if we have received data
        if not self.joint_state_received:
            self.get_logger().debug("Waiting for joint states...")
            return

        with self.lock:
            joint_pos = self.joint_positions.copy()
            joint_vel = self.joint_velocities.copy()
            ang_vel = self.angular_velocity.copy()
            projected_gravity = self.compute_projected_gravity()

        # Get action from policy
        try:
            target_positions = self.policy_runner.get_action(
                joint_pos, joint_vel, ang_vel, projected_gravity
            )
        except Exception as e:
            self.get_logger().error(f"Policy inference error: {e}")
            return

        # Publish joint commands
        self.publish_joint_commands(target_positions)

    def publish_joint_commands(self, target_positions: np.ndarray):
        """Publish joint EFFORT commands using actuator network or PD control."""
        # Get current state
        with self.lock:
            current_pos = self.joint_positions.copy()
            current_vel = self.joint_velocities.copy()

        # Compute efforts based on control mode
        if self.use_actuator_network and self.actuator_network is not None:
            # Use MLP actuator network (matches training)
            efforts = self.actuator_network.compute_torques(
                target_positions, current_pos, current_vel
            )
        else:
            # Fallback to simple PD control
            position_error = target_positions - current_pos
            efforts = self.kp_gains * position_error - self.kd_gains * current_vel

        # Publish effort to each joint's command topic
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in self.joint_cmd_pubs:
                msg = Float64()
                msg.data = float(efforts[i])
                self.joint_cmd_pubs[joint_name].publish(msg)

    def status_callback(self):
        """Print status information periodically."""
        status = []
        if self.joint_state_received:
            status.append("JointState: OK")
        else:
            status.append("JointState: WAITING")

        if self.imu_received:
            status.append("IMU: OK")
        else:
            status.append("IMU: WAITING")

        self.get_logger().info(f"Status: {', '.join(status)}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Go2PolicyDeployNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
