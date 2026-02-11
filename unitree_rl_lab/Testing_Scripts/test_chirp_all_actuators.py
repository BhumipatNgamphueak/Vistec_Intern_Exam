#!/usr/bin/env python3
"""
Test chirp motor commands with MLP, LSTM, and Implicit actuators.
Robot is suspended in air (hanging) for isolated motor testing.
Collects data for comparison with Gazebo.

Usage:
    python test_chirp_all_actuators.py --actuator mlp
    python test_chirp_all_actuators.py --actuator lstm
    python test_chirp_all_actuators.py --actuator implicit
    python test_chirp_all_actuators.py --actuator all  # Run all three sequentially
"""

import argparse
import numpy as np
import os
import torch
from datetime import datetime

from isaaclab.app import AppLauncher

# Parse arguments
parser = argparse.ArgumentParser(description="Chirp test for Go2 actuators (hanging configuration)")
parser.add_argument(
    "--actuator",
    type=str,
    default="all",
    choices=["mlp", "lstm", "implicit", "all"],
    help="Actuator type to test"
)
parser.add_argument("--duration", type=float, default=10.0, help="Chirp duration in seconds")
parser.add_argument("--f0", type=float, default=0.1, help="Starting frequency (Hz)")
parser.add_argument("--f1", type=float, default=20.0, help="Ending frequency (Hz)")
parser.add_argument("--amplitude", type=float, default=0.5, help="Chirp amplitude (radians)")
parser.add_argument("--output_dir", type=str, default="chirp_data_isaaclab", help="Output directory")
parser.add_argument("--test_joint", type=str, default="FR_hip_joint", help="Joint to test")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch Isaac Sim
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Import after launching
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext
from unitree_rl_lab.assets.robots.unitree import (
    UNITREE_GO2_MLP_CFG,
    UNITREE_GO2_LSTM_CFG,
    UNITREE_GO2_CFG,
)


class ChirpTester:
    """Chirp signal tester for Go2 robot with hanging configuration."""

    def __init__(self, robot_cfg: ArticulationCfg, actuator_name: str, test_joint: str):
        """
        Args:
            robot_cfg: Robot articulation configuration
            actuator_name: Name of actuator type (mlp, lstm, implicit)
            test_joint: Name of joint to test
        """
        self.actuator_name = actuator_name
        self.test_joint = test_joint
        self.hanging_height = 1.5  # meters above ground

        # Create simulation context
        self.sim = SimulationContext(
            sim_utils.SimulationCfg(dt=0.005, device="cuda:0", render_interval=2)
        )

        # Design scene (ground plane and lights)
        print(f"[ChirpTester] Setting up scene...")
        self._setup_scene()

        # Create robot
        robot_cfg.prim_path = "/World/Robot"
        self.robot = Articulation(robot_cfg)

        # Reset simulation to initialize robot
        print(f"[ChirpTester] Initializing simulation...")
        self.sim.reset()

        # Set robot to hanging position initially
        default_root_state = self.robot.data.default_root_state.clone()
        default_root_state[:, 2] = self.hanging_height  # Set Z position
        self.robot.write_root_pose_to_sim(default_root_state[:, :7])
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:])

        # Step simulation to apply changes
        for _ in range(10):
            self.robot.write_data_to_sim()
            self.sim.step()
            self.robot.update(self.sim.get_physics_dt())

        # Now we can access robot data
        self.joint_names = self.robot.data.joint_names
        if test_joint not in self.joint_names:
            raise ValueError(f"Joint {test_joint} not found. Available: {self.joint_names}")
        self.joint_idx = self.joint_names.index(test_joint)

        # Data storage
        self.data = {
            "time": [],
            "cmd_pos": [],
            "actual_pos": [],
            "actual_vel": [],
            "actual_torque": [],
        }

        print(f"[ChirpTester] Initialized {actuator_name} actuator")
        print(f"[ChirpTester] Testing joint: {test_joint} (index {self.joint_idx})")
        print(f"[ChirpTester] Robot hanging at height: {self.hanging_height}m")

    def _setup_scene(self):
        """Setup the simulation scene with ground plane and lighting."""
        # Ground plane
        cfg_ground = sim_utils.GroundPlaneCfg()
        cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

        # Lighting
        cfg_light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        cfg_light.func("/World/Light", cfg_light)

    def reset_robot(self):
        """Reset robot to hanging position."""
        # Play simulation
        self.sim.reset()

        # Set robot to hanging position (suspended in air)
        default_root_state = self.robot.data.default_root_state.clone()
        default_root_state[:, 2] = self.hanging_height  # Set Z position
        self.robot.write_root_pose_to_sim(default_root_state[:, :7])
        self.robot.write_root_velocity_to_sim(default_root_state[:, 7:])

        # Reset joints to default position
        default_joint_pos = self.robot.data.default_joint_pos.clone()
        default_joint_vel = self.robot.data.default_joint_vel.clone()
        self.robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)

        # Clear internal buffers
        self.robot.reset()

        # Step simulation to apply changes
        for _ in range(10):
            self.robot.write_data_to_sim()
            self.sim.step()
            self.robot.update(self.sim.get_physics_dt())

        print(f"[ChirpTester] Robot reset to hanging position at {self.hanging_height}m")

    def generate_chirp(self, duration, f0, f1, dt):
        """
        Generate chirp signal (frequency sweep).

        Args:
            duration: Signal duration (seconds)
            f0: Starting frequency (Hz)
            f1: Ending frequency (Hz)
            dt: Time step (seconds)

        Returns:
            time_array: Time points
            chirp_signal: Chirp signal values
        """
        num_steps = int(duration / dt)
        time_array = np.linspace(0, duration, num_steps)

        # Chirp parameters
        k = (f1 - f0) / duration  # Frequency sweep rate

        # Generate chirp: f(t) = f0 + k*t
        # Phase: φ(t) = 2π * (f0*t + 0.5*k*t²)
        phase = 2 * np.pi * (f0 * time_array + 0.5 * k * time_array**2)
        chirp_signal = np.sin(phase)

        return time_array, chirp_signal

    def run_chirp_test(self, duration, f0, f1, amplitude):
        """
        Run chirp test on the specified joint.

        Args:
            duration: Test duration (seconds)
            f0: Starting frequency (Hz)
            f1: Ending frequency (Hz)
            amplitude: Chirp amplitude (radians)
        """
        print(f"\n[ChirpTester] Running chirp test:")
        print(f"  Duration: {duration}s")
        print(f"  Frequency sweep: {f0} Hz → {f1} Hz")
        print(f"  Amplitude: {amplitude} rad ({np.degrees(amplitude):.1f}°)")

        # Reset robot
        self.reset_robot()

        # Get default joint position for the test joint
        default_joint_pos = self.robot.data.default_joint_pos[0, self.joint_idx].item()

        # Generate chirp signal
        dt = self.sim.get_physics_dt()
        time_array, chirp_signal = self.generate_chirp(duration, f0, f1, dt)

        # Clear data storage
        self.data = {
            "time": [],
            "cmd_pos": [],
            "actual_pos": [],
            "actual_vel": [],
            "actual_torque": [],
        }

        # Run test
        print(f"[ChirpTester] Executing chirp command...")
        for i, (t, chirp_val) in enumerate(zip(time_array, chirp_signal)):
            # Create joint position command
            joint_pos_cmd = self.robot.data.default_joint_pos.clone()
            joint_pos_cmd[0, self.joint_idx] = default_joint_pos + amplitude * chirp_val

            # Maintain hanging position (fix base in air)
            default_root_state = self.robot.data.default_root_state.clone()
            default_root_state[:, 2] = self.hanging_height
            default_root_state[:, 3:7] = torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=self.sim.device)  # Identity quaternion
            default_root_state[:, 7:] = 0.0  # Zero velocities
            self.robot.write_root_pose_to_sim(default_root_state[:, :7])
            self.robot.write_root_velocity_to_sim(default_root_state[:, 7:])

            # Apply joint command
            self.robot.set_joint_position_target(joint_pos_cmd)
            self.robot.write_data_to_sim()

            # Step simulation
            self.sim.step()

            # Update robot buffers
            self.robot.update(self.sim.get_physics_dt())

            # Record data
            actual_pos = self.robot.data.joint_pos[0, self.joint_idx].item()
            actual_vel = self.robot.data.joint_vel[0, self.joint_idx].item()

            # Get applied torque (from actuator)
            if hasattr(self.robot.data, 'applied_torque'):
                actual_torque = self.robot.data.applied_torque[0, self.joint_idx].item()
            else:
                actual_torque = 0.0  # Fallback if not available

            self.data["time"].append(t)
            self.data["cmd_pos"].append(joint_pos_cmd[0, self.joint_idx].item())
            self.data["actual_pos"].append(actual_pos)
            self.data["actual_vel"].append(actual_vel)
            self.data["actual_torque"].append(actual_torque)

            # Progress indicator
            if (i + 1) % 200 == 0:
                progress = (i + 1) / len(time_array) * 100
                print(f"  Progress: {progress:.1f}% ({i+1}/{len(time_array)} steps)")

        print(f"[ChirpTester] Chirp test complete! Collected {len(self.data['time'])} samples")

    def save_data(self, output_dir):
        """Save collected data to files."""
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"chirp_{self.actuator_name}_{self.test_joint}_{timestamp}"

        # Save as NumPy npz
        npz_path = os.path.join(output_dir, f"{filename}.npz")
        np.savez(
            npz_path,
            time=np.array(self.data["time"]),
            cmd_pos=np.array(self.data["cmd_pos"]),
            actual_pos=np.array(self.data["actual_pos"]),
            actual_vel=np.array(self.data["actual_vel"]),
            actual_torque=np.array(self.data["actual_torque"]),
            actuator=self.actuator_name,
            joint=self.test_joint,
            hanging_height=self.hanging_height,
        )
        print(f"[ChirpTester] Data saved to: {npz_path}")

        # Save as CSV for easy viewing
        csv_path = os.path.join(output_dir, f"{filename}.csv")
        with open(csv_path, 'w') as f:
            f.write("time,cmd_pos,actual_pos,actual_vel,actual_torque\n")
            for i in range(len(self.data["time"])):
                f.write(f"{self.data['time'][i]:.6f},"
                       f"{self.data['cmd_pos'][i]:.6f},"
                       f"{self.data['actual_pos'][i]:.6f},"
                       f"{self.data['actual_vel'][i]:.6f},"
                       f"{self.data['actual_torque'][i]:.6f}\n")
        print(f"[ChirpTester] CSV saved to: {csv_path}")

        return npz_path

    def cleanup(self):
        """Cleanup resources."""
        self.sim.stop()


def test_actuator(actuator_type, args):
    """Test a specific actuator type."""
    print(f"\n{'='*70}")
    print(f"Testing {actuator_type.upper()} Actuator")
    print(f"{'='*70}")

    # Select robot configuration
    if actuator_type == "mlp":
        robot_cfg = UNITREE_GO2_MLP_CFG
        print("Using MLP Actuator (Neural Network, Kp=25.0, Kd=0.5)")
    elif actuator_type == "lstm":
        robot_cfg = UNITREE_GO2_LSTM_CFG
        print("Using LSTM Actuator (Recurrent Neural Network, Kp=25.0, Kd=0.5)")
    elif actuator_type == "implicit":
        robot_cfg = UNITREE_GO2_CFG
        print("Using Implicit Actuator (Physics-based, Kp=25.0, Kd=0.5)")
    else:
        raise ValueError(f"Unknown actuator type: {actuator_type}")

    # Create tester
    tester = ChirpTester(robot_cfg, actuator_type, args.test_joint)

    # Run chirp test
    tester.run_chirp_test(args.duration, args.f0, args.f1, args.amplitude)

    # Save data
    output_path = tester.save_data(args.output_dir)

    # Cleanup
    tester.cleanup()

    return output_path


def main():
    """Main entry point."""
    print("\n" + "="*70)
    print("Go2 Chirp Motor Test - Isaac Lab")
    print("Hanging Configuration (Robot Suspended in Air)")
    print("="*70)
    print(f"\nTest Configuration:")
    print(f"  Joint: {args_cli.test_joint}")
    print(f"  Chirp: {args_cli.f0} Hz → {args_cli.f1} Hz ({args_cli.duration}s)")
    print(f"  Amplitude: {args_cli.amplitude} rad ({np.degrees(args_cli.amplitude):.1f}°)")
    print(f"  Output: {args_cli.output_dir}/")

    # Test actuators
    if args_cli.actuator == "all":
        print("\nTesting all actuator types sequentially...")
        actuator_types = ["mlp", "lstm", "implicit"]
        results = {}
        for act_type in actuator_types:
            try:
                results[act_type] = test_actuator(act_type, args_cli)
            except Exception as e:
                print(f"\n[ERROR] Failed to test {act_type}: {e}")
                results[act_type] = None

        # Summary
        print("\n" + "="*70)
        print("Test Summary")
        print("="*70)
        for act_type, path in results.items():
            status = "✓ Success" if path else "✗ Failed"
            print(f"{act_type.upper():10s}: {status}")
            if path:
                print(f"            {path}")
    else:
        test_actuator(args_cli.actuator, args_cli)

    print("\n" + "="*70)
    print("All tests complete!")
    print("="*70)

    # Close simulation app
    simulation_app.close()


if __name__ == "__main__":
    main()
