#!/usr/bin/env python3
"""
Test and characterize Gazebo motor responses (effort controller).

This script applies step inputs and sinusoidal commands to measure:
- Step response characteristics
- Frequency response
- Torque output
- Position tracking accuracy

Requires:
- ROS 2
- Gazebo with Go2 robot loaded
- Joint controllers running
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json
from datetime import datetime
import time

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError:
    print("Error: ROS 2 Python packages not found!")
    print("Make sure you've sourced ROS 2: source /opt/ros/humble/setup.bash")
    exit(1)


class MotorTester(Node):
    """ROS 2 node for testing Gazebo motor response."""

    def __init__(self, joint_name, controller_type="position"):
        super().__init__('motor_tester')

        self.joint_name = joint_name
        self.controller_type = controller_type

        # Data storage
        self.time_data = []
        self.pos_cmd_data = []
        self.pos_actual_data = []
        self.vel_actual_data = []
        self.effort_actual_data = []

        self.start_time = None
        self.recording = False

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for position commands
        if controller_type == "position":
            self.cmd_pub = self.create_publisher(
                Float64MultiArray,
                '/go2_position_controller/commands',
                10
            )
        elif controller_type == "effort":
            self.cmd_pub = self.create_publisher(
                Float64MultiArray,
                '/go2_effort_controller/commands',
                10
            )

        # Get joint index
        self.joint_idx = None
        self.current_pos = None
        self.current_vel = None
        self.current_effort = None

        # Wait for first joint state
        print("Waiting for joint states...")
        while self.joint_idx is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        print(f"✅ Connected to joint: {joint_name} (index: {self.joint_idx})")

    def joint_state_callback(self, msg):
        """Process joint state messages."""
        if self.joint_idx is None:
            # Find joint index
            try:
                self.joint_idx = msg.name.index(self.joint_name)
            except ValueError:
                return

        # Store current state
        self.current_pos = msg.position[self.joint_idx]
        self.current_vel = msg.velocity[self.joint_idx]
        self.current_effort = msg.effort[self.joint_idx]

        # Record data if active
        if self.recording and self.start_time is not None:
            t = time.time() - self.start_time
            self.time_data.append(t)
            self.pos_actual_data.append(self.current_pos)
            self.vel_actual_data.append(self.current_vel)
            self.effort_actual_data.append(self.current_effort)

    def send_position_command(self, position):
        """Send position command to joint."""
        msg = Float64MultiArray()
        # Assuming 12 joints, set all to current position except target
        if self.current_pos is not None:
            msg.data = [0.0] * 12  # All joints
            msg.data[self.joint_idx] = float(position)
            self.cmd_pub.publish(msg)
            self.pos_cmd_data.append(position)

    def start_recording(self):
        """Start data recording."""
        self.recording = True
        self.start_time = time.time()
        self.time_data = []
        self.pos_cmd_data = []
        self.pos_actual_data = []
        self.vel_actual_data = []
        self.effort_actual_data = []

    def stop_recording(self):
        """Stop data recording."""
        self.recording = False

    def get_data(self):
        """Return recorded data."""
        return {
            "time": self.time_data.copy(),
            "position_command": self.pos_cmd_data.copy(),
            "position_actual": self.pos_actual_data.copy(),
            "velocity_actual": self.vel_actual_data.copy(),
            "effort": self.effort_actual_data.copy()
        }

    def clear_data(self):
        """Clear recorded data."""
        self.time_data = []
        self.pos_cmd_data = []
        self.pos_actual_data = []
        self.vel_actual_data = []
        self.effort_actual_data = []


def run_step_test(tester, step_size=0.5, duration=5.0, step_time=1.0):
    """Run step response test."""

    print(f"\n  Step size: {step_size} rad at t={step_time}s")
    print(f"  Duration: {duration}s")

    tester.clear_data()
    tester.start_recording()

    start = time.time()
    rate = tester.create_rate(50)  # 50 Hz

    while (time.time() - start) < duration:
        t = time.time() - start

        # Step command
        if t < step_time:
            pos_cmd = 0.0
        else:
            pos_cmd = step_size

        tester.send_position_command(pos_cmd)
        rclpy.spin_once(tester, timeout_sec=0.0)
        rate.sleep()

    tester.stop_recording()

    # Analyze
    data = tester.get_data()
    time_array = np.array(data["time"])
    pos_actual = np.array(data["position_actual"])

    # Find step start
    step_idx = np.argmax(time_array > step_time)

    # Rise time
    settled_value = step_size
    idx_10 = np.argmax(pos_actual > 0.1 * settled_value)
    idx_90 = np.argmax(pos_actual > 0.9 * settled_value)
    rise_time = time_array[idx_90] - time_array[idx_10] if idx_90 > idx_10 else 0

    # Overshoot
    max_value = np.max(pos_actual[step_idx:])
    overshoot = ((max_value - settled_value) / settled_value) * 100 if settled_value > 0 else 0

    # Settling time (2%)
    tolerance = 0.02 * abs(settled_value)
    settled_idx = None
    for i in range(step_idx, len(pos_actual) - 10):
        if np.all(np.abs(pos_actual[i:i+10] - settled_value) < tolerance):
            settled_idx = i
            break
    settling_time = (time_array[settled_idx] - time_array[step_idx]) if settled_idx else None

    print(f"  ✅ Rise time: {rise_time*1000:.1f} ms")
    print(f"  ✅ Overshoot: {overshoot:.2f}%")
    if settling_time:
        print(f"  ✅ Settling time: {settling_time*1000:.1f} ms")

    data["metrics"] = {
        "step_size": step_size,
        "rise_time": rise_time,
        "overshoot_percent": overshoot,
        "settling_time": settling_time
    }

    return data


def run_sine_test(tester, freq=1.0, amplitude=0.3, duration=5.0):
    """Run sinusoidal tracking test."""

    print(f"\n  Frequency: {freq} Hz")
    print(f"  Amplitude: {amplitude} rad")
    print(f"  Duration: {duration}s")

    tester.clear_data()
    tester.start_recording()

    start = time.time()
    rate = tester.create_rate(50)  # 50 Hz

    while (time.time() - start) < duration:
        t = time.time() - start

        # Sinusoidal command
        pos_cmd = amplitude * np.sin(2 * np.pi * freq * t)

        tester.send_position_command(pos_cmd)
        rclpy.spin_once(tester, timeout_sec=0.0)
        rate.sleep()

    tester.stop_recording()

    # Analyze
    data = tester.get_data()
    pos_cmd = np.array(data["position_command"])
    pos_actual = np.array(data["position_actual"])
    error = pos_actual - pos_cmd

    rmse = np.sqrt(np.mean(error**2))
    max_error = np.max(np.abs(error))

    print(f"  ✅ RMSE: {rmse*1000:.2f} mrad")
    print(f"  ✅ Max error: {max_error*1000:.2f} mrad")

    data["metrics"] = {
        "frequency": freq,
        "amplitude": amplitude,
        "rmse": rmse,
        "max_error": max_error
    }

    return data


def run_chirp_test(tester, f0=0.5, f1=10.0, amplitude=0.2, duration=5.0):
    """Run frequency sweep test."""

    print(f"\n  Frequency sweep: {f0} Hz → {f1} Hz")
    print(f"  Amplitude: {amplitude} rad")
    print(f"  Duration: {duration}s")

    tester.clear_data()
    tester.start_recording()

    start = time.time()
    rate = tester.create_rate(50)  # 50 Hz

    while (time.time() - start) < duration:
        t = time.time() - start

        # Chirp signal
        k = (f1 - f0) / duration
        phase = 2 * np.pi * (f0 * t + 0.5 * k * t**2)
        pos_cmd = amplitude * np.sin(phase)

        tester.send_position_command(pos_cmd)
        rclpy.spin_once(tester, timeout_sec=0.0)
        rate.sleep()

    tester.stop_recording()

    data = tester.get_data()
    data["metrics"] = {
        "f0": f0,
        "f1": f1,
        "amplitude": amplitude,
        "duration": duration
    }

    return data


def main():
    parser = argparse.ArgumentParser(description="Test Gazebo motor responses")
    parser.add_argument("--joint", type=str, default="FL_hip_joint",
                        help="Joint to test (default: FL_hip_joint)")
    parser.add_argument("--pd_gains", type=str, required=True,
                        choices=["low", "high"],
                        help="PD gains: low (25/0.5 for MLP/LSTM) or high (160/5 for Implicit)")
    parser.add_argument("--test", type=str, default="all",
                        choices=["step", "sine", "chirp", "all"],
                        help="Test type to run")
    parser.add_argument("--output", type=str, default="actuator_analysis/gazebo",
                        help="Output directory")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Test duration in seconds")

    args = parser.parse_args()

    # Initialize ROS
    rclpy.init()

    print("="*60)
    print(f"Gazebo Motor Testing")
    print("="*60)
    print(f"\nPD Gains: {'Low (Kp=25, Kd=0.5)' if args.pd_gains == 'low' else 'High (Kp=160, Kd=5)'}")
    print(f"Joint: {args.joint}")

    # Create output directory
    output_dir = Path(args.output) / f"pd_{args.pd_gains}"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Create tester node
    tester = MotorTester(args.joint)

    # Results storage
    results = {
        "platform": "gazebo",
        "pd_gains": args.pd_gains,
        "joint_name": args.joint,
        "tests": {}
    }

    try:
        # Run tests
        if args.test in ["step", "all"]:
            print("\n" + "="*60)
            print("Test 1: Step Response")
            print("="*60)
            results["tests"]["step"] = run_step_test(tester, duration=args.duration)

        if args.test in ["sine", "all"]:
            print("\n" + "="*60)
            print("Test 2: Sinusoidal Tracking (1 Hz)")
            print("="*60)
            results["tests"]["sine_1hz"] = run_sine_test(tester, freq=1.0, duration=args.duration)

            print("\n" + "="*60)
            print("Test 3: Sinusoidal Tracking (5 Hz)")
            print("="*60)
            results["tests"]["sine_5hz"] = run_sine_test(tester, freq=5.0, duration=args.duration)

        if args.test in ["chirp", "all"]:
            print("\n" + "="*60)
            print("Test 4: Frequency Sweep (Chirp)")
            print("="*60)
            results["tests"]["chirp"] = run_chirp_test(tester, duration=args.duration)

        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_file = output_dir / f"test_results_{timestamp}.json"
        with open(json_file, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n✅ Results saved to: {json_file}")

        # Generate plots
        from test_isaaclab_actuators import plot_step_response, plot_sine_tracking, plot_chirp_response
        print("\nGenerating plots...")
        plot_dir = output_dir / f"plots_{timestamp}"
        plot_dir.mkdir(parents=True, exist_ok=True)

        if "step" in results["tests"]:
            plot_step_response(results["tests"]["step"], f"gazebo_pd_{args.pd_gains}", plot_dir)
        if "sine_1hz" in results["tests"]:
            plot_sine_tracking(results["tests"]["sine_1hz"], f"gazebo_pd_{args.pd_gains}", plot_dir, "1hz")
        if "sine_5hz" in results["tests"]:
            plot_sine_tracking(results["tests"]["sine_5hz"], f"gazebo_pd_{args.pd_gains}", plot_dir, "5hz")
        if "chirp" in results["tests"]:
            plot_chirp_response(results["tests"]["chirp"], f"gazebo_pd_{args.pd_gains}", plot_dir)

        print(f"✅ Plots saved to: {plot_dir}")

    finally:
        tester.destroy_node()
        rclpy.shutdown()

    print("\n" + "="*60)
    print("Testing Complete!")
    print("="*60)


if __name__ == "__main__":
    main()
