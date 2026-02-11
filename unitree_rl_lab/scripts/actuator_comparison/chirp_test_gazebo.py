#!/usr/bin/env python3
"""
Chirp (frequency sweep) test for Gazebo motors.

Prerequisites:
- Gazebo running with Go2 robot at height
- Joint controllers loaded
- ROS 2 sourced
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json
from datetime import datetime
import time
from scipy import signal

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
except ImportError:
    print("Error: ROS 2 Python packages not found!")
    exit(1)


class ChirpTester(Node):
    """ROS 2 node for chirp testing."""

    def __init__(self, joint_name):
        super().__init__('chirp_tester')

        self.joint_name = joint_name

        # Data storage
        self.time_data = []
        self.pos_cmd_data = []
        self.pos_actual_data = []
        self.vel_actual_data = []
        self.effort_actual_data = []
        self.freq_data = []

        self.start_time = None
        self.recording = False

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/go2_position_controller/commands',
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
            try:
                self.joint_idx = msg.name.index(self.joint_name)
            except ValueError:
                return

        self.current_pos = msg.position[self.joint_idx]
        self.current_vel = msg.velocity[self.joint_idx]
        self.current_effort = msg.effort[self.joint_idx]

        if self.recording and self.start_time is not None:
            t = time.time() - self.start_time
            self.time_data.append(t)
            self.pos_actual_data.append(self.current_pos)
            self.vel_actual_data.append(self.current_vel)
            self.effort_actual_data.append(self.current_effort)

    def send_command(self, position, freq):
        """Send position command."""
        msg = Float64MultiArray()
        msg.data = [0.0] * 12
        msg.data[self.joint_idx] = float(position)
        self.cmd_pub.publish(msg)
        self.pos_cmd_data.append(position)
        self.freq_data.append(freq)

    def start_recording(self):
        """Start data recording."""
        self.recording = True
        self.start_time = time.time()
        self.time_data = []
        self.pos_cmd_data = []
        self.pos_actual_data = []
        self.vel_actual_data = []
        self.effort_actual_data = []
        self.freq_data = []

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
            "effort": self.effort_actual_data.copy(),
            "frequency": self.freq_data.copy()
        }


def run_chirp_test(tester, args):
    """Run chirp test."""

    print("\n" + "="*70)
    print("Chirp (Frequency Sweep) Test - Gazebo")
    print("="*70)
    print(f"\nJoint: {args.joint}")
    print(f"PD Gains: {'LOW (Kp=25, Kd=0.5)' if args.pd_gains == 'low' else 'HIGH (Kp=160, Kd=5)'}")
    print(f"Frequency Range: {args.f0} Hz → {args.f1} Hz")
    print(f"Duration: {args.duration} s")
    print(f"Amplitude: {args.amplitude} rad")
    print("")
    print("Starting chirp test...")
    print("="*70)

    # Default standing pose for target joint
    default_pos = [
        0.0, 0.8, -1.5,  # FL
        0.0, 0.8, -1.5,  # FR
        0.0, 1.0, -1.5,  # RL
        0.0, 1.0, -1.5,  # RR
    ]
    default_target = default_pos[tester.joint_idx]

    # Chirp parameters
    f0 = args.f0
    f1 = args.f1
    T = args.duration
    amplitude = args.amplitude

    tester.start_recording()

    start = time.time()
    rate = tester.create_rate(50)  # 50 Hz

    while (time.time() - start) < T:
        t = time.time() - start

        # Generate chirp signal
        k = (f1 - f0) / T
        instantaneous_freq = f0 + k * t
        phase = 2 * np.pi * (f0 * t + 0.5 * k * t**2)
        pos_cmd = default_target + amplitude * np.sin(phase)

        # Send command
        tester.send_command(pos_cmd, instantaneous_freq)

        # Progress update
        if int(t * 2) % 4 == 0 and int(t * 2) != int((t - 1/50) * 2):
            print(f"  Progress: {t:.1f}s / {T}s  |  Freq: {instantaneous_freq:.2f} Hz")

        rclpy.spin_once(tester, timeout_sec=0.0)
        rate.sleep()

    tester.stop_recording()

    print("\n" + "="*70)
    print("Chirp Test Complete!")
    print("="*70)

    return tester.get_data()


def main():
    parser = argparse.ArgumentParser(description="Chirp test for Gazebo motors")
    parser.add_argument("--joint", type=str, default="FL_hip_joint",
                        help="Joint to test")
    parser.add_argument("--pd_gains", type=str, required=True,
                        choices=["low", "high"],
                        help="PD gains configuration")
    parser.add_argument("--f0", type=float, default=0.1,
                        help="Start frequency (Hz)")
    parser.add_argument("--f1", type=float, default=20.0,
                        help="End frequency (Hz)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Chirp duration (seconds)")
    parser.add_argument("--amplitude", type=float, default=0.3,
                        help="Chirp amplitude (rad)")
    parser.add_argument("--output", type=str, default="chirp_analysis",
                        help="Output directory")

    args = parser.parse_args()

    # Initialize ROS
    rclpy.init()

    # Create tester
    tester = ChirpTester(args.joint)

    try:
        # Run test
        data = run_chirp_test(tester, args)

        # Add metadata
        data["platform"] = "gazebo"
        data["pd_gains"] = args.pd_gains
        data["joint_name"] = args.joint
        data["f0"] = args.f0
        data["f1"] = args.f1
        data["duration"] = args.duration
        data["amplitude"] = args.amplitude
        data["dt"] = 1.0 / 50  # 50 Hz control
        data["control_freq"] = 50

        # Create output directory
        output_dir = Path(args.output) / f"gazebo_pd_{args.pd_gains}"
        output_dir.mkdir(parents=True, exist_ok=True)

        # Save data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_file = output_dir / f"chirp_results_{timestamp}.json"
        with open(json_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"\n✅ Data saved to: {json_file}")

        # Generate plots
        print("\nGenerating plots...")
        plot_dir = output_dir / f"chirp_plots_{timestamp}"
        plot_chirp_results(data, plot_dir)

    finally:
        tester.destroy_node()
        rclpy.shutdown()

    print("\n" + "="*70)
    print("Analysis Complete!")
    print("="*70)


def plot_chirp_results(data, output_dir):
    """Generate chirp analysis plots (same as IsaacLab version)."""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    time = np.array(data["time"])
    pos_cmd = np.array(data["position_command"])
    pos_actual = np.array(data["position_actual"])
    vel_actual = np.array(data["velocity_actual"])
    effort = np.array(data["effort"])
    freq = np.array(data["frequency"])

    # Figure 1: Time domain
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    title = f'Chirp Test - Gazebo (PD {data["pd_gains"].upper()})\n{data["f0"]} Hz → {data["f1"]} Hz'
    fig.suptitle(title, fontsize=14, fontweight='bold')

    axes[0].plot(time, pos_cmd, 'k--', label='Command', linewidth=1, alpha=0.7)
    axes[0].plot(time, pos_actual, 'r-', label='Actual', linewidth=1.5)
    axes[0].set_ylabel('Position (rad)', fontsize=11)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)

    error = pos_actual - pos_cmd
    axes[1].plot(time, error * 1000, 'r-', linewidth=1.5)
    axes[1].set_ylabel('Error (mrad)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color='k', linestyle=':', alpha=0.5)
    axes[1].set_title(f'Tracking Error (RMS: {np.sqrt(np.mean(error**2))*1000:.2f} mrad)')

    axes[2].plot(time, vel_actual, 'g-', linewidth=1.5)
    axes[2].set_ylabel('Velocity (rad/s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(time, effort, 'm-', linewidth=1.5)
    axes[3].set_ylabel('Effort (Nm)', fontsize=11)
    axes[3].set_xlabel('Time (s)', fontsize=11)
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / 'chirp_time_domain.png', dpi=150)
    plt.close()

    # Figure 2: Frequency analysis
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle(f'Frequency Response - Gazebo PD {data["pd_gains"].upper()}',
                 fontsize=14, fontweight='bold')

    axes[0].plot(time, freq, 'r-', linewidth=2)
    axes[0].set_ylabel('Frequency (Hz)', fontsize=11)
    axes[0].set_xlabel('Time (s)', fontsize=11)
    axes[0].grid(True, alpha=0.3)

    scatter = axes[1].scatter(freq, np.abs(error) * 1000, c=time,
                             cmap='plasma', s=2, alpha=0.6)
    axes[1].set_xlabel('Frequency (Hz)', fontsize=11)
    axes[1].set_ylabel('|Error| (mrad)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=axes[1], label='Time (s)')

    plt.tight_layout()
    plt.savefig(output_dir / 'chirp_frequency_analysis.png', dpi=150)
    plt.close()

    print(f"✅ Plots saved to: {output_dir}")


if __name__ == "__main__":
    main()
