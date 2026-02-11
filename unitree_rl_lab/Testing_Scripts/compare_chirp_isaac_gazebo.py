#!/usr/bin/env python3
"""
Compare chirp test results between Isaac Lab and Gazebo.

Usage:
    python compare_chirp_isaac_gazebo.py \
        --isaac chirp_data_isaaclab/chirp_mlp_FR_hip_joint_*.npz \
        --gazebo chirp_data_gazebo/chirp_FR_hip_joint_*.npz \
        --output comparison_results
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import glob


def load_data(file_path):
    """Load chirp data from npz file."""
    data = np.load(file_path, allow_pickle=True)
    return {
        'time': data['time'],
        'cmd_pos': data['cmd_pos'],
        'actual_pos': data['actual_pos'],
        'actual_vel': data['actual_vel'],
        'actual_torque': data['actual_torque'] if 'actual_torque' in data else None,
        'actuator': str(data['actuator']) if 'actuator' in data else 'unknown',
        'joint': str(data['joint']) if 'joint' in data else 'unknown',
    }


def plot_comparison(isaac_data, gazebo_data, output_dir):
    """Plot comparison between Isaac and Gazebo data."""
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)

    actuator = isaac_data['actuator']
    joint = isaac_data['joint']

    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(f'Isaac Lab vs Gazebo - {actuator.upper()} Actuator - {joint}', fontsize=14, fontweight='bold')

    # Plot 1: Position tracking
    ax = axes[0]
    ax.plot(isaac_data['time'], isaac_data['cmd_pos'], 'k--', label='Command', linewidth=1, alpha=0.7)
    ax.plot(isaac_data['time'], isaac_data['actual_pos'], 'b-', label='Isaac Lab', linewidth=1.5)
    ax.plot(gazebo_data['time'], gazebo_data['actual_pos'], 'r-', label='Gazebo', linewidth=1.5, alpha=0.7)
    ax.set_ylabel('Position (rad)', fontsize=11)
    ax.set_title('Position Tracking', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Plot 2: Velocity
    ax = axes[1]
    ax.plot(isaac_data['time'], isaac_data['actual_vel'], 'b-', label='Isaac Lab', linewidth=1.5)
    ax.plot(gazebo_data['time'], gazebo_data['actual_vel'], 'r-', label='Gazebo', linewidth=1.5, alpha=0.7)
    ax.set_ylabel('Velocity (rad/s)', fontsize=11)
    ax.set_title('Velocity Response', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Plot 3: Torque (if available)
    ax = axes[2]
    if isaac_data['actual_torque'] is not None and gazebo_data['actual_torque'] is not None:
        ax.plot(isaac_data['time'], isaac_data['actual_torque'], 'b-', label='Isaac Lab', linewidth=1.5)
        ax.plot(gazebo_data['time'], gazebo_data['actual_torque'], 'r-', label='Gazebo', linewidth=1.5, alpha=0.7)
        ax.set_ylabel('Torque (Nm)', fontsize=11)
        ax.set_title('Applied Torque', fontsize=12, fontweight='bold')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    else:
        ax.text(0.5, 0.5, 'Torque data not available',
                horizontalalignment='center', verticalalignment='center',
                transform=ax.transAxes, fontsize=12)
        ax.set_ylabel('Torque (Nm)', fontsize=11)

    ax.set_xlabel('Time (s)', fontsize=11)

    plt.tight_layout()

    # Save figure
    output_file = output_dir / f'comparison_{actuator}_{joint}.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"[Compare] Plot saved to: {output_file}")

    plt.close()


def compute_metrics(isaac_data, gazebo_data):
    """Compute comparison metrics."""
    # Interpolate Gazebo data to Isaac time points (in case they differ)
    gazebo_pos_interp = np.interp(isaac_data['time'], gazebo_data['time'], gazebo_data['actual_pos'])
    gazebo_vel_interp = np.interp(isaac_data['time'], gazebo_data['time'], gazebo_data['actual_vel'])

    # Position tracking error
    isaac_pos_error = isaac_data['actual_pos'] - isaac_data['cmd_pos']
    gazebo_pos_error = gazebo_pos_interp - isaac_data['cmd_pos']

    # Position difference between simulators
    sim_difference = isaac_data['actual_pos'] - gazebo_pos_interp

    metrics = {
        'Isaac Tracking RMSE (rad)': np.sqrt(np.mean(isaac_pos_error**2)),
        'Isaac Tracking MAE (rad)': np.mean(np.abs(isaac_pos_error)),
        'Isaac Tracking Max Error (rad)': np.max(np.abs(isaac_pos_error)),
        'Gazebo Tracking RMSE (rad)': np.sqrt(np.mean(gazebo_pos_error**2)),
        'Gazebo Tracking MAE (rad)': np.mean(np.abs(gazebo_pos_error)),
        'Gazebo Tracking Max Error (rad)': np.max(np.abs(gazebo_pos_error)),
        'Simulator Difference RMSE (rad)': np.sqrt(np.mean(sim_difference**2)),
        'Simulator Difference MAE (rad)': np.mean(np.abs(sim_difference)),
        'Simulator Difference Max (rad)': np.max(np.abs(sim_difference)),
    }

    return metrics


def main():
    parser = argparse.ArgumentParser(description="Compare Isaac Lab and Gazebo chirp test results")
    parser.add_argument("--isaac", type=str, required=True, help="Isaac Lab data file (npz)")
    parser.add_argument("--gazebo", type=str, required=True, help="Gazebo data file (npz)")
    parser.add_argument("--output", type=str, default="comparison_results", help="Output directory")
    args = parser.parse_args()

    # Load data
    print(f"Loading Isaac Lab data: {args.isaac}")
    isaac_files = glob.glob(args.isaac)
    if not isaac_files:
        print(f"[ERROR] No Isaac Lab files found matching: {args.isaac}")
        return
    isaac_data = load_data(isaac_files[0])

    print(f"Loading Gazebo data: {args.gazebo}")
    gazebo_files = glob.glob(args.gazebo)
    if not gazebo_files:
        print(f"[ERROR] No Gazebo files found matching: {args.gazebo}")
        return
    gazebo_data = load_data(gazebo_files[0])

    print(f"\nIsaac Lab: {isaac_data['actuator']} actuator, {isaac_data['joint']}")
    print(f"  Samples: {len(isaac_data['time'])}, Duration: {isaac_data['time'][-1]:.2f}s")
    print(f"Gazebo: {gazebo_data['actuator']} actuator, {gazebo_data['joint']}")
    print(f"  Samples: {len(gazebo_data['time'])}, Duration: {gazebo_data['time'][-1]:.2f}s")

    # Compute metrics
    print("\nComputing comparison metrics...")
    metrics = compute_metrics(isaac_data, gazebo_data)

    # Print metrics
    print("\nComparison Metrics:")
    print("-" * 50)
    for key, value in metrics.items():
        print(f"  {key:.<45s} {value:.6f}")
    print("-" * 50)

    # Save metrics
    output_dir = Path(args.output)
    output_dir.mkdir(exist_ok=True)

    metrics_file = output_dir / f"metrics_{isaac_data['actuator']}_{isaac_data['joint']}.txt"
    with open(metrics_file, 'w') as f:
        f.write("Isaac Lab vs Gazebo Comparison Metrics\n")
        f.write("=" * 50 + "\n")
        f.write(f"Actuator: {isaac_data['actuator']}\n")
        f.write(f"Joint: {isaac_data['joint']}\n")
        f.write("=" * 50 + "\n\n")
        for key, value in metrics.items():
            f.write(f"{key}: {value:.6f}\n")
    print(f"\nMetrics saved to: {metrics_file}")

    # Plot comparison
    print("\nGenerating comparison plots...")
    plot_comparison(isaac_data, gazebo_data, args.output)

    print("\nComparison complete!")


if __name__ == "__main__":
    main()
