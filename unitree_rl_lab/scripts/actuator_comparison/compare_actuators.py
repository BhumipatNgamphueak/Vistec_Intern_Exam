#!/usr/bin/env python3
"""
Compare actuator responses between IsaacLab and Gazebo.

This script loads test results from both platforms and generates
comparative plots and analysis.
"""

import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd


def load_results(file_path):
    """Load test results from JSON file."""
    with open(file_path, 'r') as f:
        return json.load(f)


def compare_step_response(isaac_data, gazebo_data, output_dir):
    """Compare step response between IsaacLab and Gazebo."""

    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    fig.suptitle('Step Response Comparison: IsaacLab vs Gazebo', fontsize=16, fontweight='bold')

    # Position
    axes[0].plot(isaac_data["time"], isaac_data["position_command"],
                'k--', label='Command', linewidth=2, alpha=0.7)
    axes[0].plot(isaac_data["time"], isaac_data["position_actual"],
                'b-', label='IsaacLab', linewidth=2)
    axes[0].plot(gazebo_data["time"], gazebo_data["position_actual"],
                'r-', label='Gazebo', linewidth=2, alpha=0.8)
    axes[0].set_ylabel('Position (rad)', fontsize=12)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)

    # Add metrics annotation
    isaac_metrics = isaac_data["metrics"]
    gazebo_metrics = gazebo_data["metrics"]
    metrics_text = (
        f'IsaacLab: Rise={isaac_metrics["rise_time"]*1000:.1f}ms, '
        f'Overshoot={isaac_metrics["overshoot_percent"]:.1f}%\n'
        f'Gazebo: Rise={gazebo_metrics["rise_time"]*1000:.1f}ms, '
        f'Overshoot={gazebo_metrics["overshoot_percent"]:.1f}%'
    )
    axes[0].text(0.02, 0.98, metrics_text, transform=axes[0].transAxes,
                verticalalignment='top', fontsize=9,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Velocity
    axes[1].plot(isaac_data["time"], isaac_data["velocity_actual"],
                'b-', label='IsaacLab', linewidth=2)
    axes[1].plot(gazebo_data["time"], gazebo_data["velocity_actual"],
                'r-', label='Gazebo', linewidth=2, alpha=0.8)
    axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)

    # Torque/Effort
    axes[2].plot(isaac_data["time"], isaac_data["torque"],
                'b-', label='IsaacLab Torque', linewidth=2)
    axes[2].plot(gazebo_data["time"], gazebo_data["effort"],
                'r-', label='Gazebo Effort', linewidth=2, alpha=0.8)
    axes[2].set_ylabel('Torque/Effort (Nm)', fontsize=12)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].legend(fontsize=10)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / 'comparison_step_response.png', dpi=150)
    plt.close()

    print("✅ Step response comparison saved")


def compare_sine_tracking(isaac_data, gazebo_data, output_dir, freq_label):
    """Compare sinusoidal tracking between IsaacLab and Gazebo."""

    fig, axes = plt.subplots(4, 1, figsize=(14, 14))
    freq = isaac_data["metrics"]["frequency"]
    fig.suptitle(f'Sinusoidal Tracking Comparison ({freq} Hz): IsaacLab vs Gazebo',
                fontsize=16, fontweight='bold')

    # Position
    axes[0].plot(isaac_data["time"], isaac_data["position_command"],
                'k--', label='Command', linewidth=2, alpha=0.5)
    axes[0].plot(isaac_data["time"], isaac_data["position_actual"],
                'b-', label='IsaacLab', linewidth=2)
    axes[0].plot(gazebo_data["time"], gazebo_data["position_actual"],
                'r-', label='Gazebo', linewidth=2, alpha=0.8)
    axes[0].set_ylabel('Position (rad)', fontsize=12)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)

    # Add metrics
    isaac_metrics = isaac_data["metrics"]
    gazebo_metrics = gazebo_data["metrics"]
    metrics_text = (
        f'IsaacLab: RMSE={isaac_metrics["rmse"]*1000:.2f}mrad\n'
        f'Gazebo: RMSE={gazebo_metrics["rmse"]*1000:.2f}mrad'
    )
    axes[0].text(0.02, 0.98, metrics_text, transform=axes[0].transAxes,
                verticalalignment='top', fontsize=9,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Tracking error
    isaac_error = np.array(isaac_data["position_actual"]) - np.array(isaac_data["position_command"])
    gazebo_error = np.array(gazebo_data["position_actual"]) - np.array(gazebo_data["position_command"])
    axes[1].plot(isaac_data["time"], isaac_error * 1000, 'b-', label='IsaacLab', linewidth=2)
    axes[1].plot(gazebo_data["time"], gazebo_error * 1000, 'r-', label='Gazebo', linewidth=2, alpha=0.8)
    axes[1].set_ylabel('Error (mrad)', fontsize=12)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color='k', linestyle=':', alpha=0.5)

    # Velocity
    axes[2].plot(isaac_data["time"], isaac_data["velocity_actual"],
                'b-', label='IsaacLab', linewidth=2)
    axes[2].plot(gazebo_data["time"], gazebo_data["velocity_actual"],
                'r-', label='Gazebo', linewidth=2, alpha=0.8)
    axes[2].set_ylabel('Velocity (rad/s)', fontsize=12)
    axes[2].legend(fontsize=10)
    axes[2].grid(True, alpha=0.3)

    # Torque/Effort
    axes[3].plot(isaac_data["time"], isaac_data["torque"],
                'b-', label='IsaacLab Torque', linewidth=2)
    axes[3].plot(gazebo_data["time"], gazebo_data["effort"],
                'r-', label='Gazebo Effort', linewidth=2, alpha=0.8)
    axes[3].set_ylabel('Torque/Effort (Nm)', fontsize=12)
    axes[3].set_xlabel('Time (s)', fontsize=12)
    axes[3].legend(fontsize=10)
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / f'comparison_sine_{freq_label}.png', dpi=150)
    plt.close()

    print(f"✅ Sine tracking comparison ({freq_label}) saved")


def generate_metrics_table(isaac_results, gazebo_results, output_dir):
    """Generate comparison metrics table."""

    metrics = []

    # Step response metrics
    if "step" in isaac_results["tests"] and "step" in gazebo_results["tests"]:
        isaac_step = isaac_results["tests"]["step"]["metrics"]
        gazebo_step = gazebo_results["tests"]["step"]["metrics"]

        metrics.append({
            "Test": "Step Response",
            "Metric": "Rise Time (ms)",
            "IsaacLab": f"{isaac_step['rise_time']*1000:.1f}",
            "Gazebo": f"{gazebo_step['rise_time']*1000:.1f}",
            "Difference": f"{abs(isaac_step['rise_time'] - gazebo_step['rise_time'])*1000:.1f}"
        })

        metrics.append({
            "Test": "Step Response",
            "Metric": "Overshoot (%)",
            "IsaacLab": f"{isaac_step['overshoot_percent']:.2f}",
            "Gazebo": f"{gazebo_step['overshoot_percent']:.2f}",
            "Difference": f"{abs(isaac_step['overshoot_percent'] - gazebo_step['overshoot_percent']):.2f}"
        })

    # Sine tracking metrics
    for freq_label in ["sine_1hz", "sine_5hz"]:
        if freq_label in isaac_results["tests"] and freq_label in gazebo_results["tests"]:
            isaac_sine = isaac_results["tests"][freq_label]["metrics"]
            gazebo_sine = gazebo_results["tests"][freq_label]["metrics"]
            freq = isaac_sine["frequency"]

            metrics.append({
                "Test": f"Sine {freq} Hz",
                "Metric": "RMSE (mrad)",
                "IsaacLab": f"{isaac_sine['rmse']*1000:.2f}",
                "Gazebo": f"{gazebo_sine['rmse']*1000:.2f}",
                "Difference": f"{abs(isaac_sine['rmse'] - gazebo_sine['rmse'])*1000:.2f}"
            })

            metrics.append({
                "Test": f"Sine {freq} Hz",
                "Metric": "Max Error (mrad)",
                "IsaacLab": f"{isaac_sine['max_error']*1000:.2f}",
                "Gazebo": f"{gazebo_sine['max_error']*1000:.2f}",
                "Difference": f"{abs(isaac_sine['max_error'] - gazebo_sine['max_error'])*1000:.2f}"
            })

    # Create DataFrame
    df = pd.DataFrame(metrics)

    # Save as CSV
    csv_file = output_dir / "comparison_metrics.csv"
    df.to_csv(csv_file, index=False)
    print(f"✅ Metrics table saved to: {csv_file}")

    # Print table
    print("\n" + "="*80)
    print("COMPARISON METRICS")
    print("="*80)
    print(df.to_string(index=False))
    print("="*80)

    return df


def main():
    parser = argparse.ArgumentParser(description="Compare IsaacLab and Gazebo actuator responses")
    parser.add_argument("--isaac", type=str, required=True,
                        help="IsaacLab test results JSON file")
    parser.add_argument("--gazebo", type=str, required=True,
                        help="Gazebo test results JSON file")
    parser.add_argument("--output", type=str, default="actuator_analysis/comparison",
                        help="Output directory for comparison plots")

    args = parser.parse_args()

    print("="*60)
    print("Actuator Response Comparison")
    print("="*60)

    # Load results
    print("\nLoading results...")
    isaac_results = load_results(args.isaac)
    gazebo_results = load_results(args.gazebo)

    print(f"✅ IsaacLab: {isaac_results.get('actuator_type', 'unknown')}")
    print(f"✅ Gazebo: PD gains = {gazebo_results.get('pd_gains', 'unknown')}")

    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate comparisons
    print("\nGenerating comparisons...")

    if "step" in isaac_results["tests"] and "step" in gazebo_results["tests"]:
        compare_step_response(
            isaac_results["tests"]["step"],
            gazebo_results["tests"]["step"],
            output_dir
        )

    if "sine_1hz" in isaac_results["tests"] and "sine_1hz" in gazebo_results["tests"]:
        compare_sine_tracking(
            isaac_results["tests"]["sine_1hz"],
            gazebo_results["tests"]["sine_1hz"],
            output_dir,
            "1hz"
        )

    if "sine_5hz" in isaac_results["tests"] and "sine_5hz" in gazebo_results["tests"]:
        compare_sine_tracking(
            isaac_results["tests"]["sine_5hz"],
            gazebo_results["tests"]["sine_5hz"],
            output_dir,
            "5hz"
        )

    # Generate metrics table
    generate_metrics_table(isaac_results, gazebo_results, output_dir)

    print("\n" + "="*60)
    print("Comparison Complete!")
    print("="*60)
    print(f"\nResults saved to: {output_dir}")


if __name__ == "__main__":
    main()
