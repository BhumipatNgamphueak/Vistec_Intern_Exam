#!/usr/bin/env python3
"""
Compare chirp test data between Gazebo and IsaacLab.

Ensures both platforms use SAME PD gains for fair comparison.
"""

import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd


def load_chirp_data(file_path):
    """Load chirp test results from JSON."""
    with open(file_path, 'r') as f:
        return json.load(f)


def compare_chirp_responses(gazebo_data, isaaclab_data, output_dir, label=""):
    """Generate comprehensive comparison plots and metrics."""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Extract data
    time_gz = np.array(gazebo_data['time'])
    pos_cmd_gz = np.array(gazebo_data['position_command'])
    pos_act_gz = np.array(gazebo_data['position_actual'])
    vel_act_gz = np.array(gazebo_data['velocity_actual'])
    effort_gz = np.array(gazebo_data['effort'])
    freq_gz = np.array(gazebo_data['frequency'])

    time_is = np.array(isaaclab_data['time'])
    pos_cmd_is = np.array(isaaclab_data['position_command'])
    pos_act_is = np.array(isaaclab_data['position_actual'])
    vel_act_is = np.array(isaaclab_data['velocity_actual'])
    torque_is = np.array(isaaclab_data['torque'])
    freq_is = np.array(isaaclab_data['frequency'])

    # Calculate errors
    error_gz = pos_act_gz - pos_cmd_gz
    error_is = pos_act_is - pos_cmd_is

    print("\n" + "="*70)
    print(f"Chirp Data Comparison - {label}")
    print("="*70)

    # Figure 1: Time domain comparison
    fig, axes = plt.subplots(5, 1, figsize=(14, 16))
    fig.suptitle(f'Chirp Response Comparison: Gazebo vs IsaacLab\n{label}',
                 fontsize=14, fontweight='bold')

    # Position
    axes[0].plot(time_gz, pos_cmd_gz, 'k--', label='Command', linewidth=1.5, alpha=0.7)
    axes[0].plot(time_gz, pos_act_gz, 'r-', label='Gazebo', linewidth=1.5, alpha=0.8)
    axes[0].plot(time_is, pos_act_is, 'b-', label='IsaacLab', linewidth=1.5, alpha=0.8)
    axes[0].set_ylabel('Position (rad)', fontsize=11)
    axes[0].legend(fontsize=10, loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Position Response', fontsize=11)

    # Tracking error
    axes[1].plot(time_gz, error_gz * 1000, 'r-', label='Gazebo', linewidth=1.5, alpha=0.8)
    axes[1].plot(time_is, error_is * 1000, 'b-', label='IsaacLab', linewidth=1.5, alpha=0.8)
    axes[1].set_ylabel('Error (mrad)', fontsize=11)
    axes[1].legend(fontsize=10)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color='k', linestyle=':', alpha=0.5)

    rms_gz = np.sqrt(np.mean(error_gz**2)) * 1000
    rms_is = np.sqrt(np.mean(error_is**2)) * 1000
    axes[1].set_title(f'Tracking Error (RMS - Gazebo: {rms_gz:.2f} mrad, IsaacLab: {rms_is:.2f} mrad)',
                     fontsize=11)

    # Velocity
    axes[2].plot(time_gz, vel_act_gz, 'r-', label='Gazebo', linewidth=1.5, alpha=0.8)
    axes[2].plot(time_is, vel_act_is, 'b-', label='IsaacLab', linewidth=1.5, alpha=0.8)
    axes[2].set_ylabel('Velocity (rad/s)', fontsize=11)
    axes[2].legend(fontsize=10)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('Velocity Response', fontsize=11)

    # Torque/Effort
    axes[3].plot(time_gz, effort_gz, 'r-', label='Gazebo Effort', linewidth=1.5, alpha=0.8)
    axes[3].plot(time_is, torque_is, 'b-', label='IsaacLab Torque', linewidth=1.5, alpha=0.8)
    axes[3].set_ylabel('Torque/Effort (Nm)', fontsize=11)
    axes[3].legend(fontsize=10)
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title('Actuator Torque Output', fontsize=11)

    # Frequency profile
    axes[4].plot(time_gz, freq_gz, 'r-', label='Gazebo', linewidth=2, alpha=0.8)
    axes[4].plot(time_is, freq_is, 'b-', label='IsaacLab', linewidth=2, alpha=0.8)
    axes[4].set_ylabel('Frequency (Hz)', fontsize=11)
    axes[4].set_xlabel('Time (s)', fontsize=11)
    axes[4].legend(fontsize=10)
    axes[4].grid(True, alpha=0.3)
    axes[4].set_title('Frequency Sweep Profile', fontsize=11)

    plt.tight_layout()
    plt.savefig(output_dir / 'comparison_time_domain.png', dpi=150)
    plt.close()

    print(f"✅ Saved: comparison_time_domain.png")

    # Figure 2: Frequency domain comparison
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle(f'Frequency Response Comparison\n{label}',
                 fontsize=14, fontweight='bold')

    # Error vs frequency (both platforms)
    axes[0].scatter(freq_gz, np.abs(error_gz) * 1000, c='red', s=3, alpha=0.6, label='Gazebo')
    axes[0].scatter(freq_is, np.abs(error_is) * 1000, c='blue', s=3, alpha=0.6, label='IsaacLab')
    axes[0].set_xlabel('Frequency (Hz)', fontsize=11)
    axes[0].set_ylabel('|Error| (mrad)', fontsize=11)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Tracking Error vs Frequency', fontsize=11)

    # Error difference vs frequency
    # Interpolate to common frequency grid
    freq_common = np.linspace(max(freq_gz[0], freq_is[0]),
                              min(freq_gz[-1], freq_is[-1]), 100)
    error_gz_interp = np.interp(freq_common, freq_gz, np.abs(error_gz) * 1000)
    error_is_interp = np.interp(freq_common, freq_is, np.abs(error_is) * 1000)
    error_diff = error_gz_interp - error_is_interp

    axes[1].plot(freq_common, error_diff, 'purple', linewidth=2)
    axes[1].axhline(0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_xlabel('Frequency (Hz)', fontsize=11)
    axes[1].set_ylabel('Error Difference (mrad)\n(Gazebo - IsaacLab)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Error Difference vs Frequency', fontsize=11)

    plt.tight_layout()
    plt.savefig(output_dir / 'comparison_frequency_domain.png', dpi=150)
    plt.close()

    print(f"✅ Saved: comparison_frequency_domain.png")

    # Figure 3: Bandwidth comparison
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    fig.suptitle(f'Bandwidth Comparison\n{label}', fontsize=14, fontweight='bold')

    # Moving average of error vs frequency
    window_size = 20
    freq_bins = np.linspace(freq_gz[0], freq_gz[-1], 50)
    error_gz_binned = []
    error_is_binned = []

    for i in range(len(freq_bins)-1):
        mask_gz = (freq_gz >= freq_bins[i]) & (freq_gz < freq_bins[i+1])
        mask_is = (freq_is >= freq_bins[i]) & (freq_is < freq_bins[i+1])

        if np.any(mask_gz):
            error_gz_binned.append(np.mean(np.abs(error_gz[mask_gz])) * 1000)
        else:
            error_gz_binned.append(np.nan)

        if np.any(mask_is):
            error_is_binned.append(np.mean(np.abs(error_is[mask_is])) * 1000)
        else:
            error_is_binned.append(np.nan)

    freq_centers = (freq_bins[:-1] + freq_bins[1:]) / 2

    ax.plot(freq_centers, error_gz_binned, 'r-o', linewidth=2, markersize=4,
            label='Gazebo', alpha=0.8)
    ax.plot(freq_centers, error_is_binned, 'b-o', linewidth=2, markersize=4,
            label='IsaacLab', alpha=0.8)

    ax.set_xlabel('Frequency (Hz)', fontsize=12)
    ax.set_ylabel('Mean |Error| (mrad)', fontsize=12)
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.set_title('Mean Tracking Error vs Frequency (Bandwidth Analysis)', fontsize=12)

    plt.tight_layout()
    plt.savefig(output_dir / 'comparison_bandwidth.png', dpi=150)
    plt.close()

    print(f"✅ Saved: comparison_bandwidth.png")

    # Generate metrics table
    metrics = []

    # RMS errors
    metrics.append({
        "Metric": "RMS Error (mrad)",
        "Gazebo": f"{rms_gz:.2f}",
        "IsaacLab": f"{rms_is:.2f}",
        "Difference": f"{abs(rms_gz - rms_is):.2f}",
        "Diff (%)": f"{abs(rms_gz - rms_is)/((rms_gz + rms_is)/2)*100:.1f}%"
    })

    # Max errors
    max_err_gz = np.max(np.abs(error_gz)) * 1000
    max_err_is = np.max(np.abs(error_is)) * 1000
    metrics.append({
        "Metric": "Max Error (mrad)",
        "Gazebo": f"{max_err_gz:.2f}",
        "IsaacLab": f"{max_err_is:.2f}",
        "Difference": f"{abs(max_err_gz - max_err_is):.2f}",
        "Diff (%)": f"{abs(max_err_gz - max_err_is)/((max_err_gz + max_err_is)/2)*100:.1f}%"
    })

    # Max velocities
    max_vel_gz = np.max(np.abs(vel_act_gz))
    max_vel_is = np.max(np.abs(vel_act_is))
    metrics.append({
        "Metric": "Max Velocity (rad/s)",
        "Gazebo": f"{max_vel_gz:.3f}",
        "IsaacLab": f"{max_vel_is:.3f}",
        "Difference": f"{abs(max_vel_gz - max_vel_is):.3f}",
        "Diff (%)": f"{abs(max_vel_gz - max_vel_is)/((max_vel_gz + max_vel_is)/2)*100:.1f}%"
    })

    # Max torques
    max_torque_gz = np.max(np.abs(effort_gz))
    max_torque_is = np.max(np.abs(torque_is))
    metrics.append({
        "Metric": "Max Torque (Nm)",
        "Gazebo": f"{max_torque_gz:.3f}",
        "IsaacLab": f"{max_torque_is:.3f}",
        "Difference": f"{abs(max_torque_gz - max_torque_is):.3f}",
        "Diff (%)": f"{abs(max_torque_gz - max_torque_is)/((max_torque_gz + max_torque_is)/2)*100:.1f}%"
    })

    # Estimated bandwidth (frequency at 2x baseline error)
    baseline_err_gz = np.mean(np.abs(error_gz[:int(len(error_gz)*0.1)])) * 1000
    baseline_err_is = np.mean(np.abs(error_is[:int(len(error_is)*0.1)])) * 1000

    threshold_gz = 2 * baseline_err_gz
    threshold_is = 2 * baseline_err_is

    bandwidth_gz_idx = np.where(np.abs(error_gz) * 1000 > threshold_gz)[0]
    bandwidth_is_idx = np.where(np.abs(error_is) * 1000 > threshold_is)[0]

    bandwidth_gz = freq_gz[bandwidth_gz_idx[0]] if len(bandwidth_gz_idx) > 0 else freq_gz[-1]
    bandwidth_is = freq_is[bandwidth_is_idx[0]] if len(bandwidth_is_idx) > 0 else freq_is[-1]

    metrics.append({
        "Metric": "Est. Bandwidth (Hz)",
        "Gazebo": f"{bandwidth_gz:.2f}",
        "IsaacLab": f"{bandwidth_is:.2f}",
        "Difference": f"{abs(bandwidth_gz - bandwidth_is):.2f}",
        "Diff (%)": f"{abs(bandwidth_gz - bandwidth_is)/((bandwidth_gz + bandwidth_is)/2)*100:.1f}%"
    })

    # Save metrics
    df = pd.DataFrame(metrics)
    csv_file = output_dir / "comparison_metrics.csv"
    df.to_csv(csv_file, index=False)

    print(f"✅ Saved: comparison_metrics.csv")
    print("\n" + "="*70)
    print("COMPARISON METRICS")
    print("="*70)
    print(df.to_string(index=False))
    print("="*70)

    # Save summary
    summary_file = output_dir / "summary.txt"
    with open(summary_file, 'w') as f:
        f.write(f"Chirp Response Comparison Summary\n")
        f.write(f"{label}\n")
        f.write("="*70 + "\n\n")

        f.write(f"Gazebo Configuration:\n")
        f.write(f"  Platform: {gazebo_data.get('platform', 'gazebo')}\n")
        f.write(f"  PD Gains: {gazebo_data.get('pd_gains', 'unknown')}\n")
        f.write(f"  Joint: {gazebo_data.get('joint_name', 'unknown')}\n")
        f.write(f"  Frequency Range: {gazebo_data.get('f0', 0)} - {gazebo_data.get('f1', 0)} Hz\n\n")

        f.write(f"IsaacLab Configuration:\n")
        f.write(f"  Actuator: {isaaclab_data.get('actuator_type', 'unknown')}\n")
        f.write(f"  Joint: {isaaclab_data.get('joint_name', 'unknown')}\n")
        f.write(f"  Frequency Range: {isaaclab_data.get('f0', 0)} - {isaaclab_data.get('f1', 0)} Hz\n\n")

        f.write("Metrics:\n")
        f.write(df.to_string(index=False))
        f.write("\n\n")

        f.write("Interpretation:\n")
        diff_pct = float(df[df['Metric'] == 'RMS Error (mrad)']['Diff (%)'].values[0].rstrip('%'))
        if diff_pct < 10:
            f.write("  ✅ EXCELLENT MATCH (< 10% difference)\n")
        elif diff_pct < 20:
            f.write("  ✅ GOOD MATCH (10-20% difference)\n")
        elif diff_pct < 50:
            f.write("  ⚠️  MODERATE MATCH (20-50% difference)\n")
        else:
            f.write("  ❌ POOR MATCH (> 50% difference)\n")

    print(f"✅ Saved: summary.txt\n")


def main():
    parser = argparse.ArgumentParser(description="Compare Gazebo vs IsaacLab chirp data")
    parser.add_argument("--gazebo", type=str, required=True,
                        help="Gazebo chirp results JSON file")
    parser.add_argument("--isaaclab", type=str, required=True,
                        help="IsaacLab chirp results JSON file")
    parser.add_argument("--output", type=str, required=True,
                        help="Output directory for comparison results")
    parser.add_argument("--label", type=str, default="",
                        help="Label for plots (e.g., 'LOW Gains')")

    args = parser.parse_args()

    print("="*70)
    print("Gazebo ↔ IsaacLab Chirp Data Comparison")
    print("="*70)

    # Load data
    print("\nLoading data...")
    gazebo_data = load_chirp_data(args.gazebo)
    isaaclab_data = load_chirp_data(args.isaaclab)

    print(f"✅ Gazebo: {args.gazebo}")
    print(f"✅ IsaacLab: {args.isaaclab}")

    # Compare
    print("\nGenerating comparison...")
    compare_chirp_responses(gazebo_data, isaaclab_data, args.output, args.label)

    print("\n" + "="*70)
    print("Comparison Complete!")
    print("="*70)
    print(f"\nResults saved to: {args.output}")
    print("\nFiles generated:")
    print("  - comparison_time_domain.png")
    print("  - comparison_frequency_domain.png")
    print("  - comparison_bandwidth.png")
    print("  - comparison_metrics.csv")
    print("  - summary.txt")


if __name__ == "__main__":
    main()
