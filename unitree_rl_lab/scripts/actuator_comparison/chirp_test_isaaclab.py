#!/usr/bin/env python3
"""
Chirp (frequency sweep) test for IsaacLab actuators.

Tests actuator response across frequency range using chirp signal.
Useful for:
- Frequency response analysis
- Bandwidth identification
- Resonance detection
- Bode plot generation
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import torch
import json
from datetime import datetime
from scipy import signal

# IsaacLab imports (must be after AppLauncher)
from omni.isaac.lab.app import AppLauncher

def main():
    parser = argparse.ArgumentParser(description="Chirp test for IsaacLab actuators")
    parser.add_argument("--actuator", type=str, required=True,
                        choices=["mlp", "lstm", "implicit"],
                        help="Actuator type to test")
    parser.add_argument("--joint", type=str, default="FL_hip_joint",
                        help="Joint to test")
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
    parser.add_argument("--headless", action="store_true",
                        help="Run in headless mode")

    args_cli = parser.parse_args()

    # Launch IsaacSim
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    # Now import IsaacLab modules
    import omni.isaac.lab.sim as sim_utils
    from omni.isaac.lab.assets import Articulation
    from omni.isaac.lab.sim import SimulationContext

    # Import configs
    import sys
    sys.path.append(str(Path(__file__).parents[2] / "source/unitree_rl_lab"))
    from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG
    from unitree_rl_lab.assets.robots.unitree_actuators import (
        UnitreeActuatorCfg_Go2_MLP,
        UnitreeActuatorCfg_Go2_LSTM
    )
    from omni.isaac.lab.actuators import IdealPDActuatorCfg

    print("="*70)
    print("Chirp (Frequency Sweep) Test - IsaacLab")
    print("="*70)
    print(f"\nActuator: {args_cli.actuator.upper()}")
    print(f"Joint: {args_cli.joint}")
    print(f"Frequency Range: {args_cli.f0} Hz → {args_cli.f1} Hz")
    print(f"Duration: {args_cli.duration} s")
    print(f"Amplitude: {args_cli.amplitude} rad")
    print("")

    # Setup simulation
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 1.0])

    # Create robot
    robot_cfg = UNITREE_GO2_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"

    # Configure actuator
    if args_cli.actuator == "mlp":
        print("Actuator: MLP (Kp=25.0, Kd=0.5)")
        robot_cfg.actuators["legs"] = UnitreeActuatorCfg_Go2_MLP(joint_names_expr=[".*"])
    elif args_cli.actuator == "lstm":
        print("Actuator: LSTM (Kp=25.0, Kd=0.5)")
        robot_cfg.actuators["legs"] = UnitreeActuatorCfg_Go2_LSTM(joint_names_expr=[".*"])
    else:
        print("Actuator: Implicit (Kp=160.0, Kd=5.0)")
        robot_cfg.actuators["legs"] = IdealPDActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=160.0,
            damping=5.0,
        )

    robot = Articulation(robot_cfg)

    # Ground plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)

    # Play simulation
    sim.reset()

    # Hang robot at 1.5m
    hanging_height = 1.5
    robot.write_root_pose_to_sim(
        torch.tensor([[0.0, 0.0, hanging_height]], device=sim.device),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=sim.device)
    )

    # Get joint index
    joint_names = robot.data.joint_names
    if args_cli.joint not in joint_names:
        print(f"Error: Joint {args_cli.joint} not found!")
        simulation_app.close()
        return
    joint_idx = joint_names.index(args_cli.joint)

    # Test parameters
    dt = sim_cfg.dt
    control_freq = 50
    decimation = int(1.0 / (control_freq * dt))
    num_steps = int(args_cli.duration / dt)

    print(f"\nTest Parameters:")
    print(f"  Physics dt: {dt*1000:.1f} ms")
    print(f"  Control frequency: {control_freq} Hz")
    print(f"  Total steps: {num_steps}")
    print("")
    print("Starting chirp test...")
    print("="*70)

    # Data storage
    time_data = []
    pos_cmd_data = []
    pos_actual_data = []
    vel_actual_data = []
    torque_data = []
    freq_data = []

    # Initialize at standing pose
    default_pos = torch.zeros((1, robot.num_joints), device=sim.device)
    default_pos[0, :] = torch.tensor([
        0.0, 0.8, -1.5,  # FL
        0.0, 0.8, -1.5,  # FR
        0.0, 1.0, -1.5,  # RL
        0.0, 1.0, -1.5,  # RR
    ], device=sim.device)

    robot.write_joint_state_to_sim(default_pos, torch.zeros_like(default_pos))

    # Run chirp test
    f0 = args_cli.f0
    f1 = args_cli.f1
    T = args_cli.duration
    amplitude = args_cli.amplitude

    for step in range(num_steps):
        t = step * dt

        # Generate chirp signal
        k = (f1 - f0) / T
        instantaneous_freq = f0 + k * t
        phase = 2 * np.pi * (f0 * t + 0.5 * k * t**2)
        pos_cmd = default_pos[0, joint_idx].item() + amplitude * np.sin(phase)

        # Apply command
        if step % decimation == 0:
            cmd = default_pos.clone()
            cmd[0, joint_idx] = pos_cmd
            robot.set_joint_position_target(cmd)

        # Maintain hanging height
        if step % 10 == 0:
            robot.write_root_pose_to_sim(
                torch.tensor([[0.0, 0.0, hanging_height]], device=sim.device),
                torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=sim.device)
            )

        # Step simulation
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

        # Record data
        time_data.append(t)
        pos_cmd_data.append(pos_cmd)
        pos_actual_data.append(robot.data.joint_pos[0, joint_idx].item())
        vel_actual_data.append(robot.data.joint_vel[0, joint_idx].item())
        torque_data.append(robot.data.applied_torque[0, joint_idx].item())
        freq_data.append(instantaneous_freq)

        # Progress update
        if step % (control_freq * 2) == 0:
            print(f"  Progress: {t:.1f}s / {T}s  |  Current freq: {instantaneous_freq:.2f} Hz")

    # Close simulation
    simulation_app.close()

    print("\n" + "="*70)
    print("Chirp Test Complete!")
    print("="*70)

    # Create output directory
    output_dir = Path(args_cli.output) / args_cli.actuator
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save data
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results = {
        "actuator_type": args_cli.actuator,
        "joint_name": args_cli.joint,
        "f0": f0,
        "f1": f1,
        "duration": T,
        "amplitude": amplitude,
        "dt": dt,
        "control_freq": control_freq,
        "time": time_data,
        "position_command": pos_cmd_data,
        "position_actual": pos_actual_data,
        "velocity_actual": vel_actual_data,
        "torque": torque_data,
        "frequency": freq_data
    }

    json_file = output_dir / f"chirp_results_{timestamp}.json"
    with open(json_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n✅ Data saved to: {json_file}")

    # Generate plots
    print("\nGenerating plots...")
    plot_chirp_results(results, output_dir / f"chirp_plots_{timestamp}")

    print("\n" + "="*70)
    print("Analysis Complete!")
    print("="*70)
    print(f"\nResults: {output_dir}")


def plot_chirp_results(data, output_dir):
    """Generate chirp analysis plots."""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    time = np.array(data["time"])
    pos_cmd = np.array(data["position_command"])
    pos_actual = np.array(data["position_actual"])
    vel_actual = np.array(data["velocity_actual"])
    torque = np.array(data["torque"])
    freq = np.array(data["frequency"])

    # Figure 1: Time domain response
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    fig.suptitle(f'Chirp Test - {data["actuator_type"].upper()} Actuator\n'
                 f'{data["f0"]} Hz → {data["f1"]} Hz',
                 fontsize=14, fontweight='bold')

    # Position
    axes[0].plot(time, pos_cmd, 'k--', label='Command', linewidth=1, alpha=0.7)
    axes[0].plot(time, pos_actual, 'b-', label='Actual', linewidth=1.5)
    axes[0].set_ylabel('Position (rad)', fontsize=11)
    axes[0].legend(fontsize=10)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Position Response', fontsize=11)

    # Tracking error
    error = pos_actual - pos_cmd
    axes[1].plot(time, error * 1000, 'r-', linewidth=1.5)
    axes[1].set_ylabel('Error (mrad)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color='k', linestyle=':', alpha=0.5)
    axes[1].set_title(f'Tracking Error (RMS: {np.sqrt(np.mean(error**2))*1000:.2f} mrad)',
                     fontsize=11)

    # Velocity
    axes[2].plot(time, vel_actual, 'g-', linewidth=1.5)
    axes[2].set_ylabel('Velocity (rad/s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('Velocity Response', fontsize=11)

    # Torque
    axes[3].plot(time, torque, 'm-', linewidth=1.5)
    axes[3].set_ylabel('Torque (Nm)', fontsize=11)
    axes[3].set_xlabel('Time (s)', fontsize=11)
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title('Torque Output', fontsize=11)

    plt.tight_layout()
    plt.savefig(output_dir / 'chirp_time_domain.png', dpi=150)
    plt.close()

    # Figure 2: Frequency domain analysis
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle(f'Frequency Response - {data["actuator_type"].upper()}',
                 fontsize=14, fontweight='bold')

    # Instantaneous frequency vs time
    axes[0].plot(time, freq, 'b-', linewidth=2)
    axes[0].set_ylabel('Frequency (Hz)', fontsize=11)
    axes[0].set_xlabel('Time (s)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Frequency Sweep Profile', fontsize=11)

    # Error vs frequency (color-coded by time)
    scatter = axes[1].scatter(freq, np.abs(error) * 1000, c=time,
                             cmap='viridis', s=2, alpha=0.6)
    axes[1].set_xlabel('Frequency (Hz)', fontsize=11)
    axes[1].set_ylabel('|Error| (mrad)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Tracking Error vs Frequency', fontsize=11)
    cbar = plt.colorbar(scatter, ax=axes[1])
    cbar.set_label('Time (s)', fontsize=10)

    plt.tight_layout()
    plt.savefig(output_dir / 'chirp_frequency_analysis.png', dpi=150)
    plt.close()

    # Figure 3: Spectrogram
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle(f'Spectrogram Analysis - {data["actuator_type"].upper()}',
                 fontsize=14, fontweight='bold')

    dt = data["dt"]
    fs = 1.0 / dt

    # Command spectrogram
    f, t_spec, Sxx = signal.spectrogram(pos_cmd, fs, nperseg=256)
    axes[0].pcolormesh(t_spec, f, 10 * np.log10(Sxx), shading='gouraud', cmap='viridis')
    axes[0].set_ylabel('Frequency (Hz)', fontsize=11)
    axes[0].set_ylim([0, min(50, data["f1"]*2)])
    axes[0].set_title('Command Spectrogram', fontsize=11)
    axes[0].grid(True, alpha=0.3, color='white', linewidth=0.5)

    # Actual spectrogram
    f, t_spec, Sxx = signal.spectrogram(pos_actual, fs, nperseg=256)
    im = axes[1].pcolormesh(t_spec, f, 10 * np.log10(Sxx), shading='gouraud', cmap='viridis')
    axes[1].set_ylabel('Frequency (Hz)', fontsize=11)
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].set_ylim([0, min(50, data["f1"]*2)])
    axes[1].set_title('Actual Response Spectrogram', fontsize=11)
    axes[1].grid(True, alpha=0.3, color='white', linewidth=0.5)

    plt.colorbar(im, ax=axes[1], label='Power (dB)')
    plt.tight_layout()
    plt.savefig(output_dir / 'chirp_spectrogram.png', dpi=150)
    plt.close()

    print(f"✅ Plots saved to: {output_dir}")


if __name__ == "__main__":
    main()
