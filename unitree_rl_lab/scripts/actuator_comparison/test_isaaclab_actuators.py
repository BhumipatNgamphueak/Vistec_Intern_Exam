#!/usr/bin/env python3
"""
Test and characterize IsaacLab actuator responses (MLP, LSTM, Implicit).

This script applies step inputs and sinusoidal commands to measure:
- Step response characteristics (rise time, overshoot, settling time)
- Frequency response
- Torque output
- Position tracking accuracy
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import torch
import yaml
import json
from datetime import datetime

# IsaacLab imports (must be after AppLauncher)
from omni.isaac.lab.app import AppLauncher

def main():
    parser = argparse.ArgumentParser(description="Test IsaacLab actuator responses")
    parser.add_argument("--actuator", type=str, required=True,
                        choices=["mlp", "lstm", "implicit"],
                        help="Actuator type to test")
    parser.add_argument("--joint", type=str, default="FL_hip_joint",
                        help="Joint to test (default: FL_hip_joint)")
    parser.add_argument("--test", type=str, default="all",
                        choices=["step", "sine", "chirp", "all"],
                        help="Test type to run")
    parser.add_argument("--output", type=str, default="actuator_analysis/isaaclab",
                        help="Output directory")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Test duration in seconds")
    parser.add_argument("--headless", action="store_true",
                        help="Run in headless mode")

    # Parse args
    args_cli = parser.parse_args()

    # Launch IsaacSim
    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    # Now import IsaacLab modules
    import omni.isaac.lab.sim as sim_utils
    from omni.isaac.lab.assets import Articulation, ArticulationCfg
    from omni.isaac.lab.sim import SimulationContext

    # Import actuator configs
    import sys
    sys.path.append(str(Path(__file__).parents[2] / "source/unitree_rl_lab"))
    from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG
    from unitree_rl_lab.assets.robots.unitree_actuators import (
        UnitreeActuatorCfg_Go2_MLP,
        UnitreeActuatorCfg_Go2_LSTM
    )
    from omni.isaac.lab.actuators import IdealPDActuatorCfg

    # Create output directory
    output_dir = Path(args_cli.output) / args_cli.actuator
    output_dir.mkdir(parents=True, exist_ok=True)

    print("="*60)
    print(f"IsaacLab Actuator Testing: {args_cli.actuator.upper()}")
    print("="*60)

    # Setup simulation
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.5])

    # Create robot with specific actuator
    robot_cfg = UNITREE_GO2_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"

    # Configure actuator based on type
    if args_cli.actuator == "mlp":
        print("\nActuator Configuration: MLP (Neural Network)")
        print("  Kp = 25.0")
        print("  Kd = 0.5")
        robot_cfg.actuators["legs"] = UnitreeActuatorCfg_Go2_MLP(
            joint_names_expr=[".*"],
        )
    elif args_cli.actuator == "lstm":
        print("\nActuator Configuration: LSTM (Neural Network)")
        print("  Kp = 25.0")
        print("  Kd = 0.5")
        robot_cfg.actuators["legs"] = UnitreeActuatorCfg_Go2_LSTM(
            joint_names_expr=[".*"],
        )
    else:  # implicit
        print("\nActuator Configuration: Implicit (IdealPD)")
        print("  Kp = 160.0")
        print("  Kd = 5.0")
        robot_cfg.actuators["legs"] = IdealPDActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=160.0,
            damping=5.0,
        )

    # Create robot
    robot = Articulation(robot_cfg)

    # Define ground plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)

    # Play simulation
    sim.reset()

    # Get joint index
    joint_names = robot.data.joint_names
    if args_cli.joint not in joint_names:
        print(f"Error: Joint {args_cli.joint} not found!")
        print(f"Available joints: {joint_names}")
        simulation_app.close()
        return

    joint_idx = joint_names.index(args_cli.joint)
    print(f"\nTesting joint: {args_cli.joint} (index: {joint_idx})")

    # Initialize robot
    robot.write_root_pose_to_sim(
        torch.tensor([[0.0, 0.0, 0.4]], device=sim.device),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=sim.device)
    )

    # Test parameters
    dt = sim_cfg.dt
    control_freq = 50  # Hz
    decimation = int(1.0 / (control_freq * dt))
    num_steps = int(args_cli.duration / dt)

    print(f"\nTest Parameters:")
    print(f"  Physics dt: {dt*1000:.1f} ms")
    print(f"  Control frequency: {control_freq} Hz")
    print(f"  Test duration: {args_cli.duration} s")
    print(f"  Total steps: {num_steps}")

    # Data storage
    results = {
        "actuator_type": args_cli.actuator,
        "joint_name": args_cli.joint,
        "joint_index": joint_idx,
        "dt": dt,
        "control_freq": control_freq,
        "tests": {}
    }

    # Run tests
    if args_cli.test in ["step", "all"]:
        print("\n" + "="*60)
        print("Test 1: Step Response")
        print("="*60)
        results["tests"]["step"] = run_step_test(
            robot, sim, joint_idx, num_steps, decimation, dt
        )

    if args_cli.test in ["sine", "all"]:
        print("\n" + "="*60)
        print("Test 2: Sinusoidal Tracking (1 Hz)")
        print("="*60)
        results["tests"]["sine_1hz"] = run_sine_test(
            robot, sim, joint_idx, num_steps, decimation, dt, freq=1.0
        )

        print("\n" + "="*60)
        print("Test 3: Sinusoidal Tracking (5 Hz)")
        print("="*60)
        results["tests"]["sine_5hz"] = run_sine_test(
            robot, sim, joint_idx, num_steps, decimation, dt, freq=5.0
        )

    if args_cli.test in ["chirp", "all"]:
        print("\n" + "="*60)
        print("Test 4: Frequency Sweep (Chirp)")
        print("="*60)
        results["tests"]["chirp"] = run_chirp_test(
            robot, sim, joint_idx, num_steps, decimation, dt
        )

    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Save JSON
    json_file = output_dir / f"test_results_{timestamp}.json"
    with open(json_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n✅ Results saved to: {json_file}")

    # Generate plots
    print("\nGenerating plots...")
    plot_results(results, output_dir / f"plots_{timestamp}")

    # Close simulation
    simulation_app.close()

    print("\n" + "="*60)
    print("Testing Complete!")
    print("="*60)


def run_step_test(robot, sim, joint_idx, num_steps, decimation, dt):
    """Run step response test."""

    # Initial position: 0.0 rad
    # Step to: 0.5 rad at t=1.0s

    step_time = 1.0  # seconds
    step_size = 0.5  # radians
    step_step = int(step_time / dt)

    time_data = []
    pos_cmd_data = []
    pos_actual_data = []
    vel_actual_data = []
    torque_data = []

    # Reset robot to neutral position
    default_pos = torch.zeros((1, robot.num_joints), device=sim.device)
    robot.write_joint_state_to_sim(default_pos, torch.zeros_like(default_pos))
    sim.reset()

    for step in range(num_steps):
        # Generate step command
        if step < step_step:
            pos_cmd = 0.0
        else:
            pos_cmd = step_size

        # Apply command every decimation steps
        if step % decimation == 0:
            cmd = default_pos.clone()
            cmd[0, joint_idx] = pos_cmd
            robot.set_joint_position_target(cmd)

        # Step simulation
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)

        # Record data
        time_data.append(step * dt)
        pos_cmd_data.append(pos_cmd)
        pos_actual_data.append(robot.data.joint_pos[0, joint_idx].item())
        vel_actual_data.append(robot.data.joint_vel[0, joint_idx].item())
        torque_data.append(robot.data.applied_torque[0, joint_idx].item())

    # Analyze step response
    time_array = np.array(time_data)
    pos_actual = np.array(pos_actual_data)

    # Find rise time (10% to 90%)
    settled_value = step_size
    idx_10 = np.argmax(pos_actual > 0.1 * settled_value)
    idx_90 = np.argmax(pos_actual > 0.9 * settled_value)
    rise_time = time_array[idx_90] - time_array[idx_10]

    # Find overshoot
    max_value = np.max(pos_actual[step_step:])
    overshoot = ((max_value - settled_value) / settled_value) * 100

    # Find settling time (within 2%)
    tolerance = 0.02 * settled_value
    settled_idx = None
    for i in range(step_step, len(pos_actual)):
        if np.all(np.abs(pos_actual[i:] - settled_value) < tolerance):
            settled_idx = i
            break
    settling_time = (time_array[settled_idx] - time_array[step_step]) if settled_idx else None

    print(f"  Rise time (10%-90%): {rise_time*1000:.1f} ms")
    print(f"  Overshoot: {overshoot:.2f}%")
    if settling_time:
        print(f"  Settling time (2%): {settling_time*1000:.1f} ms")

    return {
        "time": time_data,
        "position_command": pos_cmd_data,
        "position_actual": pos_actual_data,
        "velocity_actual": vel_actual_data,
        "torque": torque_data,
        "metrics": {
            "step_size": step_size,
            "rise_time": rise_time,
            "overshoot_percent": overshoot,
            "settling_time": settling_time
        }
    }


def run_sine_test(robot, sim, joint_idx, num_steps, decimation, dt, freq=1.0):
    """Run sinusoidal tracking test."""

    amplitude = 0.3  # radians

    time_data = []
    pos_cmd_data = []
    pos_actual_data = []
    vel_actual_data = []
    torque_data = []

    # Reset robot
    default_pos = torch.zeros((1, robot.num_joints), device=sim.device)
    robot.write_joint_state_to_sim(default_pos, torch.zeros_like(default_pos))
    sim.reset()

    for step in range(num_steps):
        t = step * dt

        # Generate sinusoidal command
        pos_cmd = amplitude * np.sin(2 * np.pi * freq * t)

        # Apply command
        if step % decimation == 0:
            cmd = default_pos.clone()
            cmd[0, joint_idx] = pos_cmd
            robot.set_joint_position_target(cmd)

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

    # Analyze tracking
    pos_cmd = np.array(pos_cmd_data)
    pos_actual = np.array(pos_actual_data)
    error = pos_actual - pos_cmd

    rmse = np.sqrt(np.mean(error**2))
    max_error = np.max(np.abs(error))

    # Phase lag (cross-correlation)
    correlation = np.correlate(pos_cmd, pos_actual, mode='full')
    lag = np.argmax(correlation) - len(pos_cmd) + 1
    phase_lag = lag * dt

    print(f"  Frequency: {freq} Hz")
    print(f"  RMSE: {rmse*1000:.2f} mrad")
    print(f"  Max error: {max_error*1000:.2f} mrad")
    print(f"  Phase lag: {phase_lag*1000:.1f} ms")

    return {
        "time": time_data,
        "position_command": pos_cmd_data,
        "position_actual": pos_actual_data,
        "velocity_actual": vel_actual_data,
        "torque": torque_data,
        "metrics": {
            "frequency": freq,
            "amplitude": amplitude,
            "rmse": rmse,
            "max_error": max_error,
            "phase_lag": phase_lag
        }
    }


def run_chirp_test(robot, sim, joint_idx, num_steps, decimation, dt):
    """Run frequency sweep (chirp) test."""

    amplitude = 0.2  # radians
    f0 = 0.5  # Start frequency (Hz)
    f1 = 10.0  # End frequency (Hz)
    duration = num_steps * dt

    time_data = []
    pos_cmd_data = []
    pos_actual_data = []
    vel_actual_data = []
    torque_data = []

    # Reset robot
    default_pos = torch.zeros((1, robot.num_joints), device=sim.device)
    robot.write_joint_state_to_sim(default_pos, torch.zeros_like(default_pos))
    sim.reset()

    for step in range(num_steps):
        t = step * dt

        # Generate chirp signal
        k = (f1 - f0) / duration
        instantaneous_freq = f0 + k * t
        phase = 2 * np.pi * (f0 * t + 0.5 * k * t**2)
        pos_cmd = amplitude * np.sin(phase)

        # Apply command
        if step % decimation == 0:
            cmd = default_pos.clone()
            cmd[0, joint_idx] = pos_cmd
            robot.set_joint_position_target(cmd)

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

    print(f"  Frequency sweep: {f0} Hz → {f1} Hz")
    print(f"  Duration: {duration} s")

    return {
        "time": time_data,
        "position_command": pos_cmd_data,
        "position_actual": pos_actual_data,
        "velocity_actual": vel_actual_data,
        "torque": torque_data,
        "metrics": {
            "amplitude": amplitude,
            "f0": f0,
            "f1": f1,
            "duration": duration
        }
    }


def plot_results(results, output_dir):
    """Generate plots from test results."""

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    actuator_type = results["actuator_type"]

    # Plot step response
    if "step" in results["tests"]:
        plot_step_response(results["tests"]["step"], actuator_type, output_dir)

    # Plot sine tracking
    if "sine_1hz" in results["tests"]:
        plot_sine_tracking(results["tests"]["sine_1hz"], actuator_type, output_dir, "1hz")
    if "sine_5hz" in results["tests"]:
        plot_sine_tracking(results["tests"]["sine_5hz"], actuator_type, output_dir, "5hz")

    # Plot chirp response
    if "chirp" in results["tests"]:
        plot_chirp_response(results["tests"]["chirp"], actuator_type, output_dir)

    print(f"✅ Plots saved to: {output_dir}")


def plot_step_response(data, actuator_type, output_dir):
    """Plot step response."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(f'Step Response - {actuator_type.upper()} Actuator', fontsize=14, fontweight='bold')

    time = np.array(data["time"])

    # Position
    axes[0].plot(time, data["position_command"], 'k--', label='Command', linewidth=2)
    axes[0].plot(time, data["position_actual"], 'b-', label='Actual', linewidth=1.5)
    axes[0].set_ylabel('Position (rad)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'Rise Time: {data["metrics"]["rise_time"]*1000:.1f} ms, '
                     f'Overshoot: {data["metrics"]["overshoot_percent"]:.2f}%')

    # Velocity
    axes[1].plot(time, data["velocity_actual"], 'g-', linewidth=1.5)
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].grid(True, alpha=0.3)

    # Torque
    axes[2].plot(time, data["torque"], 'r-', linewidth=1.5)
    axes[2].set_ylabel('Torque (Nm)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / f'step_response_{actuator_type}.png', dpi=150)
    plt.close()


def plot_sine_tracking(data, actuator_type, output_dir, suffix):
    """Plot sinusoidal tracking."""

    fig, axes = plt.subplots(4, 1, figsize=(12, 12))
    freq = data["metrics"]["frequency"]
    fig.suptitle(f'Sinusoidal Tracking ({freq} Hz) - {actuator_type.upper()} Actuator',
                 fontsize=14, fontweight='bold')

    time = np.array(data["time"])

    # Position
    axes[0].plot(time, data["position_command"], 'k--', label='Command', linewidth=2)
    axes[0].plot(time, data["position_actual"], 'b-', label='Actual', linewidth=1.5)
    axes[0].set_ylabel('Position (rad)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'RMSE: {data["metrics"]["rmse"]*1000:.2f} mrad, '
                     f'Phase Lag: {data["metrics"]["phase_lag"]*1000:.1f} ms')

    # Tracking error
    error = np.array(data["position_actual"]) - np.array(data["position_command"])
    axes[1].plot(time, error * 1000, 'r-', linewidth=1.5)
    axes[1].set_ylabel('Error (mrad)')
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(0, color='k', linestyle=':', alpha=0.5)

    # Velocity
    axes[2].plot(time, data["velocity_actual"], 'g-', linewidth=1.5)
    axes[2].set_ylabel('Velocity (rad/s)')
    axes[2].grid(True, alpha=0.3)

    # Torque
    axes[3].plot(time, data["torque"], 'm-', linewidth=1.5)
    axes[3].set_ylabel('Torque (Nm)')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / f'sine_tracking_{suffix}_{actuator_type}.png', dpi=150)
    plt.close()


def plot_chirp_response(data, actuator_type, output_dir):
    """Plot chirp (frequency sweep) response."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(f'Frequency Sweep - {actuator_type.upper()} Actuator',
                 fontsize=14, fontweight='bold')

    time = np.array(data["time"])

    # Position
    axes[0].plot(time, data["position_command"], 'k--', label='Command', alpha=0.6)
    axes[0].plot(time, data["position_actual"], 'b-', label='Actual', linewidth=1.5)
    axes[0].set_ylabel('Position (rad)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'{data["metrics"]["f0"]} Hz → {data["metrics"]["f1"]} Hz')

    # Velocity
    axes[1].plot(time, data["velocity_actual"], 'g-', linewidth=1.5)
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].grid(True, alpha=0.3)

    # Torque
    axes[2].plot(time, data["torque"], 'r-', linewidth=1.5)
    axes[2].set_ylabel('Torque (Nm)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_dir / f'chirp_response_{actuator_type}.png', dpi=150)
    plt.close()


if __name__ == "__main__":
    main()
