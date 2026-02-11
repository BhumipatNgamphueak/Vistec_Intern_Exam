#!/usr/bin/env python3
"""
Hang Go2 robot in the air and test individual motor commands.

This script:
- Suspends robot at height (no ground contact)
- Fixes base position (no falling)
- Allows commanding individual joints
- Visualizes joint movements
"""

import argparse
import numpy as np
import torch
from pathlib import Path

# IsaacLab imports (must be after AppLauncher)
from omni.isaac.lab.app import AppLauncher

def main():
    parser = argparse.ArgumentParser(description="Test Go2 motors while hanging")
    parser.add_argument("--actuator", type=str, default="mlp",
                        choices=["mlp", "lstm", "implicit"],
                        help="Actuator type to test")
    parser.add_argument("--joint", type=str, default="all",
                        help="Joint to test (or 'all' for sequence)")
    parser.add_argument("--motion", type=str, default="sine",
                        choices=["sine", "step", "sweep"],
                        help="Motion type")
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
    from omni.isaac.lab.utils.math import quat_from_euler_xyz

    # Import configs
    import sys
    sys.path.append(str(Path(__file__).parents[2] / "source/unitree_rl_lab"))
    from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG
    from unitree_rl_lab.assets.robots.unitree_actuators import (
        UnitreeActuatorCfg_Go2_MLP,
        UnitreeActuatorCfg_Go2_LSTM
    )
    from omni.isaac.lab.actuators import IdealPDActuatorCfg

    print("="*60)
    print("Go2 Motor Testing - Hanging Configuration")
    print("="*60)
    print(f"\nActuator: {args_cli.actuator.upper()}")
    print(f"Joint: {args_cli.joint}")
    print(f"Motion: {args_cli.motion}")
    print("")

    # Setup simulation
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device="cuda:0")
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 2.0, 2.0], [0.0, 0.0, 1.0])

    # Create robot with specific actuator
    robot_cfg = UNITREE_GO2_CFG.copy()
    robot_cfg.prim_path = "/World/Robot"

    # Configure actuator
    if args_cli.actuator == "mlp":
        print("Actuator: MLP (Kp=25.0, Kd=0.5)")
        robot_cfg.actuators["legs"] = UnitreeActuatorCfg_Go2_MLP(
            joint_names_expr=[".*"],
        )
    elif args_cli.actuator == "lstm":
        print("Actuator: LSTM (Kp=25.0, Kd=0.5)")
        robot_cfg.actuators["legs"] = UnitreeActuatorCfg_Go2_LSTM(
            joint_names_expr=[".*"],
        )
    else:  # implicit
        print("Actuator: Implicit (Kp=160.0, Kd=5.0)")
        robot_cfg.actuators["legs"] = IdealPDActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=160.0,
            damping=5.0,
        )

    # Create robot
    robot = Articulation(robot_cfg)

    # Define ground plane (for reference, but robot will be above it)
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)

    # Play simulation
    sim.reset()

    # Hang robot in the air (1.5m height)
    print("\n🔧 Suspending robot at 1.5m height...")
    hanging_height = 1.5
    robot.write_root_pose_to_sim(
        torch.tensor([[0.0, 0.0, hanging_height]], device=sim.device),
        torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=sim.device)  # No rotation
    )

    # Fix base position (disable gravity effect on base)
    # Note: In IsaacLab, we'll continuously set base position to maintain height

    # Get joint info
    joint_names = robot.data.joint_names
    print(f"\nAvailable joints: {len(joint_names)}")
    for i, name in enumerate(joint_names):
        print(f"  [{i}] {name}")

    # Determine which joints to test
    if args_cli.joint == "all":
        test_joints = list(range(len(joint_names)))
        print("\nTesting ALL joints sequentially")
    else:
        if args_cli.joint in joint_names:
            test_joints = [joint_names.index(args_cli.joint)]
            print(f"\nTesting joint: {args_cli.joint}")
        else:
            print(f"\n❌ Joint '{args_cli.joint}' not found!")
            simulation_app.close()
            return

    # Test parameters
    dt = sim_cfg.dt
    control_freq = 50  # Hz
    decimation = int(1.0 / (control_freq * dt))

    print(f"\nControl frequency: {control_freq} Hz")
    print(f"Physics dt: {dt*1000:.1f} ms")
    print("")
    print("Press Ctrl+C to stop")
    print("="*60)

    # Initialize default position (standing pose)
    default_pos = torch.zeros((1, robot.num_joints), device=sim.device)
    # Set to standing configuration
    default_pos[0, :] = torch.tensor([
        0.0, 0.8, -1.5,  # FL: hip, thigh, calf
        0.0, 0.8, -1.5,  # FR
        0.0, 1.0, -1.5,  # RL
        0.0, 1.0, -1.5,  # RR
    ], device=sim.device)

    robot.write_joint_state_to_sim(default_pos, torch.zeros_like(default_pos))

    # Run simulation with motor commands
    step = 0
    joint_cycle_duration = 5.0  # seconds per joint
    current_joint_idx = 0

    try:
        while simulation_app.is_running():
            # Calculate time
            t = step * dt

            # For "all" mode, cycle through joints
            if args_cli.joint == "all":
                current_joint_idx = int(t / joint_cycle_duration) % len(test_joints)
                joint_to_move = test_joints[current_joint_idx]

                # Show which joint is being tested
                if step % (control_freq * 5) == 0:  # Every 5 seconds
                    print(f"[{t:.1f}s] Testing: {joint_names[joint_to_move]}")
            else:
                joint_to_move = test_joints[0]

            # Generate motion command
            cmd = default_pos.clone()

            if args_cli.motion == "sine":
                # Sinusoidal motion
                amplitude = 0.5  # radians
                frequency = 1.0  # Hz
                local_t = t % joint_cycle_duration
                cmd[0, joint_to_move] = default_pos[0, joint_to_move] + \
                                        amplitude * np.sin(2 * np.pi * frequency * local_t)

            elif args_cli.motion == "step":
                # Step motion (alternating)
                step_size = 0.5
                local_t = t % joint_cycle_duration
                if local_t < joint_cycle_duration / 2:
                    cmd[0, joint_to_move] = default_pos[0, joint_to_move] + step_size
                else:
                    cmd[0, joint_to_move] = default_pos[0, joint_to_move] - step_size

            elif args_cli.motion == "sweep":
                # Sweep through range
                local_t = t % joint_cycle_duration
                progress = local_t / joint_cycle_duration
                sweep_range = 1.0  # ±0.5 rad
                cmd[0, joint_to_move] = default_pos[0, joint_to_move] + \
                                        sweep_range * (progress - 0.5)

            # Apply command every decimation steps
            if step % decimation == 0:
                robot.set_joint_position_target(cmd)

            # Keep base at fixed height (compensate for any drift)
            if step % 10 == 0:  # Update every 10 steps
                robot.write_root_pose_to_sim(
                    torch.tensor([[0.0, 0.0, hanging_height]], device=sim.device),
                    torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=sim.device)
                )

            # Step simulation
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)

            step += 1

    except KeyboardInterrupt:
        print("\n\n⏹️  Stopped by user")

    # Close simulation
    simulation_app.close()

    print("\n" + "="*60)
    print("Motor Testing Complete!")
    print("="*60)


if __name__ == "__main__":
    main()
