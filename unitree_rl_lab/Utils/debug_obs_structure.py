#!/usr/bin/env python3
"""Debug script to check observation structure."""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Unitree-Go2-Velocity-MLP-Custom")
parser.add_argument("--num_envs", type=int, default=32)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
from unitree_rl_lab.utils.parser_cfg import parse_env_cfg

def main():
    # Parse environment config
    env_cfg = parse_env_cfg(
        args_cli.task,
        device="cuda:0",
        num_envs=args_cli.num_envs,
        use_fabric=True,
        entry_point_key="play_env_cfg_entry_point",
    )

    # Create environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if not args_cli.headless else None)

    # Reset and check observations
    print("\n" + "="*70)
    print("Observation Structure Diagnostics")
    print("="*70)

    # Get observations using the direct method
    obs = env.get_observations()

    print(f"\n1. Type of obs from env.get_observations(): {type(obs)}")

    if isinstance(obs, tuple):
        print(f"   → obs is a TUPLE with {len(obs)} elements")
        for i, elem in enumerate(obs):
            print(f"   → Element {i}: type={type(elem)}, ", end="")
            if hasattr(elem, 'shape'):
                print(f"shape={elem.shape}")
            else:
                print(f"value={elem}")
    elif isinstance(obs, dict):
        print(f"   → obs is a DICT with keys: {obs.keys()}")
        for key, val in obs.items():
            print(f"   → {key}: type={type(val)}, ", end="")
            if hasattr(val, 'shape'):
                print(f"shape={val.shape}")
            else:
                print(f"value={val}")
    else:
        print(f"   → obs is a tensor/array with shape: {obs.shape if hasattr(obs, 'shape') else 'N/A'}")

    # Try the observation group method
    print(f"\n2. Checking env.unwrapped.observation_manager...")
    if hasattr(env.unwrapped, 'observation_manager'):
        obs_manager = env.unwrapped.observation_manager
        print(f"   → Has observation_manager: {obs_manager}")

        if hasattr(obs_manager, 'compute_group'):
            policy_obs = obs_manager.compute_group("policy")
            print(f"   → policy obs type: {type(policy_obs)}")
            if hasattr(policy_obs, 'shape'):
                print(f"   → policy obs shape: {policy_obs.shape}")

    print("\n" + "="*70)
    print("Diagnostics Complete!")
    print("="*70 + "\n")

    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
