#!/usr/bin/env python3
"""
Fixed play script that handles tuple observations.
This is a wrapper around the original play.py with observation handling fixes.
"""

import sys
import os

# Monkey-patch to fix tuple observation issue
original_file = os.path.join(os.path.dirname(__file__), "scripts", "rsl_rl", "play.py")

# Read the original play.py
with open(original_file, 'r') as f:
    play_code = f.read()

# Find and replace the observation handling section
old_code = """    # reset environment
    obs = env.get_observations()
    if version("rsl-rl-lib").startswith("2.3."):
        obs, _ = env.get_observations()
    timestep = 0
    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)"""

new_code = """    # reset environment
    obs = env.get_observations()
    if version("rsl-rl-lib").startswith("2.3."):
        obs, _ = env.get_observations()

    # Fix for tuple observations: extract first element if tuple
    def get_policy_obs(obs_data):
        if isinstance(obs_data, tuple):
            # If it's a tuple, return the first element (assumed to be policy obs)
            return obs_data[0]
        elif isinstance(obs_data, dict):
            # If it's a dict, return the 'policy' key
            return obs_data.get('policy', obs_data)
        else:
            # Otherwise return as-is (assumed to be tensor)
            return obs_data

    timestep = 0
    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            policy_obs = get_policy_obs(obs)
            actions = policy(policy_obs)"""

# Replace and fix the obs handling in the step as well
play_code = play_code.replace(old_code, new_code)

# Also need to handle obs in the step
play_code = play_code.replace(
    "            # env stepping\n            obs, _, _, _ = env.step(actions)",
    "            # env stepping\n            obs, _, _, _ = env.step(actions)\n            # Handle tuple obs in next iteration\n            # (get_policy_obs will be called at top of loop)"
)

# Execute the modified code
exec(play_code, globals())
