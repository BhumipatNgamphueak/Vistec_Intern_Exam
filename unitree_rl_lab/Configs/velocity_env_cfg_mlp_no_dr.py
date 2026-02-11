"""
Configuration for Go2 velocity tracking with MLP actuator and NO domain randomization.

This is a BASELINE configuration for comparison experiments:
- MLP actuator (same as custom DR config)
- NO domain randomization
- Fixed physics parameters
- Fixed joint positions at reset
- No disturbances

Use this to compare the impact of comprehensive DR vs no DR on the same MLP actuator.
"""

import os
from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

# Import the base MLP configuration
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_MLP_CFG
from unitree_rl_lab.assets.robots import unitree_actuators
from unitree_rl_lab.tasks.locomotion import mdp


##
## NO DOMAIN RANDOMIZATION CONFIGURATION (BASELINE)
##

@configclass
class NoDREventCfg:
    """Minimal event configuration with NO domain randomization for baseline comparison."""

    # Only keep basic reset without any randomization
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)},  # Fixed spawn position
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    # Reset joints to default position (no randomization)
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (0.0, 0.0),  # No randomization - always default pose
            "velocity_range": (0.0, 0.0),  # No velocity randomization
        },
    )


# Use the default MLP actuator config (no custom modifications)
@configclass
class BaselineMLPActuatorCfg(unitree_actuators.MLPActuatorNetworkCfg):
    """Baseline MLP actuator using the trained model with default settings."""

    # Point to the MLP actuator model
    network_file = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "assets", "actuator_models", "actuator_mlp.pth")

    # PD gains (default)
    stiffness = {".*": 25.0}
    damping = {".*": 0.5}

    # Go2 actuator limits (default, no reduction)
    effort_limit = {
        ".*_hip_joint": 23.7,
        ".*_thigh_joint": 23.7,
        ".*_calf_joint": 45.43,
    }

    # Velocity limits: Use FULL nominal values (no reduction for baseline)
    velocity_limit = {
        ".*_hip_joint": 30.1,      # Full nominal
        ".*_thigh_joint": 30.1,    # Full nominal
        ".*_calf_joint": 15.7,     # Full nominal
    }


# Create baseline robot config with MLP actuator
UNITREE_GO2_BASELINE_MLP_CFG = UNITREE_GO2_MLP_CFG.replace(
    actuators={
        "legs": BaselineMLPActuatorCfg(
            joint_names_expr=[".*"],
        ),
    }
)


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """Baseline configuration using MLP actuator with NO domain randomization."""

    # Use no-DR event configuration
    events: NoDREventCfg = NoDREventCfg()

    def __post_init__(self):
        # Replace the robot config with baseline MLP version
        self.scene.robot = UNITREE_GO2_BASELINE_MLP_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Call parent post_init
        super().__post_init__()


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play configuration using MLP actuator with no DR."""

    def __post_init__(self):
        super().__post_init__()

        # Disable observation corruption for inference
        self.observations.policy.enable_corruption = False

        # Reduce number of environments for visualization
        self.scene.num_envs = 32
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 1

        # Use full command ranges during play
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
