"""
Configuration for Go2 velocity tracking with LSTM actuator and NO domain randomization.

This is a CLEAN BASELINE configuration for:
- LSTM actuator (for temporal modeling)
- NO domain randomization
- Fixed physics parameters
- Fixed joint positions at reset
- No disturbances
- No observation noise

Use this for training a baseline LSTM policy without DR to compare against:
- LSTM with DR (velocity_env_cfg_lstm_custom.py)
- MLP with DR (velocity_env_cfg_mlp_custom.py)
- MLP without DR (velocity_env_cfg_mlp_no_dr.py)
"""

import os
from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm

# Import the base velocity configuration
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
    ObservationsCfg as BaseObservationsCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_LSTM_CFG
from unitree_rl_lab.assets.robots import unitree_actuators
from unitree_rl_lab.tasks.locomotion import mdp


##
## NO DOMAIN RANDOMIZATION CONFIGURATION
##

@configclass
class NoDREventCfg:
    """Minimal event configuration with NO domain randomization."""

    # Reset base to fixed position (no randomization)
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (0.0, 0.0),      # Fixed X position
                "y": (0.0, 0.0),      # Fixed Y position
                "yaw": (0.0, 0.0),    # Fixed yaw orientation
            },
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
            "position_range": (0.0, 0.0),  # No position randomization
            "velocity_range": (0.0, 0.0),  # No velocity randomization
        },
    )


@configclass
class NoNoiseObservationsCfg(BaseObservationsCfg):
    """Observations with NO noise (clean baseline)."""

    @configclass
    class PolicyCfg(BaseObservationsCfg.PolicyCfg):
        """Observations for policy with NO noise."""

        # Override all observations to remove noise
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            scale=0.2,
            clip=(-100, 100),
            # NO NOISE
        )

        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            clip=(-100, 100),
            # NO NOISE
        )

        joint_pos_rel = ObsTerm(
            func=mdp.joint_pos_rel,
            clip=(-100, 100),
            # NO NOISE
        )

        joint_vel_rel = ObsTerm(
            func=mdp.joint_vel_rel,
            scale=0.05,
            clip=(-100, 100),
            # NO NOISE
        )

        def __post_init__(self):
            # Disable observation corruption (no noise)
            self.enable_corruption = False
            self.concatenate_terms = True

    # Override policy observations
    policy: PolicyCfg = PolicyCfg()


# Use baseline LSTM actuator config (no custom modifications)
@configclass
class BaselineLSTMActuatorCfg(unitree_actuators.LSTMActuatorNetworkCfg):
    """Baseline LSTM actuator using the trained model with default settings."""

    # Point to the LSTM actuator model
    network_file = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "assets", "actuator_models", "actuator_lstm.pth"
    )

    # PD gains (default)
    stiffness = {".*": 25.0}
    damping = {".*": 0.5}

    # Go2 actuator limits (default)
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


# Create baseline robot config with LSTM actuator
UNITREE_GO2_BASELINE_LSTM_CFG = UNITREE_GO2_LSTM_CFG.replace(
    actuators={
        "legs": BaselineLSTMActuatorCfg(
            joint_names_expr=[".*"],
        ),
    }
)


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """Baseline configuration using LSTM actuator with NO domain randomization."""

    # Use no-DR event configuration
    events: NoDREventCfg = NoDREventCfg()

    # Use no-noise observations
    observations: NoNoiseObservationsCfg = NoNoiseObservationsCfg()

    def __post_init__(self):
        # Replace the robot config with baseline LSTM version
        self.scene.robot = UNITREE_GO2_BASELINE_LSTM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Call parent post_init
        super().__post_init__()


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play configuration using LSTM actuator with no DR."""

    def __post_init__(self):
        super().__post_init__()

        # Ensure observation corruption is disabled for inference
        self.observations.policy.enable_corruption = False

        # Reduce number of environments for visualization
        self.scene.num_envs = 32
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 1

        # Use full command ranges during play
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
