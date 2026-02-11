"""
Configuration for Go2 velocity tracking with CUSTOM trained LSTM actuator.

This configuration uses your custom trained LSTM model from actuator_net.

DOMAIN RANDOMIZATION: Inherits from base LSTM config
- Physics material randomization (friction, restitution)
- Base mass randomization (-1 to +3 kg)
- Initial state randomization (position, orientation, joint velocities)
- External disturbances (periodic pushes)
- Observation noise (IMU, encoders)

To modify domain randomization, override the 'events' attribute below.
"""

import os
from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

# Import the base LSTM configuration
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg_lstm import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
    EventCfg as BaseEventCfg,
    ActionsCfg as BaseActionsCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_LSTM_CFG
from unitree_rl_lab.assets.robots import unitree_actuators
from unitree_rl_lab.tasks.locomotion import mdp
from isaaclab.utils.noise import GaussianNoiseCfg


##
## DOMAIN RANDOMIZATION CONFIGURATION
## Uncomment and modify the section below to customize randomization
##

@configclass
class CustomEventCfg(BaseEventCfg):
    """Custom domain randomization with joint friction/damping, joint parameters, and action noise."""

    # Keep default friction range (can modify if needed)
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.2),  # Default range
            "dynamic_friction_range": (0.3, 1.2),
            "restitution_range": (0.0, 0.15),
            "num_buckets": 64,
        },
    )

    # Keep default mass randomization (can modify if needed)
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-1.0, 3.0),  # Default range
            "operation": "add",
        },
    )

    # Keep default pushes (can modify if needed)
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(5.0, 10.0),  # Default interval
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},  # Default strength
    )

    # NEW: Joint stiffness and damping randomization
    randomize_joint_stiffness_damping = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (20.0, 30.0),  # 20-30 Nm/rad (default: 25)
            "damping_distribution_params": (0.3, 0.7),  # 0.3-0.7 Nm*s/rad (default: 0.5)
            "operation": "abs",
            "distribution": "uniform",
        },
    )

    # NEW: Joint friction and armature randomization (affects motor response)
    randomize_joint_friction = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "friction_distribution_params": (0.0, 0.1),  # Joint friction: 0-0.1 Nm
            "armature_distribution_params": (0.0, 0.01),  # Motor inertia variation
            "operation": "abs",
            "distribution": "uniform",
        },
    )

    # Override spawn randomization with larger range (optional - uncomment to enable)
    # reset_base = EventTerm(
    #     func=mdp.reset_root_state_uniform,
    #     mode="reset",
    #     params={
    #         "pose_range": {
    #             "x": (-1.0, 1.0),        # Wider X range (default: ±0.5m)
    #             "y": (-1.0, 1.0),        # Wider Y range (default: ±0.5m)
    #             "z": (-0.05, 0.05),      # Small Z variation (default: 0)
    #             "roll": (-0.1, 0.1),     # Small roll variation (default: 0)
    #             "pitch": (-0.1, 0.1),    # Small pitch variation (default: 0)
    #             "yaw": (-3.14, 3.14),    # Full rotation (already default)
    #         },
    #         "velocity_range": {
    #             "x": (-0.2, 0.2),        # Initial forward/backward velocity
    #             "y": (-0.2, 0.2),        # Initial sideways velocity
    #             "z": (0.0, 0.0),         # No vertical velocity
    #             "roll": (-0.1, 0.1),     # Small roll rate
    #             "pitch": (-0.1, 0.1),    # Small pitch rate
    #             "yaw": (-0.2, 0.2),      # Small yaw rate
    #         },
    #     },
    # )

    # NEW: Action delay randomization (simulates communication/computation delays)
    # Note: Action noise is typically handled via the action configuration itself,
    # but we can add systematic delays here. For Gaussian action noise, see ActionsCfg below.


@configclass
class CustomActionsCfg(BaseActionsCfg):
    """Custom actions configuration with Gaussian noise."""

    # Override joint position action with noise
    JointPositionAction = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.25,
        use_default_offset=True,
        clip={".*": (-100.0, 100.0)},
        # Add Gaussian noise to actions (simulates actuator response noise)
        # noise_cfg=GaussianNoiseCfg(
        #     mean=0.0,
        #     std=0.01,  # 1% noise on normalized actions
        #     operation="add",
        # ),
    )


# Create a custom actuator config that uses your trained model
@configclass
class CustomLSTMActuatorCfg(unitree_actuators.LSTMActuatorNetworkCfg):
    """Custom LSTM actuator using your trained model."""

    # Point to YOUR custom trained model
    network_file = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "assets", "actuator_models", "actuator_lstm.pth")

    # LSTM architecture (must match your training)
    hidden_size = 64
    num_layers = 2

    # PD gains
    stiffness = {".*": 25.0}
    damping = {".*": 0.5}

    # Go2 actuator limits
    effort_limit = {
        ".*_hip_joint": 23.7,
        ".*_thigh_joint": 23.7,
        ".*_calf_joint": 45.43,
    }
    velocity_limit = {
        ".*_hip_joint": 30.1,
        ".*_thigh_joint": 30.1,
        ".*_calf_joint": 15.7,
    }


# Create custom robot config with your LSTM model
UNITREE_GO2_CUSTOM_LSTM_CFG = UNITREE_GO2_LSTM_CFG.replace(
    actuators={
        "legs": CustomLSTMActuatorCfg(
            joint_names_expr=[".*"],
        ),
    }
)


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """Configuration using CUSTOM trained LSTM actuator with comprehensive domain randomization."""

    # Custom domain randomization with joint dynamics
    events: CustomEventCfg = CustomEventCfg()

    # Custom actions with noise (commented by default - uncomment to enable action noise)
    # actions: CustomActionsCfg = CustomActionsCfg()

    def __post_init__(self):
        # Replace the robot config with custom LSTM version
        from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg_lstm import RobotSceneCfg

        # Update the robot configuration to use custom LSTM
        self.scene.robot = UNITREE_GO2_CUSTOM_LSTM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Call parent post_init
        super().__post_init__()


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play configuration using CUSTOM trained LSTM actuator."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 1
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
