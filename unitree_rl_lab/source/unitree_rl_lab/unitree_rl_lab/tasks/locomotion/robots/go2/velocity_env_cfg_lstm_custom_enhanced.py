"""
Configuration for Go2 velocity tracking with CUSTOM trained LSTM actuator.
WITH ENHANCED DOMAIN RANDOMIZATION for better sim-to-real transfer.
"""

import math
import os
from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

# Import the base LSTM configuration
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg_lstm import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
    EventCfg as BaseEventCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_LSTM_CFG
from unitree_rl_lab.assets.robots import unitree_actuators
from unitree_rl_lab.tasks.locomotion import mdp


# Create a custom actuator config that uses your trained model
@configclass
class CustomLSTMActuatorCfg(unitree_actuators.LSTMActuatorNetworkCfg):
    """Custom LSTM actuator using your trained model."""

    # Point to YOUR custom trained model (relative path from this file)
    network_file = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "assets", "actuator_models", "actuator_lstm.pth")

    # LSTM architecture (must match your training in train_lstm.py)
    # Input: 6 features (same as MLP) - no sequence needed
    hidden_size = 64   # HIDDEN_DIM in train_lstm.py
    num_layers = 2     # NUM_LAYERS in train_lstm.py

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
class EnhancedEventCfg(BaseEventCfg):
    """Enhanced domain randomization configuration."""

    # INCREASED physics material randomization (wider friction range)
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.2, 1.5),  # Wider range: ice to rubber
            "dynamic_friction_range": (0.2, 1.5),
            "restitution_range": (0.0, 0.2),  # More bounce
            "num_buckets": 128,  # More discrete levels
        },
    )

    # INCREASED base mass randomization (heavier payloads)
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-2.0, 5.0),  # -2kg to +5kg
            "operation": "add",
        },
    )

    # STRONGER external disturbances
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(3.0, 8.0),  # More frequent pushes
        params={"velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0)}},  # Stronger pushes
    )

    # ADD: Joint friction randomization (NEW)
    randomize_joint_friction = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (20.0, 30.0),  # 20-30 Nm/rad
            "damping_distribution_params": (0.3, 0.7),  # 0.3-0.7 Nm*s/rad
            "operation": "abs",
            "distribution": "uniform",
        },
    )

    # ADD: Motor strength randomization (NEW)
    randomize_motor_strength = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "effort_limit_distribution_params": (0.9, 1.1),  # ±10% motor strength
            "velocity_limit_distribution_params": (0.95, 1.05),  # ±5% max velocity
            "operation": "scale",
            "distribution": "uniform",
        },
    )


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """Configuration using CUSTOM trained LSTM actuator with ENHANCED randomization."""

    # Override with enhanced events
    events: EnhancedEventCfg = EnhancedEventCfg()

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
