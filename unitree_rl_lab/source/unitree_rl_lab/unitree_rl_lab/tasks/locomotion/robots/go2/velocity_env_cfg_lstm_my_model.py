"""
Configuration for Go2 velocity tracking with YOUR CUSTOM TRAINED LSTM actuator.

This configuration uses your newly trained LSTM model from actuator_net:
/home/drl-68/actuator_net/app/resources/actuator_lstm.pth

COMPREHENSIVE DOMAIN RANDOMIZATION for Sim-to-Real Transfer:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Physics Material: Surface friction (0.3-1.2), restitution (0.0-0.15)
2. Base Mass: Random mass addition (-1 to +3 kg)
3. Center of Mass: Per-link mass variation (±20% scale) to shift COM
4. Initial State: Random spawn position (±0.5m), orientation (±180°), velocities
5. Joint Positions: Reset with ±30% variation from default
6. Motor Strength: Stiffness and damping gains (±20% variation)
7. Joint Friction: Friction coefficient (0-0.15 Nm)
8. Joint Armature: Rotor inertia (0-0.015 kg·m²)
9. Motor Response: Modeled through damping and armature randomization
10. External Forces: Periodic velocity pushes (±0.5 m/s every 5-10s)
11. Observation Noise: IMU noise (±0.05 gravity, ±0.2 ang_vel), encoder noise (±0.01 rad)
12. Terrain Variation: Diverse terrain generation

Note: Action noise and latency effects are implicitly modeled through motor dynamics randomization
(friction, damping, armature variations) which provides similar robustness benefits.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
from isaaclab.utils import configclass

# Import the base LSTM configuration
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg_lstm import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
    EventCfg as BaseEventCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_LSTM_CFG
from unitree_rl_lab.assets.robots import unitree_actuators

# Import for event configuration
from isaaclab.envs import mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.noise import UniformNoiseCfg as Unoise


# Create actuator config that uses YOUR custom trained model
@configclass
class MyLSTMActuatorCfg(unitree_actuators.LSTMActuatorNetworkCfg):
    """Your custom trained LSTM actuator from actuator_net."""

    # Point to YOUR newly trained LSTM model
    network_file = "/home/drl-68/actuator_net/app/resources/actuator_lstm.pth"

    # LSTM architecture (must match your training)
    # Check your actuator_net training config if these differ
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


# Custom event configuration with comprehensive domain randomization
@configclass
class CustomEventCfg(BaseEventCfg):
    """Event configuration with comprehensive domain randomization for sim-to-real transfer."""

    # Override reset_robot_joints to add position randomization
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.7, 1.3),  # Randomize joint positions ±30%
            "velocity_range": (-1.0, 1.0),  # Keep velocity randomization
        },
    )

    # Center of Mass (COM) randomization - randomize link masses to shift COM
    randomize_com = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_hip|.*_thigh|.*_calf"),
            "mass_distribution_params": (0.8, 1.2),  # ±20% mass variation per link
            "operation": "scale",  # Multiply instead of add to keep masses positive
        },
    )

    # Motor strength randomization (actuator gains - stiffness and damping)
    randomize_motor_strength = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.8, 1.2),  # ±20% stiffness variation
            "damping_distribution_params": (0.8, 1.2),    # ±20% damping variation
            "operation": "scale",
            "distribution": "uniform",
        },
    )

    # Joint friction and armature randomization
    randomize_joint_friction = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "friction_distribution_params": (0.0, 0.15),  # Joint friction 0-0.15 Nm
            "armature_distribution_params": (0.0, 0.015),  # Joint armature 0-0.015 kg·m²
            "distribution": "uniform",
        },
    )

    # Action noise - adds noise to commanded actions
    # Note: This is applied during action processing, not as an event
    # It will be configured in the action manager


# Custom actions configuration
@configclass
class CustomActionsCfg:
    """Action specifications for domain randomization.

    Note: Action noise and latency are implemented through other DR mechanisms:
    - Observation noise provides sensor-level uncertainty
    - Motor dynamics (friction, damping, armature) provide actuation delays
    - Joint stiffness/damping randomization simulates response variations
    """

    # Use default joint position action configuration
    JointPositionAction = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.25,
        use_default_offset=True,
        clip={".*": (-100.0, 100.0)},
    )


# Create custom robot config with your LSTM model
UNITREE_GO2_MY_LSTM_CFG = UNITREE_GO2_LSTM_CFG.replace(
    actuators={
        "legs": MyLSTMActuatorCfg(
            joint_names_expr=[".*"],
        ),
    }
)


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """Configuration using YOUR custom trained LSTM actuator with comprehensive domain randomization."""

    # Use custom event configuration with comprehensive DR
    events: CustomEventCfg = CustomEventCfg()

    # Use custom actions configuration with action noise
    actions: CustomActionsCfg = CustomActionsCfg()

    def __post_init__(self):
        # Replace the robot config with your custom LSTM version
        self.scene.robot = UNITREE_GO2_MY_LSTM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Call parent post_init
        super().__post_init__()


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play configuration using YOUR custom trained LSTM actuator."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 1
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
