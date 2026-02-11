"""
Configuration for Go2 velocity tracking with LSTM actuator and COMPREHENSIVE domain randomization.

Uses your trained LSTM actuator model with 6 input features:
    [pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]

COMPREHENSIVE DOMAIN RANDOMIZATION (SAME AS MLP Custom):
- Physics Material: Surface friction (0.4-1.25), restitution (0.0-0.15)
- Base Mass: Random mass addition (-1 to +3 kg)
- Center of Mass: Position offset randomization (+/-2cm in x,y,z per link)
- Joint Positions: Reset with +/-60 degree offset from default
- Motor Strength: Stiffness +/-25%, damping +/-50% variation
- Joint Friction: Friction coefficient (0-0.15 Nm)
- Joint Armature: Rotor inertia (0-0.015 kg*m^2)
- External Disturbances: Velocity pushes, force/torque impulses
- Spawn height: 0.0-0.3m above ground (drop test)
"""

import os
from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
    EventCfg as BaseEventCfg,
    ObservationsCfg as BaseObservationsCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_LSTM_CFG  # LSTM actuator
from unitree_rl_lab.tasks.locomotion import mdp
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

# Import custom DR functions
from unitree_rl_lab.tasks.mimic.mdp.events import randomize_rigid_body_com


@configclass
class ComprehensiveDREventCfg(BaseEventCfg):
    """Comprehensive domain randomization - IDENTICAL to MLP custom config."""

    # Physics material randomization
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.4, 1.25),
            "dynamic_friction_range": (0.4, 1.25),
            "restitution_range": (0.0, 0.15),
            "num_buckets": 64,
        },
    )

    # Base mass randomization
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
        },
    )

    # Center of Mass (COM) Position Randomization
    randomize_com = EventTerm(
        func=randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_hip|.*_thigh|.*_calf|base"),
            "com_range": {
                "x": (-0.02, 0.02),
                "y": (-0.02, 0.02),
                "z": (-0.02, 0.02),
            },
        },
    )

    # Joint position randomization at reset (+/-60 degrees)
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-1.05, 1.05),
            "velocity_range": (-1.0, 1.0),
        },
    )

    # Motor strength randomization
    randomize_motor_strength = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.75, 1.25),
            "damping_distribution_params": (0.5, 1.5),
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
            "friction_distribution_params": (0.0, 0.15),
            "armature_distribution_params": (0.0, 0.015),
            "distribution": "uniform",
        },
    )

    # External disturbances - Velocity-based pushes
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(5.0, 10.0),
        params={"velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0)}},
    )

    # External disturbances - Force and Torque at Reset
    base_external_force_torque_reset = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (-5.0, 5.0),
            "torque_range": (-2.0, 2.0),
        },
    )

    # External disturbances - Periodic Force and Torque
    base_external_force_torque_interval = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="interval",
        interval_range_s=(3.0, 8.0),
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (-10.0, 10.0),
            "torque_range": (-3.0, 3.0),
        },
    )

    # Spawn Position Configuration (0.0-0.3m above ground)
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.3),
                "yaw": (-3.14, 3.14)
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


@configclass
class CustomObservationsCfg(BaseObservationsCfg):
    """Custom observations with reduced noise."""

    @configclass
    class PolicyCfg(BaseObservationsCfg.PolicyCfg):
        joint_vel_rel = mdp.ObservationTermCfg(
            func=mdp.joint_vel_rel,
            scale=0.05,
            clip=(-100, 100),
            noise=Unoise(n_min=-0.5, n_max=0.5)
        )

    policy: PolicyCfg = PolicyCfg()


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """LSTM actuator + Comprehensive DR (same as MLP custom)."""

    events: ComprehensiveDREventCfg = ComprehensiveDREventCfg()
    observations: CustomObservationsCfg = CustomObservationsCfg()

    def __post_init__(self):
        # Use LSTM actuator
        self.scene.robot = UNITREE_GO2_LSTM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        super().__post_init__()


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play configuration: LSTM actuator + Comprehensive DR."""

    def __post_init__(self):
        super().__post_init__()
        self.observations.policy.enable_corruption = False
        self.scene.num_envs = 32
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 1
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
