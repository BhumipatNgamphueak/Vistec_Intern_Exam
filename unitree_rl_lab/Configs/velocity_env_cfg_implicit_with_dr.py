"""
Configuration for Go2 velocity tracking with IMPLICIT actuator and COMPREHENSIVE domain randomization.

This configuration uses the implicit actuator (UnitreeActuatorCfg_Go2HV) with the SAME
comprehensive domain randomization as the MLP custom configuration, enabling fair comparison
between actuator types under identical training conditions.

COMPREHENSIVE DOMAIN RANDOMIZATION (IDENTICAL to MLP Custom):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1. Physics Material: Surface friction (0.4-1.25), restitution (0.0-0.15)
2. Base Mass: Random mass addition (-1 to +3 kg)
3. Center of Mass: Position offset randomization (±2cm in x,y,z per link)
4. Initial State: Random spawn position (±0.5m), orientation (±180°)
5. Joint Positions: Reset with ±60° offset from default
6. Motor Strength: Stiffness ±25%, damping ±50% variation
7. Joint Friction: Friction coefficient (0-0.15 Nm)
8. Joint Armature: Rotor inertia (0-0.015 kg·m²)
9. External Disturbances:
   - Velocity pushes: ±1.0 m/s every 5-10s
   - Force impulses: ±10N every 3-8s
   - Torque impulses: ±3Nm every 3-8s
10. Observation Noise: IMU noise (±0.05 gravity, ±0.2 ang_vel), encoder noise
11. Spawn height: 0.0-0.3m above ground (drop test)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Use this to compare:
1. Implicit + No DR vs Implicit + Full DR → measures DR benefit on implicit actuator
2. Implicit + Full DR vs MLP + Full DR → measures actuator quality under same DR
"""

import os
from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

# Import the base configuration
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
    EventCfg as BaseEventCfg,
    ObservationsCfg as BaseObservationsCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG  # Implicit actuator
from unitree_rl_lab.tasks.locomotion import mdp
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

# Import custom DR functions
from unitree_rl_lab.tasks.mimic.mdp.events import randomize_rigid_body_com


##
## COMPREHENSIVE DOMAIN RANDOMIZATION (SAME AS MLP CUSTOM)
##

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
                "x": (-0.02, 0.02),  # ±2cm in x-axis
                "y": (-0.02, 0.02),  # ±2cm in y-axis
                "z": (-0.02, 0.02),  # ±2cm in z-axis
            },
        },
    )

    # Joint position randomization at reset (±60°)
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-1.05, 1.05),  # ±60° offset
            "velocity_range": (-1.0, 1.0),
        },
    )

    # Motor strength randomization
    randomize_motor_strength = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stiffness_distribution_params": (0.75, 1.25),  # ±25% stiffness
            "damping_distribution_params": (0.5, 1.5),      # ±50% damping
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
    """Custom observations with reduced noise (matching MLP custom config)."""

    @configclass
    class PolicyCfg(BaseObservationsCfg.PolicyCfg):
        """Observations for policy with adjusted noise levels."""

        # Override joint velocity with reduced noise
        joint_vel_rel = mdp.ObservationTermCfg(
            func=mdp.joint_vel_rel,
            scale=0.05,
            clip=(-100, 100),
            noise=Unoise(n_min=-0.5, n_max=0.5)  # Reduced from ±1.5 to ±0.5
        )

    # Override policy observations
    policy: PolicyCfg = PolicyCfg()


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """Configuration: Implicit actuator + Comprehensive DR (same as MLP custom)."""

    # Use comprehensive DR event configuration
    events: ComprehensiveDREventCfg = ComprehensiveDREventCfg()

    # Use custom observations with reduced noise
    observations: CustomObservationsCfg = CustomObservationsCfg()

    def __post_init__(self):
        # Use GitHub default implicit actuator
        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Call parent post_init
        super().__post_init__()


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play configuration: Implicit actuator + Comprehensive DR."""

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
