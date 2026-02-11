"""
Configuration for Go2 velocity tracking with IMPLICIT actuator and NO domain randomization.

This is the GitHub default actuator (UnitreeActuatorCfg_Go2HV) with physics-based
torque-speed curves and ZERO domain randomization for pure baseline comparison.

Key features:
- Implicit actuator (physics-based, no neural networks)
- NO domain randomization (fixed spawn, no randomization)
- NO trained models required - works out of the box
- Pure baseline for comparison experiments

Use this to compare:
1. Implicit actuator + No DR (this config)
2. MLP actuator + No DR (your trained model, no randomization)
3. MLP actuator + Comprehensive DR (Gazebo-ready with full randomization)
"""

from isaaclab.utils import configclass
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

# Import the base configuration
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg import (
    RobotEnvCfg as BaseRobotEnvCfg,
    RobotPlayEnvCfg as BaseRobotPlayEnvCfg,
)
from unitree_rl_lab.assets.robots.unitree import UNITREE_GO2_CFG  # GitHub default implicit actuator
from unitree_rl_lab.tasks.locomotion import mdp


##
## GITHUB DEFAULT CONFIGURATION (IMPLICIT ACTUATOR)
##

@configclass
class ImplicitEventCfg:
    """NO domain randomization - pure baseline for comparison."""

    # Basic reset with NO randomization
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


@configclass
class RobotEnvCfg(BaseRobotEnvCfg):
    """Baseline configuration: Implicit actuator + NO domain randomization."""

    # Use no-DR event configuration
    events: ImplicitEventCfg = ImplicitEventCfg()

    def __post_init__(self):
        # Replace the robot config with GitHub default implicit actuator
        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Call parent post_init
        super().__post_init__()


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """Play configuration: Implicit actuator + No DR."""

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
