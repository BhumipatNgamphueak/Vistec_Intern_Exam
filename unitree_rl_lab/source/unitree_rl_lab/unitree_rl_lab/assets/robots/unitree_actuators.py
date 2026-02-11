from __future__ import annotations

import torch
import os
from dataclasses import MISSING
from pathlib import Path

from isaaclab.actuators import DelayedPDActuator, DelayedPDActuatorCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions


class UnitreeActuator(DelayedPDActuator):
    """Unitree actuator class that implements a torque-speed curve for the actuators.

    The torque-speed curve is defined as follows:

            Torque Limit, N·m
                ^
    Y2──────────|
                |──────────────Y1
                |              │\
                |              │ \
                |              │  \
                |              |   \
    ------------+--------------|------> velocity: rad/s
                              X1   X2

    - Y1: Peak Torque Test (Torque and Speed in the Same Direction)
    - Y2: Peak Torque Test (Torque and Speed in the Opposite Direction)
    - X1: Maximum Speed at Full Torque (T-N Curve Knee Point)
    - X2: No-Load Speed Test

    - Fs: Static friction coefficient
    - Fd: Dynamic friction coefficient
    - Va: Velocity at which the friction is fully activated
    """

    cfg: UnitreeActuatorCfg

    armature: torch.Tensor
    """The armature of the actuator joints. Shape is (num_envs, num_joints).
        armature = J2 + J1 * i2 ^ 2 + Jr * (i1 * i2) ^ 2
    """

    def __init__(self, cfg: UnitreeActuatorCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)

        self._joint_vel = torch.zeros_like(self.computed_effort)
        self._effort_y1 = self._parse_joint_parameter(cfg.Y1, 1e9)
        self._effort_y2 = self._parse_joint_parameter(cfg.Y2, cfg.Y1)
        self._velocity_x1 = self._parse_joint_parameter(cfg.X1, 1e9)
        self._velocity_x2 = self._parse_joint_parameter(cfg.X2, 1e9)
        self._friction_static = self._parse_joint_parameter(cfg.Fs, 0.0)
        self._friction_dynamic = self._parse_joint_parameter(cfg.Fd, 0.0)
        self._activation_vel = self._parse_joint_parameter(cfg.Va, 0.01)

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        # save current joint vel
        self._joint_vel[:] = joint_vel
        # calculate the desired joint torques
        control_action = super().compute(control_action, joint_pos, joint_vel)

        # apply friction model on the torque
        self.applied_effort -= (
            self._friction_static * torch.tanh(joint_vel / self._activation_vel) + self._friction_dynamic * joint_vel
        )

        control_action.joint_positions = None
        control_action.joint_velocities = None
        control_action.joint_efforts = self.applied_effort

        return control_action

    def _clip_effort(self, effort: torch.Tensor) -> torch.Tensor:
        # check if the effort is the same direction as the joint velocity
        same_direction = (self._joint_vel * effort) > 0
        max_effort = torch.where(same_direction, self._effort_y1, self._effort_y2)
        # check if the joint velocity is less than the max speed at full torque
        max_effort = torch.where(
            self._joint_vel.abs() < self._velocity_x1, max_effort, self._compute_effort_limit(max_effort)
        )
        return torch.clip(effort, -max_effort, max_effort)

    def _compute_effort_limit(self, max_effort):
        k = -max_effort / (self._velocity_x2 - self._velocity_x1)
        limit = k * (self._joint_vel.abs() - self._velocity_x1) + max_effort
        return limit.clip(min=0.0)


@configclass
class UnitreeActuatorCfg(DelayedPDActuatorCfg):
    """
    Configuration for Unitree actuators.
    """

    class_type: type = UnitreeActuator

    X1: float = 1e9
    """Maximum Speed at Full Torque(T-N Curve Knee Point) Unit: rad/s"""

    X2: float = 1e9
    """No-Load Speed Test Unit: rad/s"""

    Y1: float = MISSING
    """Peak Torque Test(Torque and Speed in the Same Direction) Unit: N*m"""

    Y2: float | None = None
    """Peak Torque Test(Torque and Speed in the Opposite Direction) Unit: N*m"""

    Fs: float = 0.0
    """ Static friction coefficient """

    Fd: float = 0.0
    """ Dynamic friction coefficient """

    Va: float = 0.01
    """ Velocity at which the friction is fully activated """


@configclass
class UnitreeActuatorCfg_M107_15(UnitreeActuatorCfg):
    X1 = 14.0
    X2 = 25.6
    Y1 = 150.0
    Y2 = 182.8

    armature = 0.063259741


@configclass
class UnitreeActuatorCfg_M107_24(UnitreeActuatorCfg):
    X1 = 8.8
    X2 = 16
    Y1 = 240
    Y2 = 292.5

    armature = 0.160478022


@configclass
class UnitreeActuatorCfg_Go2HV(UnitreeActuatorCfg):
    X1 = 13.5
    X2 = 30
    Y1 = 20.2
    Y2 = 23.4


@configclass
class UnitreeActuatorCfg_N7520_14p3(UnitreeActuatorCfg):
    # Decimal point cannot be used as variable name, use `p` instead
    X1 = 22.63
    X2 = 35.52
    Y1 = 71
    Y2 = 83.3

    Fs = 1.6
    Fd = 0.16

    """
    | rotor  | 0.489e-4 kg·m²
    | gear_1 | 0.098e-4 kg·m² | ratio | 4.5
    | gear_2 | 0.533e-4 kg·m² | ratio | 48/22+1
    """
    armature = 0.01017752


@configclass
class UnitreeActuatorCfg_N7520_22p5(UnitreeActuatorCfg):
    # Decimal point cannot be used as variable name, use `p` instead
    X1 = 14.5
    X2 = 22.7
    Y1 = 111.0
    Y2 = 131.0

    Fs = 2.4
    Fd = 0.24

    """
    | rotor  | 0.489e-4 kg·m²
    | gear_1 | 0.109e-4 kg·m² | ratio | 4.5
    | gear_2 | 0.738e-4 kg·m² | ratio | 5.0
    """
    armature = 0.025101925


@configclass
class UnitreeActuatorCfg_N5010_16(UnitreeActuatorCfg):
    X1 = 27.0
    X2 = 41.5
    Y1 = 9.5
    Y2 = 17.0

    """
    | rotor  | 0.084e-4 kg·m²
    | gear_1 | 0.015e-4 kg·m² | ratio | 4
    | gear_2 | 0.068e-4 kg·m² | ratio | 4
    """
    armature = 0.0021812


@configclass
class UnitreeActuatorCfg_N5020_16(UnitreeActuatorCfg):
    X1 = 30.86
    X2 = 40.13
    Y1 = 24.8
    Y2 = 31.9

    Fs = 0.6
    Fd = 0.06

    """
    | rotor  | 0.139e-4 kg·m²
    | gear_1 | 0.017e-4 kg·m² | ratio | 46/18+1
    | gear_2 | 0.169e-4 kg·m² | ratio | 56/16+1
    """
    armature = 0.003609725


@configclass
class UnitreeActuatorCfg_W4010_25(UnitreeActuatorCfg):
    X1 = 15.3
    X2 = 24.76
    Y1 = 4.8
    Y2 = 8.6

    Fs = 0.6
    Fd = 0.06

    """
    | rotor  | 0.068e-4 kg·m²
    | gear_1 |                | ratio | 5
    | gear_2 |                | ratio | 5
    """
    armature = 0.00425


# ============================================================================
# MLP Actuator Network (Trained from Gazebo Data)
# ============================================================================


class MLPActuatorNetwork(DelayedPDActuator):
    """
    MLP-based actuator network trained on Gazebo motor dynamics data.

    This actuator uses a neural network to predict realistic motor torques
    based on joint position error, velocity, and command history.
    Enables better sim2sim transfer from Isaac Lab to Gazebo.
    """

    cfg: MLPActuatorNetworkCfg

    def __init__(self, cfg: MLPActuatorNetworkCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)

        # Load trained MLP model
        model_path = Path(cfg.network_file)
        if not model_path.exists():
            raise FileNotFoundError(
                f"MLP actuator model not found: {model_path}\n"
                f"Please ensure the model file exists at the specified path."
            )

        print(f"[MLP Actuator] Loading model from: {model_path}")
        self.model = torch.jit.load(str(model_path))

        # Move model to the correct device (same as computed_effort)
        device = self.computed_effort.device
        self.model = self.model.to(device)
        self.model.eval()

        # Initialize history buffers for the 6 input features
        # Features: [pos_err, pos_err_t-1, pos_err_t-2, vel, vel_t-1, vel_t-2]
        num_joints = self.num_joints
        num_envs = self.computed_effort.shape[0]  # Get num_envs from parent class tensor

        # Use zeros_like to ensure correct device and dtype
        template_tensor = self.computed_effort.unsqueeze(-1).expand(-1, -1, 3)
        self.pos_err_history = torch.zeros_like(template_tensor)
        self.vel_history = torch.zeros_like(template_tensor)

        print(f"[MLP Actuator] Initialized with {num_joints} joints for {num_envs} environments")

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        """
        Compute actuator torques using the MLP network.

        Args:
            control_action: Desired joint positions from the policy
            joint_pos: Current joint positions
            joint_vel: Current joint velocities

        Returns:
            ArticulationActions with predicted torques
        """
        # Compute position error
        if control_action.joint_positions is not None:
            pos_error = control_action.joint_positions - joint_pos
        else:
            pos_error = torch.zeros_like(joint_pos)

        # Update history buffers (shift and add new values)
        self.pos_err_history[:, :, 2] = self.pos_err_history[:, :, 1]  # t-2 = t-1
        self.pos_err_history[:, :, 1] = self.pos_err_history[:, :, 0]  # t-1 = t
        self.pos_err_history[:, :, 0] = pos_error                       # t = new

        self.vel_history[:, :, 2] = self.vel_history[:, :, 1]
        self.vel_history[:, :, 1] = self.vel_history[:, :, 0]
        self.vel_history[:, :, 0] = joint_vel

        # Prepare input features: [pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]
        batch_size = joint_pos.shape[0]
        num_joints = joint_pos.shape[1]

        mlp_input = torch.cat([
            self.pos_err_history[:, :, 0].unsqueeze(-1),  # pos_err(t)
            self.pos_err_history[:, :, 1].unsqueeze(-1),  # pos_err(t-1)
            self.pos_err_history[:, :, 2].unsqueeze(-1),  # pos_err(t-2)
            self.vel_history[:, :, 0].unsqueeze(-1),      # vel(t)
            self.vel_history[:, :, 1].unsqueeze(-1),      # vel(t-1)
            self.vel_history[:, :, 2].unsqueeze(-1),      # vel(t-2)
        ], dim=-1)  # Shape: (batch_size, num_joints, 6)

        # Reshape for MLP: (batch_size * num_joints, 6)
        mlp_input_flat = mlp_input.reshape(-1, 6)

        # Run inference
        with torch.no_grad():
            predicted_torque_flat = self.model(mlp_input_flat)

        # Reshape back: (batch_size, num_joints)
        predicted_torque = predicted_torque_flat.reshape(batch_size, num_joints)

        # Clip torques to effort limits
        effort_limits = self._parse_joint_parameter(self.cfg.effort_limit, 1e9)
        predicted_torque = torch.clip(predicted_torque, -effort_limits, effort_limits)

        # Set the computed torques
        self.applied_effort = predicted_torque

        # Return action with only torques (no position/velocity control)
        control_action.joint_positions = None
        control_action.joint_velocities = None
        control_action.joint_efforts = predicted_torque

        return control_action


@configclass
class MLPActuatorNetworkCfg(DelayedPDActuatorCfg):
    """
    Configuration for MLP-based actuator network trained from Gazebo data.

    This configuration is used for Go2 robot with actuator dynamics learned
    from real Gazebo simulations using chirp excitation.
    """

    class_type: type = MLPActuatorNetwork

    # Path to trained MLP model (TorchScript)
    network_file: str = MISSING
    """Path to the trained MLP model (.pth file)"""

    # PD gains used during Gazebo training (for reference)
    stiffness = {".*": 25.0}
    damping = {".*": 0.5}

    # Actuator limits (from Go2 specifications)
    effort_limit = {
        ".*_hip_joint": 23.7,
        ".*_thigh_joint": 23.7,
        ".*_calf_joint": 45.43,
    }
    velocity_limit = {
        ".*_hip_joint": 30.1,
        ".*_thigh_joint": 30.1,
        ".*_calf_joint": 15.70,
    }


@configclass
class UnitreeActuatorCfg_Go2_MLP(MLPActuatorNetworkCfg):
    """
    Go2 MLP actuator configuration using Gazebo-trained model.

    This uses an MLP network trained on Gazebo motor dynamics to provide
    realistic actuator behavior for sim2sim transfer.
    """

    # Path to the trained model
    network_file = os.path.join(
        os.path.dirname(__file__),
        "..",
        "actuator_models",
        "actuator_mlp.pth"
    )


# ============================================================================
# LSTM Actuator Network (Trained from Gazebo Data)
# ============================================================================


class LSTMActuatorNetwork(DelayedPDActuator):
    """
    LSTM-based actuator network trained on Gazebo motor dynamics data.

    This actuator uses an LSTM network to predict realistic motor torques.
    Uses MLP-style input with explicit history (6 features):
        [pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]

    This approach doesn't rely on LSTM hidden state persistence, making it
    robust for online inference where hidden states reset to zero each call.
    """

    cfg: LSTMActuatorNetworkCfg

    def __init__(self, cfg: LSTMActuatorNetworkCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)

        # Load trained LSTM model
        model_path = Path(cfg.network_file)
        if not model_path.exists():
            raise FileNotFoundError(
                f"LSTM actuator model not found: {model_path}\n"
                f"Please ensure the model file exists at the specified path."
            )

        print(f"[LSTM Actuator] Loading model from: {model_path}")
        self.model = torch.jit.load(str(model_path))

        # Move model to the correct device (same as computed_effort)
        device = self.computed_effort.device
        self.model = self.model.to(device)
        self.model.eval()

        # Get dimensions
        num_joints = self.num_joints
        num_envs = self.computed_effort.shape[0]

        # Initialize history buffers for the 6 input features (same as MLP)
        # Features: [pos_err, pos_err_t-1, pos_err_t-2, vel, vel_t-1, vel_t-2]
        template_tensor = self.computed_effort.unsqueeze(-1).expand(-1, -1, 3)
        self.pos_err_history = torch.zeros_like(template_tensor)
        self.vel_history = torch.zeros_like(template_tensor)

        print(f"[LSTM Actuator] Initialized with {num_joints} joints for {num_envs} environments")
        print(f"[LSTM Actuator] Using MLP-style 6-feature input with history")

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        """
        Compute actuator torques using the LSTM network with MLP-style history input.

        Args:
            control_action: Desired joint positions from the policy
            joint_pos: Current joint positions
            joint_vel: Current joint velocities

        Returns:
            ArticulationActions with predicted torques
        """
        # Compute position error
        if control_action.joint_positions is not None:
            pos_error = control_action.joint_positions - joint_pos
        else:
            pos_error = torch.zeros_like(joint_pos)

        batch_size = joint_pos.shape[0]
        num_joints = joint_pos.shape[1]

        # Update history buffers (shift and add new values)
        self.pos_err_history[:, :, 2] = self.pos_err_history[:, :, 1]  # t-2 = t-1
        self.pos_err_history[:, :, 1] = self.pos_err_history[:, :, 0]  # t-1 = t
        self.pos_err_history[:, :, 0] = pos_error                       # t = new

        self.vel_history[:, :, 2] = self.vel_history[:, :, 1]
        self.vel_history[:, :, 1] = self.vel_history[:, :, 0]
        self.vel_history[:, :, 0] = joint_vel

        # Build input features: [pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]
        # Shape: (batch_size, num_joints, 6)
        lstm_input = torch.cat([
            self.pos_err_history[:, :, 0].unsqueeze(-1),  # pos_err(t)
            self.pos_err_history[:, :, 1].unsqueeze(-1),  # pos_err(t-1)
            self.pos_err_history[:, :, 2].unsqueeze(-1),  # pos_err(t-2)
            self.vel_history[:, :, 0].unsqueeze(-1),      # vel(t)
            self.vel_history[:, :, 1].unsqueeze(-1),      # vel(t-1)
            self.vel_history[:, :, 2].unsqueeze(-1),      # vel(t-2)
        ], dim=-1)

        # Reshape for model: (batch_size * num_joints, 6)
        lstm_input_flat = lstm_input.reshape(batch_size * num_joints, 6)

        # Run LSTM inference
        with torch.no_grad():
            # Model takes (batch, 6) input, handles hidden states internally
            predicted_torque_flat = self.model(lstm_input_flat)

        # Reshape back: (batch_size, num_joints)
        if predicted_torque_flat.dim() == 2:
            predicted_torque = predicted_torque_flat.squeeze(-1).reshape(batch_size, num_joints)
        else:
            predicted_torque = predicted_torque_flat.reshape(batch_size, num_joints)

        # Clip torques to effort limits
        effort_limits = self._parse_joint_parameter(self.cfg.effort_limit, 1e9)
        predicted_torque = torch.clip(predicted_torque, -effort_limits, effort_limits)

        # Set the computed torques
        self.applied_effort = predicted_torque

        # Return action with only torques (no position/velocity control)
        control_action.joint_positions = None
        control_action.joint_velocities = None
        control_action.joint_efforts = predicted_torque

        return control_action

    def reset_idx(self, env_ids: torch.Tensor):
        """Reset history buffers for specific environments."""
        super().reset_idx(env_ids)
        # Reset history buffers for the specified environments
        self.pos_err_history[env_ids] = 0.0
        self.vel_history[env_ids] = 0.0


@configclass
class LSTMActuatorNetworkCfg(DelayedPDActuatorCfg):
    """
    Configuration for LSTM-based actuator network trained from Gazebo data.

    This configuration is used for Go2 robot with actuator dynamics learned
    from real Gazebo simulations using chirp excitation.

    Input features (MLP-style with history):
        [pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]

    Uses explicit history in input, so doesn't rely on LSTM hidden state persistence.
    """

    class_type: type = LSTMActuatorNetwork

    # Path to trained LSTM model (TorchScript)
    network_file: str = MISSING
    """Path to the trained LSTM model (.pth file)"""

    # LSTM architecture parameters (for reference, must match trained model)
    hidden_size: int = 64
    """LSTM hidden state size"""

    num_layers: int = 2
    """Number of LSTM layers"""

    # PD gains used during Gazebo training (for reference)
    stiffness = {".*": 25.0}
    damping = {".*": 0.5}

    # Actuator limits (from Go2 specifications)
    effort_limit = {
        ".*_hip_joint": 23.7,
        ".*_thigh_joint": 23.7,
        ".*_calf_joint": 45.43,
    }
    velocity_limit = {
        ".*_hip_joint": 30.1,
        ".*_thigh_joint": 30.1,
        ".*_calf_joint": 15.70,
    }


@configclass
class UnitreeActuatorCfg_Go2_LSTM(LSTMActuatorNetworkCfg):
    """
    Go2 LSTM actuator configuration using Gazebo-trained model.

    This uses an LSTM network trained on Gazebo motor dynamics to provide
    realistic actuator behavior with temporal modeling for sim2sim transfer.
    """

    # Path to the trained LSTM model
    network_file = os.path.join(
        os.path.dirname(__file__),
        "..",
        "actuator_models",
        "actuator_lstm.pth"
    )
