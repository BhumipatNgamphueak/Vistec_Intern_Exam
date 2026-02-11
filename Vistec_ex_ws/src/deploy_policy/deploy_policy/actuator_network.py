#!/usr/bin/env python3
"""
MLP Actuator Network for Go2 Robot Deployment.

This module provides a PyTorch implementation of the trained MLP actuator network
that maps desired joint positions to actual torques, matching the actuator dynamics
learned from Gazebo simulation data.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Optional
import pickle


class Act(nn.Module):
    """Custom activation function module supporting multiple activation types."""

    def __init__(self, act: str, slope: float = 0.05):
        super(Act, self).__init__()
        self.act = act
        self.slope = slope
        self.shift = torch.log(torch.tensor(2.0)).item()

    def forward(self, input: torch.Tensor) -> torch.Tensor:
        if self.act == "relu":
            return F.relu(input)
        elif self.act == "leaky_relu":
            return F.leaky_relu(input)
        elif self.act == "sp":
            return F.softplus(input, beta=1.)
        elif self.act == "leaky_sp":
            return F.softplus(input, beta=1.) - self.slope * F.relu(-input)
        elif self.act == "elu":
            return F.elu(input, alpha=1.)
        elif self.act == "leaky_elu":
            return F.elu(input, alpha=1.) - self.slope * F.relu(-input)
        elif self.act == "ssp":
            return F.softplus(input, beta=1.) - self.shift
        elif self.act == "leaky_ssp":
            return (F.softplus(input, beta=1.) -
                   self.slope * F.relu(-input) -
                   self.shift)
        elif self.act == "tanh":
            return torch.tanh(input)
        elif self.act == "leaky_tanh":
            return torch.tanh(input) + self.slope * input
        elif self.act == "swish":
            return torch.sigmoid(input) * input
        elif self.act == "softsign":
            return F.softsign(input)
        else:
            raise RuntimeError(f"Undefined activation: {self.act}")


class ActuatorNet(nn.Module):
    """
    MLP Actuator Network.

    Maps 6 input features per joint to torque output:
    - Position error (current)
    - Position error (t-1)
    - Position error (t-2)
    - Velocity (current)
    - Velocity (t-1)
    - Velocity (t-2)
    """

    def __init__(
        self,
        in_dim: int = 6,
        units: int = 100,
        layers: int = 4,
        out_dim: int = 1,
        act: str = 'softsign',
        layer_norm: bool = False,
        act_final: bool = False
    ):
        super(ActuatorNet, self).__init__()

        # Build network layers
        mods = [nn.Linear(in_dim, units), Act(act)]
        for i in range(layers - 1):
            mods += [nn.Linear(units, units), Act(act)]
        mods += [nn.Linear(units, out_dim)]

        if act_final:
            mods += [Act(act)]
        if layer_norm:
            mods += [nn.LayerNorm(out_dim)]

        self.model = nn.Sequential(*mods)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.model(x)


class ActuatorNetworkRunner:
    """
    Actuator Network Runner for deployment.

    Manages the MLP actuator network inference, including history tracking
    and feature computation for converting target positions to torques.
    """

    def __init__(
        self,
        model_path: str,
        scaler_path: Optional[str] = None,
        num_joints: int = 12,
        device: str = "cpu"
    ):
        """
        Initialize the actuator network runner.

        Args:
            model_path: Path to trained actuator model (.pt or .pth)
            scaler_path: Path to scaler pickle file (optional)
            num_joints: Number of robot joints
            device: Device to run inference on
        """
        self.device = torch.device(device)
        self.num_joints = num_joints

        # Load model
        self.model = self._load_model(model_path)
        self.model.eval()

        # Load scaler if provided
        self.scaler = None
        self.use_scaling = False
        if scaler_path:
            self.scaler, self.use_scaling = self._load_scaler(scaler_path)

        # History buffers for each joint (6 features: 3 pos errors + 3 vels)
        self.pos_error_history = np.zeros((num_joints, 3))  # [current, t-1, t-2]
        self.vel_history = np.zeros((num_joints, 3))  # [current, t-1, t-2]

        print(f"[ActuatorNetworkRunner] Loaded model from: {model_path}")
        print(f"[ActuatorNetworkRunner] Device: {self.device}")
        print(f"[ActuatorNetworkRunner] Scaling: {'Enabled' if self.use_scaling else 'Disabled'}")
        print(f"[ActuatorNetworkRunner] Number of joints: {num_joints}")

    def _load_model(self, model_path: str) -> nn.Module:
        """Load the trained actuator network."""
        # Try loading as TorchScript first
        try:
            model = torch.jit.load(model_path, map_location=self.device)
            print(f"[ActuatorNetworkRunner] Loaded TorchScript model")
            return model
        except:
            pass

        # Load as state dict
        checkpoint = torch.load(model_path, map_location=self.device)

        # Create model with matching architecture
        model = ActuatorNet(
            in_dim=6,
            units=100,
            layers=4,
            out_dim=1,
            act='softsign'
        )

        # Load weights
        if isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
            model.load_state_dict(checkpoint['model_state_dict'])
        else:
            model.load_state_dict(checkpoint)

        model.to(self.device)
        print(f"[ActuatorNetworkRunner] Loaded PyTorch state dict")
        return model

    def _load_scaler(self, scaler_path: str):
        """Load the feature scaler."""
        try:
            with open(scaler_path, 'rb') as f:
                scaler_dict = pickle.load(f)
            use_scaling = scaler_dict.get('use_scale', False)
            scaler = scaler_dict.get('scaler', None)
            print(f"[ActuatorNetworkRunner] Loaded scaler from: {scaler_path}")
            return scaler, use_scaling
        except Exception as e:
            print(f"[ActuatorNetworkRunner] Warning: Could not load scaler: {e}")
            return None, False

    def reset(self):
        """Reset history buffers."""
        self.pos_error_history.fill(0.0)
        self.vel_history.fill(0.0)

    def compute_torques(
        self,
        target_positions: np.ndarray,
        current_positions: np.ndarray,
        current_velocities: np.ndarray
    ) -> np.ndarray:
        """
        Compute actuator torques from target and current states.

        Args:
            target_positions: Target joint positions (num_joints,)
            current_positions: Current joint positions (num_joints,)
            current_velocities: Current joint velocities (num_joints,)

        Returns:
            Predicted torques (num_joints,)
        """
        # Compute position errors
        pos_errors = target_positions - current_positions

        # Update history buffers (shift and insert new)
        self.pos_error_history[:, 2] = self.pos_error_history[:, 1]  # t-2 = t-1
        self.pos_error_history[:, 1] = self.pos_error_history[:, 0]  # t-1 = current
        self.pos_error_history[:, 0] = pos_errors  # current = new

        self.vel_history[:, 2] = self.vel_history[:, 1]
        self.vel_history[:, 1] = self.vel_history[:, 0]
        self.vel_history[:, 0] = current_velocities

        # Build features for all joints: [pos_err_0, pos_err_1, pos_err_2, vel_0, vel_1, vel_2]
        features = np.concatenate([
            self.pos_error_history,  # (num_joints, 3)
            self.vel_history         # (num_joints, 3)
        ], axis=1)  # (num_joints, 6)

        # Apply scaling if enabled
        if self.use_scaling and self.scaler is not None:
            features = self.scaler.transform(features.astype(np.float32))

        # Convert to tensor
        features_tensor = torch.from_numpy(features).float().to(self.device)

        # Run inference for all joints at once
        with torch.no_grad():
            torques_tensor = self.model(features_tensor)

        # Convert to numpy
        torques = torques_tensor.cpu().numpy().squeeze()

        # Handle single joint case
        if torques.ndim == 0:
            torques = np.array([torques.item()])

        return torques


if __name__ == "__main__":
    # Test the actuator network
    model_path = "/home/drl-68/actuator_net/app/resources/actuator.pth"
    scaler_path = "/home/drl-68/actuator_net/app/resources/scaler.pkl"

    runner = ActuatorNetworkRunner(
        model_path=model_path,
        scaler_path=scaler_path,
        num_joints=12,
        device="cpu"
    )

    # Test with dummy data
    target_pos = np.zeros(12)
    current_pos = np.random.randn(12) * 0.1
    current_vel = np.random.randn(12) * 0.5

    torques = runner.compute_torques(target_pos, current_pos, current_vel)
    print(f"\nTest torques: {torques}")
    print(f"Torque range: [{torques.min():.3f}, {torques.max():.3f}]")
