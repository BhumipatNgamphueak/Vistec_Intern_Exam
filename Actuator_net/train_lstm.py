#!/usr/bin/env python3
"""
Train LSTM Actuator Model for Isaac Lab

This script trains an LSTM network to model actuator dynamics from Gazebo data.
Uses same 6 input features as MLP for compatibility with Isaac Lab.

Input: [pos_err, pos_err_t-1, pos_err_t-2, vel, vel_t-1, vel_t-2]
Output: Predicted motor torque

Data from: Gazebo data collection with chirp commander
"""

import torch
import torch.nn as nn
import numpy as np
from torch.utils.data import Dataset, DataLoader
import pickle
import matplotlib.pyplot as plt
from pathlib import Path

# Configuration
HIDDEN_DIM = 64   # LSTM hidden size
NUM_LAYERS = 2    # Number of LSTM layers
BATCH_SIZE = 128
LEARNING_RATE = 1e-3
EPOCHS = 500
PATIENCE = 15
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'


class LSTMActuatorNet(nn.Module):
    """
    LSTM network for actuator dynamics prediction.

    Uses same 6 input features as MLP:
        [pos_err, pos_err_t-1, pos_err_t-2, vel, vel_t-1, vel_t-2]

    Input shape: (batch, 6) - single timestep with history embedded in features
    Output shape: (batch, 1) - predicted torque
    """

    def __init__(self, input_dim=6, hidden_dim=64, num_layers=2, output_dim=1):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers

        # LSTM layers - input is (batch, 1, 6) after unsqueeze
        self.lstm = nn.LSTM(
            input_dim,
            hidden_dim,
            num_layers,
            batch_first=True,
            dropout=0.1 if num_layers > 1 else 0.0
        )

        # Output layers
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(0.1)
        self.fc2 = nn.Linear(hidden_dim // 2, output_dim)

    def forward(self, x):
        """
        Forward pass.

        Args:
            x: (batch, 6) - 6 input features (same as MLP)

        Returns:
            output: (batch, 1)
        """
        # Add sequence dimension: (batch, 6) -> (batch, 1, 6)
        if x.dim() == 2:
            x = x.unsqueeze(1)

        # LSTM forward
        lstm_out, (hidden, cell) = self.lstm(x)

        # Take last timestep output (only one timestep, so index 0 or -1)
        last_output = lstm_out[:, -1, :]

        # Fully connected layers
        out = self.fc1(last_output)
        out = self.relu(out)
        out = self.dropout(out)
        out = self.fc2(out)

        return out


class SimpleDataset(Dataset):
    """Dataset for LSTM with 6 input features (same as MLP)."""

    def __init__(self, motor_data, mode='train'):
        """
        Create dataset from processed motor data.

        Args:
            motor_data: Dict with 'input_data' and 'output_data'
            mode: 'train', 'val', or 'test'
        """
        self.mode = mode

        # Extract input features and output labels
        # input_data shape: (N, 6) - [pos_err, pos_err_t-1, pos_err_t-2, vel, vel_t-1, vel_t-2]
        self.inputs = motor_data['input_data'].astype(np.float32)  # (N, 6)
        self.targets = motor_data['output_data'].astype(np.float32)  # (N, 1)

        print(f"{mode.upper()} dataset: {len(self.inputs)} samples")
        print(f"  Input shape: {self.inputs.shape}")
        print(f"  Target shape: {self.targets.shape}")

    def __len__(self):
        return len(self.inputs)

    def __getitem__(self, idx):
        return (
            torch.FloatTensor(self.inputs[idx]),
            torch.FloatTensor(self.targets[idx])
        )


def load_and_split_data(data_path):
    """Load data and split into train/val/test."""

    print(f"Loading data from: {data_path}")
    with open(data_path, 'rb') as f:
        motor_data = pickle.load(f)

    print(f"Raw data shape: input={motor_data['input_data'].shape}, "
          f"output={motor_data['output_data'].shape}")

    # Split indices
    n_samples = len(motor_data['input_data'])
    train_end = int(0.8 * n_samples)
    val_end = int(0.9 * n_samples)

    # Create split datasets
    train_data = {
        'input_data': motor_data['input_data'][:train_end],
        'output_data': motor_data['output_data'][:train_end]
    }

    val_data = {
        'input_data': motor_data['input_data'][train_end:val_end],
        'output_data': motor_data['output_data'][train_end:val_end]
    }

    test_data = {
        'input_data': motor_data['input_data'][val_end:],
        'output_data': motor_data['output_data'][val_end:]
    }

    # Create datasets
    train_dataset = SimpleDataset(train_data, mode='train')
    val_dataset = SimpleDataset(val_data, mode='val')
    test_dataset = SimpleDataset(test_data, mode='test')

    return train_dataset, val_dataset, test_dataset


def train_epoch(model, dataloader, criterion, optimizer, device):
    """Train for one epoch."""
    model.train()
    total_loss = 0.0

    for batch_idx, (sequences, targets) in enumerate(dataloader):
        sequences = sequences.to(device)
        targets = targets.to(device)

        # Forward
        optimizer.zero_grad()
        outputs = model(sequences)
        loss = criterion(outputs, targets)

        # Backward
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()

        total_loss += loss.item()

        # Print progress every 50 batches
        if batch_idx % 50 == 0:
            print(f"  Batch {batch_idx}/{len(dataloader)}", end='\r', flush=True)

    print()  # New line after progress
    return total_loss / len(dataloader)


def validate(model, dataloader, criterion, device):
    """Validate model."""
    model.eval()
    total_loss = 0.0

    with torch.no_grad():
        for sequences, targets in dataloader:
            sequences = sequences.to(device)
            targets = targets.to(device)

            outputs = model(sequences)
            loss = criterion(outputs, targets)

            total_loss += loss.item()

    return total_loss / len(dataloader)


def test_model(model, dataloader, device):
    """Test model and compute metrics."""
    model.eval()
    predictions = []
    targets_list = []

    with torch.no_grad():
        for sequences, targets in dataloader:
            sequences = sequences.to(device)
            outputs = model(sequences)

            predictions.extend(outputs.cpu().numpy())
            targets_list.extend(targets.numpy())

    predictions = np.array(predictions).flatten()
    targets_list = np.array(targets_list).flatten()

    # Compute metrics
    mse = np.mean((predictions - targets_list) ** 2)
    rmse = np.sqrt(mse)
    r2 = 1 - (np.sum((targets_list - predictions) ** 2) /
              np.sum((targets_list - np.mean(targets_list)) ** 2))

    return {
        'mse': mse,
        'rmse': rmse,
        'r2': r2,
        'predictions': predictions,
        'targets': targets_list
    }


def plot_results(results, save_path):
    """Plot test results."""
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))

    # Predicted vs Actual
    axes[0, 0].scatter(results['targets'], results['predictions'], alpha=0.5, s=1)
    axes[0, 0].plot([results['targets'].min(), results['targets'].max()],
                     [results['targets'].min(), results['targets'].max()],
                     'r--', lw=2)
    axes[0, 0].set_xlabel('Actual Torque (N·m)')
    axes[0, 0].set_ylabel('Predicted Torque (N·m)')
    axes[0, 0].set_title(f'Predicted vs Actual (R²={results["r2"]:.4f})')
    axes[0, 0].grid(True, alpha=0.3)

    # Residuals
    residuals = results['predictions'] - results['targets']
    axes[0, 1].scatter(results['targets'], residuals, alpha=0.5, s=1)
    axes[0, 1].axhline(y=0, color='r', linestyle='--', lw=2)
    axes[0, 1].set_xlabel('Actual Torque (N·m)')
    axes[0, 1].set_ylabel('Residual (N·m)')
    axes[0, 1].set_title(f'Residuals (RMSE={results["rmse"]:.4f})')
    axes[0, 1].grid(True, alpha=0.3)

    # Residual histogram
    axes[1, 0].hist(residuals, bins=50, edgecolor='black', alpha=0.7)
    axes[1, 0].set_xlabel('Residual (N·m)')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].set_title('Residual Distribution')
    axes[1, 0].grid(True, alpha=0.3)

    # Time series (first 1000 points)
    n_plot = min(1000, len(results['targets']))
    axes[1, 1].plot(results['targets'][:n_plot], label='Actual', alpha=0.7)
    axes[1, 1].plot(results['predictions'][:n_plot], label='Predicted', alpha=0.7)
    axes[1, 1].set_xlabel('Sample')
    axes[1, 1].set_ylabel('Torque (N·m)')
    axes[1, 1].set_title('Time Series Comparison')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Results plot saved to: {save_path}")
    plt.close()


def main():
    print("="*70)
    print("LSTM Actuator Network Training")
    print("="*70)

    # Show selected dataset
    csv_link = Path('app/resources/actuator_data.csv')
    if csv_link.is_symlink():
        target = csv_link.resolve()
        dataset_name = target.parent.name
        print(f"📊 Dataset: {dataset_name}")
        print(f"   → {target}")
    else:
        print("📊 Dataset: (default/direct file)")
    print()

    print(f"Device: {DEVICE}")
    print(f"Input features: 6 (same as MLP)")
    print(f"Hidden dimension: {HIDDEN_DIM}")
    print(f"Number of LSTM layers: {NUM_LAYERS}")
    print("="*70)

    # Load data
    data_path = Path('app/resources/motor_data.pkl')
    if not data_path.exists():
        print(f"ERROR: Data file not found: {data_path}")
        print("Please run: python3 process_dataset.py first")
        return

    train_dataset, val_dataset, test_dataset = load_and_split_data(data_path)

    # Create dataloaders
    # Use fewer workers to avoid deadlocks on CPU
    num_workers = 0 if DEVICE == 'cpu' else 4

    train_loader = DataLoader(
        train_dataset,
        batch_size=BATCH_SIZE,
        shuffle=True,
        num_workers=num_workers,
        pin_memory=True if DEVICE == 'cuda' else False
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=BATCH_SIZE,
        num_workers=num_workers,
        pin_memory=True if DEVICE == 'cuda' else False
    )

    test_loader = DataLoader(
        test_dataset,
        batch_size=BATCH_SIZE,
        num_workers=num_workers,
        pin_memory=True if DEVICE == 'cuda' else False
    )

    # Create model
    model = LSTMActuatorNet(
        input_dim=6,  # [pos_err, pos_err_t-1, pos_err_t-2, vel, vel_t-1, vel_t-2]
        hidden_dim=HIDDEN_DIM,
        num_layers=NUM_LAYERS,
        output_dim=1
    ).to(DEVICE)

    print(f"\nModel architecture:")
    print(model)
    print(f"\nTotal parameters: {sum(p.numel() for p in model.parameters()):,}")

    # Loss and optimizer
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=5, verbose=True
    )

    # Training loop
    print("\n" + "="*70)
    print("Training Started")
    print("="*70)

    best_val_loss = float('inf')
    patience_counter = 0
    train_losses = []
    val_losses = []

    for epoch in range(EPOCHS):
        # Train
        train_loss = train_epoch(model, train_loader, criterion, optimizer, DEVICE)
        train_losses.append(train_loss)

        # Validate
        val_loss = validate(model, val_loader, criterion, DEVICE)
        val_losses.append(val_loss)

        # Learning rate scheduling
        scheduler.step(val_loss)

        # Print progress
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"Epoch {epoch+1:3d}/{EPOCHS}: "
                  f"Train Loss={train_loss:.6f}, "
                  f"Val Loss={val_loss:.6f}", flush=True)

        # Early stopping
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0

            # Save best model
            torch.save(model.state_dict(), 'app/resources/actuator_lstm.pt')
            print(f"  → Best model saved (Val Loss={val_loss:.6f})", flush=True)
        else:
            patience_counter += 1
            if patience_counter >= PATIENCE:
                print(f"\nEarly stopping triggered after {epoch+1} epochs!")
                break

    print("\n" + "="*70)
    print("Training Complete!")
    print("="*70)

    # Load best model for testing
    model.load_state_dict(torch.load('app/resources/actuator_lstm.pt'))

    # Test
    print("\nTesting model...")
    results = test_model(model, test_loader, DEVICE)

    print(f"\nTest Results:")
    print(f"  MSE:  {results['mse']:.6f}")
    print(f"  RMSE: {results['rmse']:.6f}")
    print(f"  R²:   {results['r2']:.6f}")

    # Plot results
    plot_results(results, 'app/resources/lstm_results.png')

    # Export to TorchScript for Isaac Lab
    print("\nExporting to TorchScript...")
    model.eval()

    # Create example input: (batch, 6) - same as MLP
    example_input = torch.randn(1, 6).to(DEVICE)

    # Trace model
    traced_model = torch.jit.trace(model, example_input)
    traced_model.save('app/resources/actuator_lstm.pth')

    print(f"✓ TorchScript model saved to: app/resources/actuator_lstm.pth")

    # Save training history
    history = {
        'train_losses': train_losses,
        'val_losses': val_losses,
        'test_results': results,
        'config': {
            'input_dim': 6,
            'hidden_dim': HIDDEN_DIM,
            'num_layers': NUM_LAYERS,
            'batch_size': BATCH_SIZE,
            'learning_rate': LEARNING_RATE
        }
    }

    with open('app/resources/lstm_training_history.pkl', 'wb') as f:
        pickle.dump(history, f)

    print(f"✓ Training history saved to: app/resources/lstm_training_history.pkl")

    print("\n" + "="*70)
    print("LSTM Actuator Model Ready for Isaac Lab!")
    print("="*70)
    print("\nNext steps:")
    print("1. Copy actuator_lstm.pth to your Isaac Lab workspace")
    print("2. Configure LSTMActuatorCfg in unitree_actuators.py")
    print("3. Train RL policy with realistic actuator dynamics")
    print("="*70)


if __name__ == '__main__':
    main()
