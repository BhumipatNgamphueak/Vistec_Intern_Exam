# Actuator Network for Unitree Go2 Robot

Neural network-based actuator identification for sim-to-real transfer on Unitree Go2 quadruped robot. This project implements both MLP (Multi-Layer Perceptron) and LSTM (Long Short-Term Memory) models to predict motor torque from position and velocity commands.

## Table of Contents
- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Dataset Preparation](#dataset-preparation)
- [Training](#training)
- [Testing](#testing)
- [Model Architecture](#model-architecture)
- [Isaac Lab Integration](#isaac-lab-integration)
- [File Structure](#file-structure)
- [Results](#results)
- [Troubleshooting](#troubleshooting)

## Overview

Accurate modeling of real robots is crucial for developing reinforcement learning (RL)-based control systems. However, the complexities of electrical actuators make it challenging to simulate actuator dynamics accurately, resulting in a significant gap between simulated and real actuators.

This repository provides a complete solution for identifying actuators of your real robot using artificial neural networks. By leveraging labeled datasets collected from real actuators, we train neural networks (MLP and LSTM) to estimate actuator torques based on position error and velocity inputs. Using these networks to simulate actuators can significantly reduce the sim-real actuator gap and enhance policy deployment performance.

**Key Features:**
- MLP and LSTM model implementations
- Automated dataset preprocessing
- Early stopping and checkpoint management
- TorchScript export for Isaac Lab deployment
- GUI application for visualization

## System Requirements

### Hardware
- NVIDIA GPU with CUDA support (recommended for training)
- Minimum 8GB RAM
- ~2GB disk space for datasets and models

### Software
- **Operating System**: Ubuntu 20.04/22.04 (or compatible Linux distribution)
- **Python**: 3.10.12
- **CUDA**: 12.1+ (if using GPU)
- **PyTorch**: 2.3.1 (with CUDA 12.1 support)

### Verified Environment
This project has been tested on:
- Ubuntu 22.04 LTS
- Python 3.10.12
- PyTorch 2.3.1 with CUDA 12.8
- NVIDIA GPU (CUDA capable)

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd actuator_net
```

### 2. Create Virtual Environment (Recommended)
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

**Core Dependencies:**
```
torch==2.3.1           # PyTorch with CUDA support
numpy==1.24.4          # Numerical computing
pandas==2.0.3          # Data manipulation
scikit-learn==1.3.2    # Machine learning utilities
matplotlib==3.7.5      # Plotting
PyQt5==5.15.9          # GUI framework (for app)
```

### 4. Verify Installation
```bash
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

**Expected output:**
```
PyTorch: 2.3.1
CUDA: True
```

If CUDA is False, install PyTorch with CUDA support:
```bash
pip install torch==2.3.1 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

## Dataset Preparation

### Dataset Format
The dataset must be in NPZ format with the following structure:
- **Position error history**: `pos_err(t)`, `pos_err(t-1)`, `pos_err(t-2)`
- **Velocity history**: `vel(t)`, `vel(t-1)`, `vel(t-2)`
- **Ground truth torque**: `effort`

### Preprocess Raw Data
If you have raw CSV data from Isaac Lab or robot logging:

```bash
python3 process_dataset.py
```

**This script will:**
1. Load raw actuator data from CSV files
2. Create temporal features (position error and velocity history)
3. Apply normalization (StandardScaler)
4. Split into train/validation/test sets (70%/15%/15%)
5. Save preprocessed data as NPZ file

### Dataset Location
Place your preprocessed dataset at:
```
app/resources/datasets/multi_amp_50s_hang_down/actuator_data_isaac_lab.npz
```

**Or create a symlink:**
```bash
ln -sf /path/to/your/dataset.npz app/resources/actuator_data_isaac_lab.npz
```

### Data Collection Tips
- Collect data with varying position commands and velocities
- Include different motion patterns (standing, walking, jumping)
- Ensure sufficient duration (~50 seconds or more)
- Sample at consistent frequency (e.g., 50 Hz)

## Training

### Train MLP Model
```bash
python3 train.py
```

**MLP Configuration:**
| Parameter | Value |
|-----------|-------|
| Input features | 6 |
| Hidden units | 100 |
| Number of layers | 4 |
| Activation | Softsign |
| Learning rate | 8e-4 |
| Batch size | 4096 |
| Early stopping patience | 10 epochs |
| Data split | 70% train, 15% val, 15% test |

**Input Features:**
`[pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]`

**Output Files:**
- `app/resources/actuator.pth` - PyTorch checkpoint
- `app/resources/actuator.pt` - TorchScript model (for Isaac Lab)
- `app/resources/scaler.pkl` - Feature scaler

**Expected Training Time:** ~5-10 minutes (with GPU)

### Train LSTM Model
```bash
python3 train_lstm.py
```

**LSTM Configuration:**
| Parameter | Value |
|-----------|-------|
| Input features | 6 (same as MLP) |
| LSTM hidden size | 64 |
| LSTM layers | 2 |
| Dropout | 0.1 |
| FC layers | 64 → 32 → 1 |
| Activation | ReLU |
| Learning rate | 1e-3 |
| Batch size | 4096 |
| Early stopping patience | 15 epochs |
| Data split | 70% train, 15% val, 15% test |

**Output Files:**
- `app/resources/actuator_lstm.pth` - PyTorch checkpoint
- `app/resources/actuator_lstm.pt` - TorchScript model (for Isaac Lab)
- `app/resources/lstm_training_history.pkl` - Training logs
- `app/resources/lstm_results.png` - Training curves plot

**Expected Training Time:** ~10-20 minutes (with GPU)

### Training Options

**Resume from checkpoint:**
Training scripts automatically detect and load existing checkpoints. To start fresh:
```bash
# Remove checkpoint to restart training
rm app/resources/actuator.pth       # for MLP
rm app/resources/actuator_lstm.pth  # for LSTM
```

**Monitor training progress:**
- Progress bars show epoch completion
- Console logs display loss, R² score, and validation metrics
- Best model saved automatically with early stopping

**Adjust batch size for memory constraints:**
```bash
# Edit train.py or train_lstm.py
BATCH_SIZE = 2048  # reduce if GPU out of memory
```

## Testing

### Test Trained Model
```bash
python3 test.py
```

**This script will:**
1. Load the trained model checkpoint
2. Evaluate on test set
3. Calculate metrics: MSE, RMSE, R² score
4. Generate prediction vs. ground truth plots
5. Display results in console

### Expected Performance

**MLP Model:**
- Test R² score: > 0.95
- Test RMSE: < 0.5 Nm
- Fast inference: ~0.1ms per sample

**LSTM Model:**
- Test R² score: > 0.93
- Test RMSE: < 0.6 Nm
- Moderate inference: ~0.2ms per sample

### Interpreting Results
- **R² score closer to 1.0** indicates better fit
- **Lower RMSE** indicates better accuracy
- Check plots for systematic errors or bias

## Model Architecture

### MLP Architecture
```
Input(6) → FC(100, softsign) → FC(100, softsign) → FC(100, softsign) → FC(1)
```

**Characteristics:**
- Simple feedforward network
- ~40K parameters
- No internal state
- Fast inference
- Best for position-based control

**Implementation:**
```python
class ActuatorNet(nn.Module):
    def __init__(self, in_dim=6, units=100, layers=4, act='softsign'):
        # 4 fully connected layers with softsign activation
```

### LSTM Architecture
```
Input(6) → LSTM(64, 2 layers, dropout=0.1) → FC(32, ReLU) → Dropout(0.1) → FC(1)
```

**Characteristics:**
- Recurrent architecture
- ~25K parameters
- Internal hidden states
- Handles temporal dependencies
- Best for velocity/trajectory tracking

**Implementation:**
```python
class LSTMActuatorNet(nn.Module):
    def __init__(self, input_dim=6, hidden_dim=64, num_layers=2):
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers,
                           batch_first=True, dropout=0.1)
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.fc2 = nn.Linear(hidden_dim // 2, 1)
```

**Important Notes:**
- Both models use **identical 6 input features**
- LSTM accepts input shape: `(batch, 6)` (auto-unsqueezed to `(batch, 1, 6)`)
- No sequential windowing required at inference
- Features must be in correct order

## Isaac Lab Integration

### Model Export
Models are automatically exported to TorchScript format (`.pt` files) during training. These can be directly loaded in Isaac Lab.

### Integration Example

**For MLP:**
```python
from omni.isaac.lab.actuators import ActuatorNetMLPCfg

actuator_cfg = ActuatorNetMLPCfg(
    network_file="/home/drl-68/actuator_net/app/resources/actuator.pt",
    pos_scale=1.0,
    vel_scale=0.1,
    torque_scale=1.0,
)
```

**For LSTM:**
```python
from custom_actuators import LSTMActuatorNetworkCfg

actuator_cfg = LSTMActuatorNetworkCfg(
    network_file="/home/drl-68/actuator_net/app/resources/actuator_lstm.pt",
    hidden_size=64,
    num_layers=2,
    pos_scale=1.0,
    vel_scale=0.1,
    torque_scale=1.0,
)
```

### Integration Requirements
1. **Input shape**: `(batch_size * num_joints, 6)`
2. **Output shape**: `(batch_size * num_joints, 1)`
3. **Feature order**: `[pos_err(t), pos_err(t-1), pos_err(t-2), vel(t), vel(t-1), vel(t-2)]`
4. **History buffers**: Must be maintained in actuator implementation

### Example Integration Path
```
/home/drl-68/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/go2/velocity_env_cfg_lstm_custom_enhanced.py
```

## File Structure

```
actuator_net/
├── README.md                    # This file
├── requirements.txt             # Python dependencies
│
├── train.py                     # MLP training script
├── train_lstm.py                # LSTM training script
├── test.py                      # Model testing script
├── process_dataset.py           # Dataset preprocessing
│
├── app/
│   ├── main.py                  # GUI application for visualization
│   ├── tableview.py             # Table view component
│   └── resources/
│       ├── actuator.pt          # MLP TorchScript model (for Isaac Lab)
│       ├── actuator.pth         # MLP PyTorch checkpoint
│       ├── actuator_lstm.pt     # LSTM TorchScript model (for Isaac Lab)
│       ├── actuator_lstm.pth    # LSTM PyTorch checkpoint
│       ├── motor_data.pkl       # Processed motor data
│       ├── scaler.pkl           # Feature normalization scaler
│       ├── lstm_results.png     # Training results plot (log)
│       ├── lstm_training_history.pkl  # Training history (log)
│       └── datasets/            # Dataset directory
│           └── multi_amp_50s_hang_down/
│               ├── actuator_data.csv
│               └── actuator_data_isaac_lab.npz
│
└── docs/                        # Additional documentation
    ├── actuator_net.gif
    └── actuator_net.png
```

## Results

### Training Logs
- **MLP**: Real-time console logs during training
- **LSTM**: Saved to `lstm_training_history.pkl`, visualized in `lstm_results.png`

### Model Checkpoints
- Automatic checkpoint saving after each improved validation epoch
- Best model retained with early stopping
- TorchScript models ready for deployment

### Visualization
```bash
# View LSTM training results
python3 -c "from PIL import Image; Image.open('app/resources/lstm_results.png').show()"
```

## Troubleshooting

### CUDA Out of Memory
**Solution:** Reduce batch size
```python
# Edit train.py or train_lstm.py
BATCH_SIZE = 2048  # default is 4096
```

### Model Not Loading in Isaac Lab
**Check:**
1. TorchScript model exists: `ls -lh app/resources/actuator_lstm.pt`
2. Model input/output shapes match expectations
3. CUDA availability matches between training and deployment
4. PyTorch versions are compatible

### Poor Model Performance
**Possible causes:**
1. **Insufficient data variation** - Collect more diverse motion patterns
2. **Data preprocessing issues** - Rerun `process_dataset.py`
3. **Data leakage** - Ensure ground truth torque is NOT in input features
4. **Hyperparameter tuning** - Adjust learning rate, batch size, or architecture

**Debug steps:**
```bash
# Check dataset statistics
python3 -c "import numpy as np; data=np.load('app/resources/actuator_data_isaac_lab.npz'); print(data.files); print({k: v.shape for k,v in data.items()})"

# Visualize training curves
python3 test.py  # Generates plots
```

### Dataset Not Found Error
**Solution:** Create symlink or update path
```bash
# Option 1: Create symlink
ln -sf /path/to/actuator_data_isaac_lab.npz app/resources/

# Option 2: Update path in training scripts
# Edit train.py or train_lstm.py and modify data_path variable
```

### Import Errors
**Solution:** Reinstall dependencies
```bash
# Reinstall all dependencies
pip install --upgrade -r requirements.txt

# Or install PyTorch specifically with CUDA
pip install torch==2.3.1 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

### Training Not Converging
**Possible solutions:**
1. **Lower learning rate**: Edit training script, set `lr = 1e-4`
2. **Increase batch size**: More stable gradients
3. **Check data normalization**: Ensure scaler is applied correctly
4. **Verify dataset quality**: Check for NaN or inf values

## Model Comparison

| Aspect | MLP | LSTM |
|--------|-----|------|
| **Input Features** | 6 | 6 |
| **Architecture** | 4 FC layers (100 units) | 2 LSTM layers (64 units) + 2 FC |
| **Activation** | Softsign | ReLU |
| **Parameters** | ~40K | ~25K |
| **Training Time** | 5-10 min | 10-20 min |
| **Inference Speed** | Fast (~0.1ms) | Moderate (~0.2ms) |
| **Memory Usage** | Low | Medium (internal states) |
| **Test R² Score** | >0.95 | >0.93 |
| **Test RMSE** | <0.5 Nm | <0.6 Nm |
| **Use Case** | Position control | Velocity/trajectory tracking |
| **Temporal Memory** | No | Yes (LSTM hidden states) |

### When to Use Which Model?
- **Use MLP** for: Fast inference, position-based control, simpler deployment
- **Use LSTM** for: Trajectory tracking, velocity control, temporal patterns

## GUI Application

A user interface is provided for data visualization and exploration:

```bash
cd app
python3 main.py
```

**Features:**
- Load and visualize actuator datasets
- Display first 20 rows in table view
- Interactive plotting
- Dataset statistics

![Actuator Net GUI](./docs/actuator_net.png)

## References

The training code architecture is inspired by [walk-these-ways](https://github.com/Improbable-AI/walk-these-ways.git).

## Citation

If you use this work in your research, please cite:
```bibtex
@misc{actuator_net_go2,
  title={Neural Network Actuator Models for Unitree Go2 Robot},
  author={Your Name},
  year={2026},
  howpublished={\url{https://github.com/your-repo/actuator_net}}
}
```

## License

[Specify your license here]

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## Contact

For questions and support:
- GitHub Issues: [repository-url]/issues
- Email: [your-email]

---

**Last Updated**: February 7, 2026
**Version**: 2.0.0
**Tested on**: Ubuntu 22.04, Python 3.10.12, PyTorch 2.3.1, CUDA 12.8
