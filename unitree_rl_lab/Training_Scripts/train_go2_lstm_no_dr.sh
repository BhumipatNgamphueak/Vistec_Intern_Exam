#!/bin/bash

# Training script for Go2 velocity tracking with LSTM actuator and NO domain randomization
# This is a baseline configuration for comparison experiments

set -e  # Exit on error

echo "================================================================"
echo "Training Go2 Velocity Tracking - LSTM Actuator, NO DR"
echo "================================================================"
echo ""
echo "Configuration:"
echo "  - Actuator: LSTM (temporal modeling)"
echo "  - Domain Randomization: DISABLED (baseline)"
echo "  - Observation Noise: DISABLED"
echo "  - Physics: Fixed parameters"
echo "  - Reset: Fixed positions"
echo ""
echo "This is a clean baseline for comparing against DR configurations."
echo "================================================================"
echo ""

# Navigate to Isaac Lab directory
cd ~/unitree_rl_lab

# Source Isaac Lab environment
source unitree_rl_lab.sh

# Run training with LSTM no-DR configuration
echo "Starting training..."
python scripts/rsl_rl/train.py \
    --task=unitree_go2_velocity_lstm_no_dr \
    --num_envs=4096 \
    --headless

echo ""
echo "================================================================"
echo "Training complete!"
echo "================================================================"
echo ""
echo "Model saved to:"
echo "  ~/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/<timestamp>/"
echo ""
echo "To evaluate the trained policy:"
echo "  cd ~/unitree_rl_lab"
echo "  source unitree_rl_lab.sh"
echo "  python scripts/rsl_rl/play.py \\"
echo "    --task=unitree_go2_velocity_lstm_no_dr \\"
echo "    --num_envs=32 \\"
echo "    --checkpoint=logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/<timestamp>/model_XXXX.pt"
