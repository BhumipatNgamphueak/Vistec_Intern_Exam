#!/bin/bash

# Train LSTM Actuator with NO Domain Randomization
# For comparison with LSTM + DR to see the effect of domain randomization

echo "============================================"
echo "Training: LSTM Actuator (No DR)"
echo "============================================"
echo "Task: Unitree-Go2-Velocity-LSTM-No-DR"
echo "Environments: 2048"
echo "Iterations: 25000"
echo "============================================"

python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-LSTM-No-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless

echo ""
echo "============================================"
echo "Training Complete!"
echo "============================================"
echo "Logs saved to: logs/rsl_rl/unitree_go2_velocity_lstm_no_dr/"
echo ""
echo "To play the trained policy:"
echo "python scripts/rsl_rl/play.py \\"
echo "  --task Unitree-Go2-Velocity-LSTM-No-DR \\"
echo "  --num_envs 32 \\"
echo "  --load_run <TIMESTAMP> \\"
echo "  --checkpoint model_25000.pt"
echo "============================================"

