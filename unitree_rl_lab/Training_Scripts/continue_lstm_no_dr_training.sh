#!/bin/bash

# Continue training LSTM-No-DR from checkpoint model_4100.pt
# This will resume training and save new checkpoints in the same run folder

echo "============================================"
echo "Resuming Training: LSTM Actuator (No DR)"
echo "============================================"
echo "Task: Unitree-Go2-Velocity-LSTM-No-DR"
echo "Run: 2026-02-06_18-49-02"
echo "Checkpoint: model_4100.pt"
echo "Target: 25000 iterations"
echo "============================================"

python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-LSTM-No-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless \
  --resume \
  --load_run 2026-02-06_18-49-02 \
  --checkpoint model_4100.pt

echo ""
echo "============================================"
echo "Training Complete!"
echo "============================================"

