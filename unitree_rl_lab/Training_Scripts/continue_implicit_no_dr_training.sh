#!/bin/bash

# Continue training Implicit (No DR) from checkpoint model_8100.pt to 25000 iterations

echo "============================================"
echo "Resuming Training: Implicit Actuator (No DR)"
echo "============================================"
echo "Task: Unitree-Go2-Velocity-Implicit"
echo "Run: 2026-02-05_16-27-42"
echo "Checkpoint: model_8100.pt"
echo "Target: 25000 iterations (16900 remaining)"
echo "============================================"

python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless \
  --resume \
  --load_run 2026-02-05_16-27-42 \
  --checkpoint model_8100.pt

echo ""
echo "============================================"
echo "Training Complete!"
echo "============================================"
echo "Final checkpoint: model_25000.pt"
echo "Location: logs/rsl_rl/unitree_go2_velocity_implicit/2026-02-05_16-27-42/"
echo "============================================"
