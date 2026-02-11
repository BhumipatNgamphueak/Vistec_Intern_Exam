#!/bin/bash

# Continue training Implicit-DR from checkpoint model_14600.pt to 25000 iterations

echo "============================================"
echo "Resuming Training: Implicit Actuator + DR"
echo "============================================"
echo "Task: Unitree-Go2-Velocity-Implicit-DR"
echo "Run: 2026-02-05_18-14-00"
echo "Checkpoint: model_14600.pt"
echo "Target: 25000 iterations (10400 remaining)"
echo "============================================"

python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --num_envs 2048 \
  --max_iterations 25000 \
  --headless \
  --resume \
  --load_run 2026-02-05_18-14-00 \
  --checkpoint model_14600.pt

echo ""
echo "============================================"
echo "Training Complete!"
echo "============================================"
echo "Final checkpoint: model_25000.pt"
echo "Location: logs/rsl_rl/unitree_go2_velocity_implicit_dr/2026-02-05_18-14-00/"
echo "============================================"
