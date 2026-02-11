#!/bin/bash
# Continue training: Implicit (No DR)
# Current: 8100 iterations
# Target: 25000 iterations (typical for implicit policies)

set -e

echo "========================================="
echo "Continue Training: Implicit (No DR)"
echo "========================================="
echo ""
echo "Configuration:"
echo "  Task: Unitree-Go2-Velocity-Implicit"
echo "  Run folder: 2026-02-05_16-27-42"
echo "  Current checkpoint: model_8100.pt"
echo "  Current iteration: 8100"
echo "  Target iteration: ~25000"
echo ""
echo "  Actuator: IdealPD"
echo "  PD Gains: Kp=160.0, Kd=5.0"
echo "  Domain Randomization: DISABLED"
echo ""
echo "Starting training..."
echo ""

python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 4096 \
  --resume \
  --load_run 2026-02-05_16-27-42 \
  --headless

echo ""
echo "========================================="
echo "Training Complete!"
echo "========================================="
echo ""
echo "New checkpoints saved to:"
echo "  logs/rsl_rl/unitree_go2_velocity_implicit/2026-02-05_16-27-42/"
echo ""
