#!/bin/bash
# Continue training: Implicit + DR
# Current: 14600 iterations
# Target: 25000 iterations (typical for implicit policies)

set -e

echo "========================================="
echo "Continue Training: Implicit + DR"
echo "========================================="
echo ""
echo "Configuration:"
echo "  Task: Unitree-Go2-Velocity-Implicit-DR"
echo "  Run folder: 2026-02-05_18-14-00"
echo "  Current checkpoint: model_14600.pt"
echo "  Current iteration: 14600"
echo "  Target iteration: ~25000"
echo ""
echo "  Actuator: IdealPD"
echo "  PD Gains: Kp=160.0, Kd=5.0"
echo "  Domain Randomization: ENABLED"
echo "    - Random mass (base)"
echo "    - Random friction"
echo "    - Random motor strength"
echo "    - Push disturbances"
echo "    - External forces"
echo ""
echo "Starting training..."
echo ""

python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --num_envs 4096 \
  --resume \
  --load_run 2026-02-05_18-14-00 \
  --headless

echo ""
echo "========================================="
echo "Training Complete!"
echo "========================================="
echo ""
echo "New checkpoints saved to:"
echo "  logs/rsl_rl/unitree_go2_velocity_implicit_dr/2026-02-05_18-14-00/"
echo ""
