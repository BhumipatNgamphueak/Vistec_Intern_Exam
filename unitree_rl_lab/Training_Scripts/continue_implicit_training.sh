#!/bin/bash
# Continue training Implicit policies from existing checkpoints

set -e

echo "========================================="
echo "Continue Training Implicit Policies"
echo "========================================="
echo ""

# Configuration
NUM_ENVS=4096  # Adjust based on your GPU memory

echo "Policy 1: Implicit (No DR)"
echo "  Current checkpoint: model_8100.pt (8100 iterations)"
echo "  Task: Unitree-Go2-Velocity-Implicit"
echo "  Actuator: IdealPD (Kp=160.0, Kd=5.0)"
echo ""

echo "Policy 2: Implicit + DR"
echo "  Current checkpoint: model_14600.pt (14600 iterations)"
echo "  Task: Unitree-Go2-Velocity-Implicit-DR"
echo "  Actuator: IdealPD (Kp=160.0, Kd=5.0)"
echo ""

# Ask user which one to train
echo "Which policy do you want to continue training?"
echo "  1) Implicit (No DR) - from iteration 8100"
echo "  2) Implicit + DR - from iteration 14600"
echo "  3) Both (run sequentially)"
echo ""
read -p "Enter choice (1/2/3): " choice

case $choice in
  1)
    echo ""
    echo "========================================="
    echo "Continuing: Implicit (No DR)"
    echo "========================================="

    python scripts/rsl_rl/train.py \
      --task Unitree-Go2-Velocity-Implicit \
      --num_envs $NUM_ENVS \
      --resume \
      --load_run 2026-02-05_16-27-42 \
      --headless

    echo "✅ Implicit (No DR) training continued!"
    ;;

  2)
    echo ""
    echo "========================================="
    echo "Continuing: Implicit + DR"
    echo "========================================="

    python scripts/rsl_rl/train.py \
      --task Unitree-Go2-Velocity-Implicit-DR \
      --num_envs $NUM_ENVS \
      --resume \
      --load_run 2026-02-05_18-14-00 \
      --headless

    echo "✅ Implicit + DR training continued!"
    ;;

  3)
    echo ""
    echo "========================================="
    echo "Training Both Policies Sequentially"
    echo "========================================="

    # Train Implicit (No DR) first
    echo ""
    echo "1/2: Continuing Implicit (No DR)..."
    python scripts/rsl_rl/train.py \
      --task Unitree-Go2-Velocity-Implicit \
      --num_envs $NUM_ENVS \
      --resume \
      --load_run 2026-02-05_16-27-42 \
      --headless

    echo "✅ Implicit (No DR) training completed!"
    echo ""

    # Then train Implicit + DR
    echo "2/2: Continuing Implicit + DR..."
    python scripts/rsl_rl/train.py \
      --task Unitree-Go2-Velocity-Implicit-DR \
      --num_envs $NUM_ENVS \
      --resume \
      --load_run 2026-02-05_18-14-00 \
      --headless

    echo "✅ Implicit + DR training completed!"
    echo ""
    echo "========================================="
    echo "Both policies training completed!"
    echo "========================================="
    ;;

  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

echo ""
echo "Training logs saved to:"
echo "  logs/rsl_rl/unitree_go2_velocity_implicit/"
echo "  logs/rsl_rl/unitree_go2_velocity_implicit_dr/"
echo ""
