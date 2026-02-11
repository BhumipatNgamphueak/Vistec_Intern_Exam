#!/bin/bash
# Continue training Implicit policies WITH VISUALIZATION

set -e

echo "=========================================="
echo "Continue Training: Implicit (WITH VISUALIZATION)"
echo "=========================================="
echo ""
echo "Select policy to continue:"
echo "1) Implicit (No DR) - from iteration 12100"
echo "2) Implicit with DR - from iteration 18400"
echo ""
read -p "Enter choice [1-2]: " choice

case $choice in
    1)
        echo ""
        echo "Continuing: Implicit (No DR) from iteration 12100"
        echo "Checkpoint: 2026-02-10_13-22-31/model_12100.pt"
        echo ""
        echo "NOTE: Visualization will slow down training!"
        echo "      Use fewer environments (1024) for smoother rendering"
        echo ""
        python scripts/rsl_rl/train.py \
            --task Unitree-Go2-Velocity-Implicit \
            --num_envs 1024 \
            --resume \
            --load_run 2026-02-10_13-22-31 \
            +save_interval=1000
        ;;
    2)
        echo ""
        echo "Continuing: Implicit with DR from iteration 18400"
        echo "Checkpoint: 2026-02-10_13-22-29/model_18400.pt"
        echo ""
        echo "NOTE: Visualization will slow down training!"
        echo "      Using 1024 environments for smoother rendering"
        echo ""
        python scripts/rsl_rl/train.py \
            --task Unitree-Go2-Velocity-Implicit-DR \
            --num_envs 1024 \
            --resume \
            --load_run 2026-02-10_13-22-29 \
            +save_interval=1000
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "========================================"
echo "Training Complete!"
echo "========================================"
