#!/bin/bash
# Play any trained policy (works with MLP, LSTM, Implicit)

set -e

echo "=========================================="
echo "Play Trained Policies"
echo "=========================================="
echo ""
echo "Select policy type:"
echo "1) MLP with DR (best for testing)"
echo "2) Implicit with DR (physics-based)"
echo "3) Implicit No DR"
echo "4) LSTM No DR (may have issues)"
echo "5) Default Velocity (original)"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        echo ""
        echo "Playing: MLP with Domain Randomization"
        echo "Task: Unitree-Go2-Velocity-MLP-Custom"
        echo "Run: 2026-02-03_15-54-07_work_good (24,999 iterations)"
        echo ""
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-MLP-Custom \
            --num_envs 1 \
            --load_run 2026-02-03_15-54-07_work_good
        ;;
    2)
        echo ""
        echo "Playing: Implicit with Domain Randomization"
        echo "Task: Unitree-Go2-Velocity-Implicit-DR"
        echo "Run: 2026-02-10_13-22-29 (18,400 iterations)"
        echo ""
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-Implicit-DR \
            --num_envs 32 \
            --load_run 2026-02-10_13-22-29
        ;;
    3)
        echo ""
        echo "Playing: Implicit No DR"
        echo "Task: Unitree-Go2-Velocity-Implicit"
        echo "Run: 2026-02-10_13-22-31 (12,100 iterations)"
        echo ""
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-Implicit \
            --num_envs 32 \
            --load_run 2026-02-10_13-22-31
        ;;
    4)
        echo ""
        echo "Playing: LSTM No DR"
        echo "Task: Unitree-Go2-Velocity-LSTM-No-DR"
        echo "Run: 2026-02-07_22-16-50 (9,900 iterations)"
        echo "NOTE: May have observation structure issues"
        echo ""
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-LSTM-No-DR \
            --num_envs 32 \
            --load_run 2026-02-07_22-16-50
        ;;
    5)
        echo ""
        echo "Playing: Default Velocity Policy"
        echo "Task: Unitree-Go2-Velocity"
        echo ""
        # Find latest run
        LATEST_RUN=$(ls -td /home/drl-68/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity/* 2>/dev/null | head -1 | xargs basename)
        if [ -z "$LATEST_RUN" ]; then
            echo "No trained policy found!"
            exit 1
        fi
        echo "Run: $LATEST_RUN"
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity \
            --num_envs 32 \
            --load_run $LATEST_RUN
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo ""
echo "========================================"
echo "Playback Complete!"
echo "========================================"
