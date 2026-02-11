#!/bin/bash
# Continue training Implicit policies from latest checkpoints

echo "=========================================="
echo "Continue Training Implicit Policies"
echo "=========================================="
echo ""
echo "Select policy to continue:"
echo "1) Implicit (No DR) - from iteration 12100"
echo "2) Implicit with DR - from iteration 18400"
echo "3) Both policies (parallel)"
echo "4) Exit"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo "Continuing: Implicit (No DR) from iteration 12100"
        echo "Checkpoint: 2026-02-10_13-22-31/model_12100.pt"
        echo ""
        python scripts/rsl_rl/train.py \
            --task Unitree-Go2-Velocity-Implicit \
            --num_envs 4096 \
            --resume \
            --load_run 2026-02-10_13-22-31 \
            --headless \
            +save_interval=1000
        ;;
    2)
        echo ""
        echo "Continuing: Implicit with DR from iteration 18400"
        echo "Checkpoint: 2026-02-10_13-22-29/model_18400.pt"
        echo ""
        python scripts/rsl_rl/train.py \
            --task Unitree-Go2-Velocity-Implicit-DR \
            --num_envs 4096 \
            --resume \
            --load_run 2026-02-10_13-22-29 \
            --headless \
            +save_interval=1000
        ;;
    3)
        echo ""
        echo "Starting both policies in parallel..."
        echo ""

        # Start first policy in background
        echo "Starting Implicit (No DR)..."
        python scripts/rsl_rl/train.py \
            --task Unitree-Go2-Velocity-Implicit \
            --num_envs 2048 \
            --resume \
            --load_run 2026-02-10_13-22-31 \
            --headless \
            +save_interval=1000 &

        PID1=$!
        echo "  → PID: $PID1 (No DR)"

        sleep 5

        # Start second policy in background
        echo "Starting Implicit with DR..."
        python scripts/rsl_rl/train.py \
            --task Unitree-Go2-Velocity-Implicit-DR \
            --num_envs 2048 \
            --resume \
            --load_run 2026-02-10_13-22-29 \
            --headless \
            +save_interval=1000 &

        PID2=$!
        echo "  → PID: $PID2 (With DR)"

        echo ""
        echo "Both policies running in parallel!"
        echo "Note: Using 2048 envs each (4096 total) to avoid memory issues"
        echo ""
        echo "To stop both:"
        echo "  kill $PID1 $PID2"
        echo ""
        echo "Press Ctrl+C to stop monitoring..."

        # Wait for both processes
        wait $PID1 $PID2
        ;;
    4)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac
