#!/bin/bash
# Play trained LSTM policy with Domain Randomization

set -e

echo "=========================================="
echo "Play LSTM Policy (with DR)"
echo "=========================================="
echo ""
echo "Policy Information:"
echo "  Task: LSTM Actuator with Domain Randomization"
echo "  Checkpoint: model_25000.pt (25,000 iterations)"
echo "  Run: 2026-02-07_11-16-35"
echo "  PD Gains: Kp=25.0, Kd=0.5"
echo ""
echo "Select mode:"
echo "1) Visualization (32 envs, see robots in action)"
echo "2) Headless (32 envs, no visualization)"
echo "3) Record video (500 steps)"
echo "4) Large scale (128 envs, visualization)"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo "Starting visualization mode..."
        echo "Camera controls:"
        echo "  - Mouse drag: rotate view"
        echo "  - Mouse wheel: zoom"
        echo "  - Middle click + drag: pan"
        echo ""
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-LSTM-DR \
            --num_envs 32 \
            --load_run 2026-02-07_11-16-35 \
            --checkpoint /home/drl-68/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-07_11-16-35/model_25000.pt
        ;;
    2)
        echo ""
        echo "Starting headless mode..."
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-LSTM-DR \
            --num_envs 32 \
            --load_run 2026-02-07_11-16-35 \
            --checkpoint /home/drl-68/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-07_11-16-35/model_25000.pt \
            --headless
        ;;
    3)
        echo ""
        echo "Recording video (500 steps)..."
        echo "Video will be saved to logs directory"
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-LSTM-DR \
            --num_envs 32 \
            --load_run 2026-02-07_11-16-35 \
            --checkpoint /home/drl-68/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-07_11-16-35/model_25000.pt \
            --headless \
            --video \
            --video_length 500
        ;;
    4)
        echo ""
        echo "Starting large scale visualization (128 robots)..."
        echo "NOTE: This may be slower depending on GPU"
        python scripts/rsl_rl/play.py \
            --task Unitree-Go2-Velocity-LSTM-DR \
            --num_envs 128 \
            --load_run 2026-02-07_11-16-35 \
            --checkpoint /home/drl-68/unitree_rl_lab/logs/rsl_rl/unitree_go2_velocity_lstm_dr/2026-02-07_11-16-35/model_25000.pt
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
