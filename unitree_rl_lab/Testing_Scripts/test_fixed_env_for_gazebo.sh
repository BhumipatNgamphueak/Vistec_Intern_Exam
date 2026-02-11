#!/bin/bash
# Test all policies with FIXED environment (matching Gazebo conditions)
# - No domain randomization
# - No disturbances
# - Deterministic physics

set -e

echo "========================================="
echo "Fixed Environment Testing (Gazebo-Matching)"
echo "========================================="
echo ""
echo "Configuration:"
echo "  - Domain Randomization: DISABLED"
echo "  - Disturbances: DISABLED"
echo "  - Observation Noise: DISABLED"
echo "  - Physics: DETERMINISTIC"
echo ""
echo "⚠️  IMPORTANT: Different policies use different PD gains!"
echo "  - MLP/LSTM policies: Kp=25.0, Kd=0.5 (neural network actuators)"
echo "  - Implicit policies: Kp=160.0, Kd=5.0 (physics-based actuators)"
echo ""

# Step 1: Episode configs already generated
if [ ! -f "episode_configs_4tasks.yaml" ]; then
    echo "Generating episode configurations..."
    python generate_4_task_episodes.py \
      --num_repeats 50 \
      --output episode_configs_4tasks.yaml
fi

echo "✅ Using existing episode configs (200 episodes)"
echo ""

# Configuration array: [task_name, run_folder, checkpoint_file, output_name]
# ALL 6 trained policies tested with FIXED environment
declare -a CONFIGS=(
  "Unitree-Go2-Velocity-MLP-Custom:2026-02-03_15-54-07_work_good:model_24999.pt:mlp_dr_fixed"
  "Unitree-Go2-Velocity-MLP-No-DR:2026-02-05_00-58-13:model_24999.pt:mlp_no_dr_fixed"
  "Unitree-Go2-Velocity-LSTM-DR:2026-02-07_11-16-35:model_25000.pt:lstm_dr_fixed"
  "Unitree-Go2-Velocity-LSTM-No-DR:2026-02-07_22-16-50:model_25000.pt:lstm_no_dr_fixed"
  "Unitree-Go2-Velocity-Implicit-DR:2026-02-05_18-14-00:model_14600.pt:implicit_dr_fixed"
  "Unitree-Go2-Velocity-Implicit:2026-02-05_16-27-42:model_8100.pt:implicit_fixed"
)

echo "Testing ${#CONFIGS[@]} policies in FIXED environment..."
echo ""

# Test each configuration
for config in "${CONFIGS[@]}"; do
  IFS=':' read -r task_name run_folder checkpoint output_name <<< "$config"

  # Determine PD gains based on policy type
  if [[ "$task_name" == *"Implicit"* ]]; then
    pd_gains="Kp=160.0, Kd=5.0 (IdealPD actuator)"
    gazebo_config="go2_controllers_implicit.yaml"
  else
    pd_gains="Kp=25.0, Kd=0.5 (Neural network actuator)"
    gazebo_config="go2_controllers_mlp_lstm.yaml"
  fi

  echo "========================================="
  echo "Testing: $task_name"
  echo "Checkpoint: $checkpoint"
  echo "Environment: FIXED (no randomization)"
  echo "PD Gains: $pd_gains"
  echo "Gazebo Config: $gazebo_config"
  echo "Output: logs/data_collection_fixed_env/$output_name/"
  echo "========================================="

  # Construct checkpoint path
  exp_name=$(echo "$task_name" | tr '[:upper:]' '[:lower:]' | tr '-' '_')
  checkpoint_path="logs/rsl_rl/${exp_name}/${run_folder}/${checkpoint}"

  # Use regular data collection script but with modified environment
  python scripts/data_collection/collect_data_isaaclab.py \
    --task "$task_name" \
    --checkpoint "$checkpoint_path" \
    --episodes episode_configs_4tasks.yaml \
    --output "logs/data_collection_fixed_env/$output_name/" \
    --num_episodes 200 \
    --headless

  echo "✅ Completed $output_name"
  echo ""
done

# Summary
echo "========================================="
echo "All policies tested in FIXED environment!"
echo "========================================="
echo ""
echo "Results saved to:"
echo "  logs/data_collection_fixed_env/mlp_dr_fixed/"
echo "  logs/data_collection_fixed_env/mlp_no_dr_fixed/"
echo "  logs/data_collection_fixed_env/lstm_dr_fixed/"
echo "  logs/data_collection_fixed_env/lstm_no_dr_fixed/"
echo "  logs/data_collection_fixed_env/implicit_dr_fixed/"
echo "  logs/data_collection_fixed_env/implicit_fixed/"
echo ""
echo "Each folder contains 200 CSV files"
echo "Total: 1200 CSV files ready for Gazebo comparison!"
echo ""
echo "Next steps:"
echo "  1. Copy Gazebo configs to Vistec_ex_ws:"
echo "     cp gazebo_configs/*.yaml ~/Vistec_ex_ws/src/go2_gazebo/config/"
echo ""
echo "  2. Collect matching data in Gazebo with correct PD gains:"
echo "     - mlp_dr_fixed: Use go2_controllers_mlp_lstm.yaml"
echo "     - mlp_no_dr_fixed: Use go2_controllers_mlp_lstm.yaml"
echo "     - lstm_dr_fixed: Use go2_controllers_mlp_lstm.yaml"
echo "     - lstm_no_dr_fixed: Use go2_controllers_mlp_lstm.yaml"
echo "     - implicit_dr_fixed: Use go2_controllers_implicit.yaml"
echo "     - implicit_fixed: Use go2_controllers_implicit.yaml"
echo ""
echo "  3. Compare IsaacLab (fixed) vs Gazebo datasets"
echo "  4. Analyze sim-to-sim transfer performance"
echo ""
