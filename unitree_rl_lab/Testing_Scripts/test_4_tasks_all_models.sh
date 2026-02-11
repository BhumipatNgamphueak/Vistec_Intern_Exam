#!/bin/bash
# Test all 6 models on the 4 specific locomotion tasks
# Tasks: Standing, Walking (varying speeds), Turn in Place, Walk+Turn

set -e

echo "========================================="
echo "Testing 4 Specific Tasks on All 6 Models"
echo "========================================="

# Step 1: Generate episode configs for 4 tasks
echo ""
echo "Step 1: Generating episode configurations for 4 tasks..."
python generate_4_task_episodes.py \
  --num_repeats 50 \
  --output episode_configs_4tasks.yaml

echo "✅ Generated 200 episodes (4 tasks × 50 repeats)"

# Configuration array: [task_name, run_folder, checkpoint_file]
declare -a CONFIGS=(
  "Unitree-Go2-Velocity-MLP-Custom:2026-02-03_15-54-07_work_good:model_24999.pt:mlp_custom"
  "Unitree-Go2-Velocity-MLP-No-DR:2026-02-05_00-58-13:model_24999.pt:mlp_no_dr"
  "Unitree-Go2-Velocity-LSTM-DR:2026-02-07_11-16-35:model_25000.pt:lstm_dr"
  "Unitree-Go2-Velocity-LSTM-No-DR:2026-02-07_22-16-50:model_25000.pt:lstm_no_dr"
  "Unitree-Go2-Velocity-Implicit-DR:2026-02-05_18-14-00:model_14600.pt:implicit_dr"
  "Unitree-Go2-Velocity-Implicit:2026-02-05_16-27-42:model_8100.pt:implicit"
)

# Step 2: Test each configuration
echo ""
echo "Step 2: Testing all 6 configurations..."
echo ""

for config in "${CONFIGS[@]}"; do
  IFS=':' read -r task_name run_folder checkpoint output_name <<< "$config"

  echo "========================================="
  echo "Testing: $task_name"
  echo "Checkpoint: $checkpoint"
  echo "Output: logs/data_collection_4tasks/$output_name/"
  echo "========================================="

  # Construct checkpoint path
  exp_name=$(echo "$task_name" | tr '[:upper:]' '[:lower:]' | tr '-' '_')
  checkpoint_path="logs/rsl_rl/${exp_name}/${run_folder}/${checkpoint}"

  python scripts/data_collection/collect_data_isaaclab.py \
    --task "$task_name" \
    --checkpoint "$checkpoint_path" \
    --episodes episode_configs_4tasks.yaml \
    --output "logs/data_collection_4tasks/$output_name/" \
    --num_episodes 200 \
    --headless

  echo "✅ Completed $output_name"
  echo ""
done

# Step 3: Summary
echo "========================================="
echo "All 6 models tested on 4 tasks!"
echo "========================================="
echo ""
echo "Results saved to:"
echo "  logs/data_collection_4tasks/mlp_custom/"
echo "  logs/data_collection_4tasks/mlp_no_dr/"
echo "  logs/data_collection_4tasks/lstm_dr/"
echo "  logs/data_collection_4tasks/lstm_no_dr/"
echo "  logs/data_collection_4tasks/implicit_dr/"
echo "  logs/data_collection_4tasks/implicit/"
echo ""
echo "Each folder contains 200 CSV files (50 repeats × 4 tasks)"
echo ""
echo "To analyze results:"
echo "  ls logs/data_collection_4tasks/*/locomotion_log_*.csv | wc -l  # Should be 1200 total"
echo "  python analyze_4tasks_results.py  # (create your analysis script)"
echo ""
