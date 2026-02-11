#!/bin/bash
###############################################################################
# Collect Data from All 6 Configurations - Using Current Checkpoints
#
# Uses the latest available checkpoints (no additional training needed):
# 1. LSTM-DR: model_25000.pt ✓
# 2. LSTM-No-DR: model_25000.pt ✓
# 3. MLP-DR: model_24999.pt ✓
# 4. MLP-No-DR: model_24999.pt ✓
# 5. Implicit-DR: model_14600.pt ✓ (current - ready!)
# 6. Implicit-No-DR: model_8100.pt ✓ (current - ready!)
#
# Usage:
#   ./collect_all_data_now.sh                 # Collect all 6
#   ./collect_all_data_now.sh lstm_dr mlp_dr  # Collect specific configs
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration mapping - Using CURRENT checkpoints
declare -A CONFIGS=(
    ["lstm_dr"]="Unitree-Go2-Velocity-LSTM-DR|2026-02-07_11-16-35|model_25000.pt"
    ["lstm_no_dr"]="Unitree-Go2-Velocity-LSTM-No-DR|2026-02-07_22-16-50|model_25000.pt"
    ["mlp_dr"]="Unitree-Go2-Velocity-MLP-Custom|2026-02-03_15-54-07_work_good|model_24999.pt"
    ["mlp_no_dr"]="Unitree-Go2-Velocity-MLP-No-DR|2026-02-05_00-58-13|model_24999.pt"
    ["implicit_dr"]="Unitree-Go2-Velocity-Implicit-DR|2026-02-05_18-14-00|model_14600.pt"
    ["implicit_no_dr"]="Unitree-Go2-Velocity-Implicit|2026-02-05_16-27-42|model_8100.pt"
)

# Determine which configs to collect
if [ $# -eq 0 ]; then
    CONFIGS_TO_COLLECT=("lstm_dr" "lstm_no_dr" "mlp_dr" "mlp_no_dr" "implicit_dr" "implicit_no_dr")
else
    CONFIGS_TO_COLLECT=("$@")
fi

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║      Data Collection - All 6 Configurations           ║${NC}"
echo -e "${BLUE}║      (Using Current Checkpoints - No Training!)        ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if task-based episodes exist
EPISODE_FILE="episode_configs_tasks.yaml"
if [ ! -f "$EPISODE_FILE" ]; then
    echo -e "${YELLOW}Task-based episodes not found. Generating...${NC}"
    python episode_config_generator_tasks.py \
        --episodes_per_task 50 \
        --output "$EPISODE_FILE"
    echo -e "${GREEN}✓ Generated $EPISODE_FILE${NC}"
else
    echo -e "${GREEN}✓ Using existing $EPISODE_FILE${NC}"
fi
echo ""

# Create output directories
mkdir -p policies
mkdir -p logs/data_collection_tasks

# Collect data for each configuration
TOTAL=${#CONFIGS_TO_COLLECT[@]}
CURRENT=0
COMPLETED=()
FAILED=()

for config_name in "${CONFIGS_TO_COLLECT[@]}"; do
    CURRENT=$((CURRENT + 1))

    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}Configuration $CURRENT/$TOTAL: ${config_name}${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

    # Get config details
    if [ -z "${CONFIGS[$config_name]}" ]; then
        echo -e "${RED}✗ Unknown configuration: $config_name${NC}"
        FAILED+=("$config_name")
        continue
    fi

    IFS='|' read -r TASK RUN_DIR CHECKPOINT <<< "${CONFIGS[$config_name]}"

    # Get task directory name
    TASK_DIR=$(echo "$TASK" | tr '[:upper:]' '[:lower:]' | tr '-' '_')
    CHECKPOINT_PATH="logs/rsl_rl/${TASK_DIR}/${RUN_DIR}/${CHECKPOINT}"

    # Check if checkpoint exists
    if [ ! -f "$CHECKPOINT_PATH" ]; then
        echo -e "${RED}✗ Checkpoint not found: $CHECKPOINT_PATH${NC}"
        FAILED+=("$config_name")
        continue
    fi

    echo -e "Task: ${GREEN}${TASK}${NC}"
    echo -e "Checkpoint: ${GREEN}${CHECKPOINT}${NC}"
    echo -e "Output: ${GREEN}logs/data_collection_tasks/${config_name}/${NC}"
    echo ""

    # Copy checkpoint to policies directory
    POLICY_FILE="policies/policy_${config_name}.pt"
    if [ ! -f "$POLICY_FILE" ]; then
        echo "Copying checkpoint..."
        cp "$CHECKPOINT_PATH" "$POLICY_FILE"
        echo -e "${GREEN}✓ Copied to $POLICY_FILE${NC}"
    else
        echo -e "${GREEN}✓ Policy already exists: $POLICY_FILE${NC}"
    fi

    # Verify policy loads
    echo "Verifying policy..."
    python -c "
import torch
try:
    policy = torch.jit.load('$POLICY_FILE')
    print('✓ Policy loaded successfully')
except Exception as e:
    print(f'✗ Error: {e}')
    exit(1)
" || {
        echo -e "${RED}✗ Failed to load policy${NC}"
        FAILED+=("$config_name")
        continue
    }

    # Create output directory
    OUTPUT_DIR="logs/data_collection_tasks/${config_name}"
    mkdir -p "$OUTPUT_DIR"

    # Check if already collected
    NUM_EXISTING=$(ls "$OUTPUT_DIR"/*.csv 2>/dev/null | wc -l)
    if [ $NUM_EXISTING -eq 200 ]; then
        echo -e "${GREEN}✓ Already collected: $NUM_EXISTING/200 episodes${NC}"
        COMPLETED+=("$config_name")
        continue
    elif [ $NUM_EXISTING -gt 0 ]; then
        echo -e "${YELLOW}⚠ Partially collected: $NUM_EXISTING/200 episodes${NC}"
        read -p "Continue collection? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Skipping $config_name"
            continue
        fi
    fi

    # Collect data
    echo ""
    echo -e "${GREEN}Starting data collection (200 episodes = 4 tasks × 50 each)...${NC}"
    echo ""

    python scripts/data_collection/collect_data_isaaclab.py \
        --task "$TASK" \
        --checkpoint "$POLICY_FILE" \
        --episodes "$EPISODE_FILE" \
        --output "$OUTPUT_DIR/" \
        --num_episodes 200 \
        --headless

    # Validate
    NUM_FILES=$(ls "$OUTPUT_DIR"/*.csv 2>/dev/null | wc -l)
    if [ $NUM_FILES -eq 200 ]; then
        echo -e "${GREEN}✓ Successfully collected $NUM_FILES episodes${NC}"
        COMPLETED+=("$config_name")
    else
        echo -e "${YELLOW}⚠ Only collected $NUM_FILES/200 episodes${NC}"
        FAILED+=("$config_name")
    fi

    echo ""
done

# Summary
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              Data Collection Complete                  ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "Completed: ${#COMPLETED[@]}/$TOTAL"
for config in "${COMPLETED[@]}"; do
    echo -e "  ${GREEN}✓ $config${NC}"
done

if [ ${#FAILED[@]} -gt 0 ]; then
    echo ""
    echo "Failed/Skipped: ${#FAILED[@]}/$TOTAL"
    for config in "${FAILED[@]}"; do
        echo -e "  ${RED}✗ $config${NC}"
    done
fi

echo ""
echo "Output directory: logs/data_collection_tasks/"
echo ""

# Show disk usage
if [ -d "logs/data_collection_tasks" ]; then
    echo "Disk usage by configuration:"
    du -sh logs/data_collection_tasks/*/ 2>/dev/null | sort -h
fi

echo ""
echo -e "${GREEN}All done!${NC}"
echo ""
echo "Next steps:"
echo "  1. Validate data: Check episode counts and data quality"
echo "  2. Analyze by task: Compare configs on standing/walking/turning"
echo "  3. Compare actuators: LSTM vs MLP vs Implicit"
echo "  4. Evaluate DR impact: With DR vs Without DR"
