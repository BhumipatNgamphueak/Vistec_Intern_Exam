#!/bin/bash
###############################################################################
# Collect Data from All 6 Configurations
#
# This script collects data from all trained policies at 25000 iterations:
# 1. LSTM + DR
# 2. LSTM - No DR
# 3. MLP + DR
# 4. MLP - No DR
# 5. Implicit + DR
# 6. Implicit - No DR
#
# Usage:
#   ./collect_all_data.sh                    # Collect all 6
#   ./collect_all_data.sh lstm_dr            # Collect specific config
#   ./collect_all_data.sh lstm_dr mlp_dr     # Collect multiple configs
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration mapping
declare -A CONFIGS=(
    ["lstm_dr"]="Unitree-Go2-Velocity-LSTM-DR|2026-02-07_11-16-35|model_25000.pt"
    ["lstm_no_dr"]="Unitree-Go2-Velocity-LSTM-No-DR|2026-02-07_22-16-50|model_25000.pt"
    ["mlp_dr"]="Unitree-Go2-Velocity-MLP-Custom|2026-02-03_15-54-07_work_good|model_24999.pt"
    ["mlp_no_dr"]="Unitree-Go2-Velocity-MLP-No-DR|2026-02-05_00-58-13|model_24999.pt"
    ["implicit_dr"]="Unitree-Go2-Velocity-Implicit-DR|2026-02-05_18-14-00|model_25000.pt"
    ["implicit_no_dr"]="Unitree-Go2-Velocity-Implicit|2026-02-05_16-27-42|model_25000.pt"
)

# Determine which configs to collect
if [ $# -eq 0 ]; then
    # No arguments - collect all
    CONFIGS_TO_COLLECT=("lstm_dr" "lstm_no_dr" "mlp_dr" "mlp_no_dr" "implicit_dr" "implicit_no_dr")
else
    # Collect specified configs
    CONFIGS_TO_COLLECT=("$@")
fi

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║      Data Collection - All 6 Configurations           ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Generate episode configs if not exists
if [ ! -f "episode_configs.yaml" ]; then
    echo -e "${YELLOW}Generating episode configurations...${NC}"
    python episode_config_generator.py \
        --num_episodes 200 \
        --output episode_configs.yaml
    echo -e "${GREEN}✓ Generated episode_configs.yaml${NC}"
    echo ""
fi

# Create output directories
mkdir -p policies
mkdir -p logs/data_collection

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
        echo -e "${YELLOW}  Run training script first:${NC}"
        echo -e "  ${BLUE}./continue_${config_name}_training.sh${NC}"
        FAILED+=("$config_name")
        continue
    fi

    echo -e "Task: ${GREEN}${TASK}${NC}"
    echo -e "Checkpoint: ${GREEN}${CHECKPOINT}${NC}"
    echo -e "Output: ${GREEN}logs/data_collection/${config_name}/${NC}"
    echo ""

    # Copy checkpoint to policies directory
    POLICY_FILE="policies/policy_${config_name}.pt"
    if [ ! -f "$POLICY_FILE" ]; then
        echo "Copying checkpoint..."
        cp "$CHECKPOINT_PATH" "$POLICY_FILE"
        echo -e "${GREEN}✓ Copied to $POLICY_FILE${NC}"
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
    OUTPUT_DIR="logs/data_collection/${config_name}"
    mkdir -p "$OUTPUT_DIR"

    # Collect data
    echo ""
    echo -e "${GREEN}Starting data collection (200 episodes)...${NC}"
    echo ""

    python scripts/data_collection/collect_data_isaaclab.py \
        --task "$TASK" \
        --checkpoint "$POLICY_FILE" \
        --episodes episode_configs.yaml \
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
    echo "Failed: ${#FAILED[@]}/$TOTAL"
    for config in "${FAILED[@]}"; do
        echo -e "  ${RED}✗ $config${NC}"
    done
fi

echo ""
echo "Output directory: logs/data_collection/"
echo ""

# Show disk usage
echo "Disk usage by configuration:"
du -sh logs/data_collection/*/ 2>/dev/null | sort -h

echo ""
echo -e "${GREEN}All done!${NC}"
