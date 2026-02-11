#!/bin/bash
# Quick verification script for GitHub users
# Run this after completing SETUP.md

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║          Vistec Intern Exam - Setup Verification          ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check counter
passed=0
failed=0

echo "1. Checking Environment Variables..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

vars=("VISTEC_REPO" "UNITREE_LAB" "ACTUATOR_NET" "VISTEC_WS")
for var in "${vars[@]}"; do
    if [ -n "${!var}" ]; then
        echo -e "  ${GREEN}✓${NC} $var=${!var}"
        ((passed++))
    else
        echo -e "  ${RED}✗${NC} $var is not set"
        ((failed++))
    fi
done
echo ""

echo "2. Checking Directories..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

dirs=("$VISTEC_REPO" "$UNITREE_LAB" "$VISTEC_WS")
dir_names=("VISTEC_REPO" "UNITREE_LAB" "VISTEC_WS")

for i in "${!dirs[@]}"; do
    if [ -d "${dirs[$i]}" ]; then
        echo -e "  ${GREEN}✓${NC} ${dir_names[$i]} exists: ${dirs[$i]}"
        ((passed++))
    else
        echo -e "  ${RED}✗${NC} ${dir_names[$i]} not found: ${dirs[$i]}"
        echo "      Run: mkdir -p ${dirs[$i]}"
        ((failed++))
    fi
done
echo ""

echo "3. Checking Essential Files..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check trained models
if [ -d "$VISTEC_REPO/trained_models" ]; then
    model_count=$(find "$VISTEC_REPO/trained_models" -name "*.pt" | wc -l)
    if [ $model_count -ge 4 ]; then
        echo -e "  ${GREEN}✓${NC} Trained models found ($model_count .pt files)"
        ((passed++))
    else
        echo -e "  ${YELLOW}⚠${NC} Expected 4+ models, found $model_count"
        ((failed++))
    fi
else
    echo -e "  ${RED}✗${NC} trained_models/ directory not found"
    ((failed++))
fi

# Check velocity command scripts
if [ -f "$VISTEC_REPO/send_velocity_commands_gazebo.sh" ] && [ -f "$VISTEC_REPO/send_velocity_commands_isaac.py" ]; then
    echo -e "  ${GREEN}✓${NC} Velocity command scripts exist"
    ((passed++))
else
    echo -e "  ${RED}✗${NC} Velocity command scripts missing"
    ((failed++))
fi

# Check configs
if [ -d "$VISTEC_REPO/unitree_rl_lab/Configs" ]; then
    config_count=$(find "$VISTEC_REPO/unitree_rl_lab/Configs" -name "*.py" | wc -l)
    if [ $config_count -ge 10 ]; then
        echo -e "  ${GREEN}✓${NC} Config files found ($config_count .py files)"
        ((passed++))
    else
        echo -e "  ${YELLOW}⚠${NC} Expected 10+ configs, found $config_count"
        ((failed++))
    fi
else
    echo -e "  ${RED}✗${NC} unitree_rl_lab/Configs/ not found"
    ((failed++))
fi
echo ""

echo "4. Checking Software Prerequisites..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check Python
if command -v python3 &> /dev/null; then
    python_ver=$(python3 --version 2>&1 | awk '{print $2}')
    echo -e "  ${GREEN}✓${NC} Python 3 installed: $python_ver"
    ((passed++))
else
    echo -e "  ${RED}✗${NC} Python 3 not found"
    ((failed++))
fi

# Check CUDA
if command -v nvidia-smi &> /dev/null; then
    cuda_ver=$(nvidia-smi | grep "CUDA Version" | awk '{print $9}')
    echo -e "  ${GREEN}✓${NC} CUDA detected: $cuda_ver"
    ((passed++))
else
    echo -e "  ${YELLOW}⚠${NC} nvidia-smi not found (GPU required for training)"
    ((failed++))
fi

# Check ROS 2
if [ -n "$ROS_DISTRO" ]; then
    echo -e "  ${GREEN}✓${NC} ROS 2 sourced: $ROS_DISTRO"
    ((passed++))
else
    echo -e "  ${YELLOW}⚠${NC} ROS 2 not sourced (required for Gazebo)"
    echo "      Run: source /opt/ros/humble/setup.bash"
    ((failed++))
fi

# Check Isaac Lab
if [ -f ~/IsaacLab/isaaclab.sh ]; then
    echo -e "  ${GREEN}✓${NC} Isaac Lab found: ~/IsaacLab/"
    ((passed++))
else
    echo -e "  ${YELLOW}⚠${NC} Isaac Lab not found at ~/IsaacLab/"
    echo "      Clone: git clone https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab"
    ((failed++))
fi
echo ""

echo "5. Checking ROS 2 Workspace..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -d "$VISTEC_WS/src" ]; then
    pkg_count=$(find "$VISTEC_WS/src" -name "package.xml" | wc -l)
    if [ $pkg_count -ge 2 ]; then
        echo -e "  ${GREEN}✓${NC} ROS 2 packages found ($pkg_count packages)"
        ((passed++))
    else
        echo -e "  ${YELLOW}⚠${NC} Expected 2+ packages, found $pkg_count"
        echo "      Run: cp -r \$VISTEC_REPO/Vistec_ex_ws/src/* \$VISTEC_WS/src/"
        ((failed++))
    fi

    if [ -d "$VISTEC_WS/install" ]; then
        echo -e "  ${GREEN}✓${NC} Workspace built (install/ exists)"
        ((passed++))
    else
        echo -e "  ${YELLOW}⚠${NC} Workspace not built"
        echo "      Run: cd \$VISTEC_WS && colcon build"
        ((failed++))
    fi
else
    echo -e "  ${RED}✗${NC} ROS 2 workspace not initialized"
    echo "      Run: mkdir -p \$VISTEC_WS/src"
    ((failed++))
fi
echo ""

echo "╔═══════════════════════════════════════════════════════════╗"
echo "║                     VERIFICATION SUMMARY                   ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo -e "  ${GREEN}Passed${NC}: $passed"
echo -e "  ${RED}Failed${NC}: $failed"
echo ""

if [ $failed -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed! Your setup is complete.${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Test Isaac Lab: See DEPLOYMENT_GUIDE.md - Option 1"
    echo "  2. Test Gazebo: See DEPLOYMENT_GUIDE.md - Option 2"
    echo "  3. Try 4 tasks: ./send_velocity_commands_gazebo.sh"
    exit 0
else
    echo -e "${YELLOW}⚠ Some checks failed. Review errors above and see SETUP.md${NC}"
    echo ""
    echo "Common fixes:"
    echo "  • Set env vars: See SETUP.md - Step 2"
    echo "  • Install Isaac Lab: See SETUP.md - Step 3.1"
    echo "  • Install ROS 2: See SETUP.md - Step 3.2"
    echo "  • Build workspace: cd \$VISTEC_WS && colcon build"
    exit 1
fi
