# Velocity Commands Quick Reference

## ✅ Option B: Simplified + Detailed Commands

You now have **TWO ways** to test the 4 locomotion tasks:

---

## 🎯 SIMPLIFIED (4 Commands - Quick Testing)

Perfect for quick testing of each main task category.

### Isaac Lab
```bash
python send_velocity_commands_isaac.py --task 1   # Task 1: Standing
python send_velocity_commands_isaac.py --task 2   # Task 2: Walking (1.0 m/s)
python send_velocity_commands_isaac.py --task 3   # Task 3: Turning (1.0 rad/s)
python send_velocity_commands_isaac.py --task 4   # Task 4: Walk+Turn (arc)
```

### Gazebo
```bash
./send_velocity_commands_gazebo.sh
# Quick select: 1, 3, 7, or 10
```

| Option | Task | Command |
|--------|------|---------|
| 1 | Standing | (0.0, 0.0, 0.0) |
| 3 | Walk normal | (1.0, 0.0, 0.0) |
| 7 | Turn normal | (0.0, 0.0, 1.0) |
| 10 | Walk+Turn | (0.8, 0.0, 0.6) |

---

## 📋 DETAILED (17 Variants - Comprehensive Testing)

Test all speed/rate variations from training episodes.

### Isaac Lab - All Variants

```bash
# TASK 1: Standing (1 variant)
python send_velocity_commands_isaac.py --task 1

# TASK 2: Walking (5 variants)
python send_velocity_commands_isaac.py --task 2      # Alias: normal (1.0 m/s)
python send_velocity_commands_isaac.py --task 2.1    # Slow (0.5 m/s)
python send_velocity_commands_isaac.py --task 2.2    # Normal (1.0 m/s)
python send_velocity_commands_isaac.py --task 2.3    # Fast (1.5 m/s)
python send_velocity_commands_isaac.py --task 2.4    # Moderate (0.8 m/s)

# TASK 3: Turn in Place (5 variants)
python send_velocity_commands_isaac.py --task 3      # Alias: normal CCW (1.0 rad/s)
python send_velocity_commands_isaac.py --task 3.1    # Slow CCW (0.5 rad/s)
python send_velocity_commands_isaac.py --task 3.2    # Normal CCW (1.0 rad/s)
python send_velocity_commands_isaac.py --task 3.3    # Normal CW (-1.0 rad/s)
python send_velocity_commands_isaac.py --task 3.4    # Fast CCW (1.5 rad/s)

# TASK 4: Walk + Turn (5 variants)
python send_velocity_commands_isaac.py --task 4      # Alias: right arc
python send_velocity_commands_isaac.py --task 4.1    # Right arc (0.8, 0.6)
python send_velocity_commands_isaac.py --task 4.2    # Straight fast (1.2, 0.0)
python send_velocity_commands_isaac.py --task 4.3    # Left arc (0.8, -0.6)
python send_velocity_commands_isaac.py --task 4.4    # Tight turn (0.5, 1.0)
```

### Gazebo - All Variants

```bash
./send_velocity_commands_gazebo.sh
```

Then select from menu:

| Option | Task | Description | (vx, vy, wz) |
|--------|------|-------------|--------------|
| **1** | **1** | **Standing** ⭐ | (0.0, 0.0, 0.0) |
| 2 | 2.1 | Walk slow | (0.5, 0.0, 0.0) |
| **3** | **2.2** | **Walk normal** ⭐ | (1.0, 0.0, 0.0) |
| 4 | 2.3 | Walk fast | (1.5, 0.0, 0.0) |
| 5 | 2.4 | Walk moderate | (0.8, 0.0, 0.0) |
| 6 | 3.1 | Turn slow CCW | (0.0, 0.0, 0.5) |
| **7** | **3.2** | **Turn normal CCW** ⭐ | (0.0, 0.0, 1.0) |
| 8 | 3.3 | Turn normal CW | (0.0, 0.0, -1.0) |
| 9 | 3.4 | Turn fast CCW | (0.0, 0.0, 1.5) |
| **10** | **4.1** | **Right arc** ⭐ | (0.8, 0.0, 0.6) |
| 11 | 4.2 | Straight fast | (1.2, 0.0, 0.0) |
| 12 | 4.3 | Left arc | (0.8, 0.0, -0.6) |
| 13 | 4.4 | Tight turn | (0.5, 0.0, 1.0) |
| 14 | - | Custom | (enter values) |

**⭐ = Simplified aliases (options 1, 3, 7, 10)**

---

## 🔍 Which Should You Use?

### Use SIMPLIFIED (1, 2, 3, 4) when:
- ✅ Quick sanity check
- ✅ Demonstrating the 4 main task categories
- ✅ Teaching/showing someone
- ✅ Initial policy validation

### Use DETAILED (2.1, 2.2, 3.3, etc.) when:
- ✅ Comprehensive policy testing
- ✅ Matching exact training episode conditions
- ✅ Testing performance across speed variations
- ✅ Research/data collection

---

## 📊 Task Mapping

| Simplified ID | Detailed IDs | Category | Representative Command |
|---------------|--------------|----------|------------------------|
| `1` | `1` | Standing | Stand still |
| `2` | `2`, `2.1-2.4` | Walking | Walk normal (1.0 m/s) |
| `3` | `3`, `3.1-3.4` | Turning | Turn normal (1.0 rad/s) |
| `4` | `4`, `4.1-4.4` | Walk+Turn | Right arc |

---

## 🧪 Example Testing Workflow

### Quick Test (4 commands)
```bash
# Test all 4 main tasks in Isaac Lab
python send_velocity_commands_isaac.py --task 1  # Standing
python send_velocity_commands_isaac.py --task 2  # Walking
python send_velocity_commands_isaac.py --task 3  # Turning
python send_velocity_commands_isaac.py --task 4  # Walk+Turn
```

### Comprehensive Test (all variants)
```bash
# Test all walking speeds
for task in 2.1 2.2 2.3 2.4; do
    python send_velocity_commands_isaac.py --task $task
done

# Test all turning rates
for task in 3.1 3.2 3.3 3.4; do
    python send_velocity_commands_isaac.py --task $task
done

# Test all combined maneuvers
for task in 4.1 4.2 4.3 4.4; do
    python send_velocity_commands_isaac.py --task $task
done
```

---

## 💡 Pro Tips

### 1. List All Available Tasks
```bash
python send_velocity_commands_isaac.py --list
```

Output shows both simplified and detailed options.

### 2. Gazebo Quick Select
In the menu, just remember:
- **1** = Standing
- **3** = Walking
- **7** = Turning
- **10** = Walk+Turn

### 3. Custom Commands (Advanced)
```bash
# Isaac Lab - custom velocity
python send_velocity_commands_isaac.py --linear_x 0.7 --angular_z 0.4

# Gazebo - option 14 for custom input
./send_velocity_commands_gazebo.sh
# Select 14, then enter custom values
```

---

## 📝 Summary

**Before (Option A)**: 13 detailed commands only
- ❌ Complex for quick testing
- ✅ Comprehensive coverage

**After (Option B)**: 4 simplified + 13 detailed
- ✅ Quick testing with 4 commands
- ✅ Detailed testing with all variants
- ✅ Best of both worlds!

---

## 🔗 Related Files

- [send_velocity_commands_isaac.py](send_velocity_commands_isaac.py) - Isaac Lab velocity sender
- [send_velocity_commands_gazebo.sh](send_velocity_commands_gazebo.sh) - Gazebo velocity sender
- [generate_4_task_episodes.py](unitree_rl_lab/Utils/generate_4_task_episodes.py) - Training episode generator

---

**Last Updated**: February 11, 2026
