# PD Gains: Critical Actuator-Specific Configuration

## ⚠️ CRITICAL DISCOVERY

**Your 6 trained policies use TWO DIFFERENT sets of PD controller gains!**

Using the wrong gains in Gazebo will cause **severe performance degradation** and make sim-to-sim comparison meaningless.

---

## The Two Actuator Types

### 1. Neural Network Actuators (MLP/LSTM)

**Used by 4 policies:**
- ✅ MLP + Custom DR
- ✅ MLP - No DR
- ✅ LSTM + DR
- ✅ LSTM - No DR

**PD Gains:**
```yaml
Kp: 25.0   # Very LOW stiffness
Kd: 0.5    # Very LOW damping
```

**Why low gains?**
- Neural networks learn to model **actuator dynamics internally**
- The network outputs desired joint positions, but also learns:
  - Motor response characteristics
  - Gear backlash
  - Torque-velocity curves
  - Dynamic friction
- With low PD gains, the network has more control authority
- High gains would fight against the learned actuator model

**Source:** `unitree_actuators.py`
```python
class UnitreeActuatorCfg_Go2_MLP(ActuatorNetMLPCfg):
    stiffness = {".*": 25.0}
    damping = {".*": 0.5}
```

---

### 2. Physics-Based Actuators (Implicit/IdealPD)

**Used by 2 policies:**
- ✅ Implicit + DR
- ✅ Implicit - No DR

**PD Gains:**
```yaml
Kp: 160.0  # HIGH stiffness
Kd: 5.0    # HIGH damping
```

**Why high gains?**
- These policies use **IdealPD** actuators (physics simulation)
- No learned actuator dynamics - just pure PD control
- Higher gains needed for tight position tracking
- The policy outputs joint positions, PD controller does the rest

**Source:** `unitree.py`
```python
actuators = {
    "legs": IdealPDActuatorCfg(
        stiffness=160.0,
        damping=5.0,
    )
}
```

---

## Why This Matters for Gazebo

### Correct Configuration (MUST DO)

When testing each policy in Gazebo, **SWITCH the PD gains** based on policy type:

#### For MLP/LSTM Policies:
```yaml
# ros2_controllers_mlp_lstm.yaml
go2_joint_controller:
  gains:
    FL_hip_joint:   {p: 25.0, d: 0.5, i: 0.0}
    FL_thigh_joint: {p: 25.0, d: 0.5, i: 0.0}
    FL_calf_joint:  {p: 25.0, d: 0.5, i: 0.0}
    # ... (all 12 joints)
```

#### For Implicit Policies:
```yaml
# ros2_controllers_implicit.yaml
go2_joint_controller:
  gains:
    FL_hip_joint:   {p: 160.0, d: 5.0, i: 0.0}
    FL_thigh_joint: {p: 160.0, d: 5.0, i: 0.0}
    FL_calf_joint:  {p: 160.0, d: 5.0, i: 0.0}
    # ... (all 12 joints)
```

### ❌ What Happens If You Use Wrong Gains?

#### Using HIGH gains (160/5) on MLP/LSTM policies:
- Robot becomes **too stiff and oscillates**
- Learned actuator model conflicts with PD controller
- Performance degrades significantly
- May become unstable

#### Using LOW gains (25/0.5) on Implicit policies:
- Robot becomes **too compliant and weak**
- Cannot track desired positions accurately
- Falls over or moves very sluggishly
- Performance degrades significantly

---

## Quick Reference Table

| Policy Name | Training Config | Actuator Type | Kp | Kd | Gazebo Config |
|-------------|----------------|---------------|-----|-----|---------------|
| MLP + Custom DR | `velocity_env_cfg.py` | ActuatorNetMLP | **25.0** | **0.5** | `mlp_lstm.yaml` |
| MLP - No DR | `velocity_env_cfg_mlp_no_dr.py` | ActuatorNetMLP | **25.0** | **0.5** | `mlp_lstm.yaml` |
| LSTM + DR | `velocity_env_cfg_lstm_with_dr.py` | ActuatorNetMLP | **25.0** | **0.5** | `mlp_lstm.yaml` |
| LSTM - No DR | `velocity_env_cfg_lstm_no_dr.py` | ActuatorNetMLP | **25.0** | **0.5** | `mlp_lstm.yaml` |
| Implicit + DR | `velocity_env_cfg_implicit_with_dr.py` | IdealPDActuator | **160.0** | **5.0** | `implicit.yaml` |
| Implicit - No DR | `velocity_env_cfg_implicit.py` | IdealPDActuator | **160.0** | **5.0** | `implicit.yaml` |

---

## Verification Commands

### Check IsaacLab Configuration

**MLP Actuator:**
```bash
grep -A 3 "class UnitreeActuatorCfg_Go2_MLP" \
  source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree_actuators.py
```

Expected output:
```python
stiffness = {".*": 25.0}
damping = {".*": 0.5}
```

**Implicit Actuator:**
```bash
grep -B 2 -A 5 "IdealPDActuatorCfg" \
  source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py
```

Expected output:
```python
actuators = {
    "legs": IdealPDActuatorCfg(
        stiffness=160.0,
        damping=5.0,
    )
}
```

---

## Implementation Checklist for Gazebo

- [ ] Create two separate ROS 2 controller YAML files:
  - [ ] `ros2_controllers_mlp_lstm.yaml` (Kp=25, Kd=0.5)
  - [ ] `ros2_controllers_implicit.yaml` (Kp=160, Kd=5)

- [ ] When testing MLP policies:
  - [ ] Load `mlp_lstm.yaml` configuration
  - [ ] Verify Kp=25.0, Kd=0.5 in controller_manager
  - [ ] Test: MLP+DR, MLP-DR, LSTM+DR, LSTM-DR

- [ ] When testing Implicit policies:
  - [ ] Load `implicit.yaml` configuration
  - [ ] Verify Kp=160.0, Kd=5.0 in controller_manager
  - [ ] Test: Implicit+DR, Implicit-DR

- [ ] Document which gains were used for each data collection run

---

## Testing Workflow in Gazebo

### Option 1: Switch YAML Files

```bash
# Test MLP policy
ros2 launch go2_gazebo go2_control.launch.py \
  controller_config:=ros2_controllers_mlp_lstm.yaml \
  policy:=mlp_dr_fixed

# Test Implicit policy
ros2 launch go2_gazebo go2_control.launch.py \
  controller_config:=ros2_controllers_implicit.yaml \
  policy:=implicit_dr_fixed
```

### Option 2: Runtime Parameter Update

```bash
# For MLP policy
ros2 param set /controller_manager/go2_joint_controller FL_hip_joint.p 25.0
ros2 param set /controller_manager/go2_joint_controller FL_hip_joint.d 0.5
# ... (repeat for all 12 joints)

# For Implicit policy
ros2 param set /controller_manager/go2_joint_controller FL_hip_joint.p 160.0
ros2 param set /controller_manager/go2_joint_controller FL_hip_joint.d 5.0
# ... (repeat for all 12 joints)
```

**⚠️ Runtime updates require careful verification - switching config files is safer!**

---

## Summary

**DO:**
- ✅ Use Kp=25, Kd=0.5 for MLP/LSTM policies
- ✅ Use Kp=160, Kd=5 for Implicit policies
- ✅ Document which gains were used for each test
- ✅ Verify gains before each data collection run

**DON'T:**
- ❌ Use the same gains for all policies
- ❌ Assume all IsaacLab policies use identical PD gains
- ❌ Skip verification of controller parameters

---

## Additional Notes

### Why Different Actuator Models?

1. **ActuatorNetMLP (MLP/LSTM)**:
   - Trains a neural network to model actuator dynamics
   - More realistic sim-to-real transfer
   - Accounts for motor delays, backlash, torque limits
   - Requires lower PD gains to avoid conflict

2. **IdealPDActuator (Implicit)**:
   - Pure physics-based PD control
   - Instant torque response (unrealistic)
   - Simpler, faster training
   - Requires higher PD gains for tracking

### Performance Implications

Policies trained with **ActuatorNetMLP** generally transfer better to real hardware because they learn actuator imperfections. However, they require more training time.

Policies trained with **IdealPD** are simpler but may struggle on real hardware due to the reality gap in actuator dynamics.

---

**Last Updated:** 2026-02-10
**Source Files:**
- `unitree_rl_lab/assets/robots/unitree_actuators.py`
- `unitree_rl_lab/assets/robots/unitree.py`
