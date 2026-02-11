# Go2 Robot Joint Specifications

## Joint Velocity Limits

The Unitree Go2 quadruped robot has **12 actuated joints** (3 per leg × 4 legs).

### Velocity Limits by Joint Type

| Joint Type | Joints per Leg | Total Joints | Velocity Limit | Velocity (degrees) | Effort Limit |
|------------|---------------|--------------|----------------|-------------------|--------------|
| **Hip** | 1 | 4 | **23.0 rad/s** | 1317.8°/s | 200.0 Nm |
| **Thigh** | 1 | 4 | **23.0 rad/s** | 1317.8°/s | 200.0 Nm |
| **Calf** | 1 | 4 | **14.0 rad/s** | 802.1°/s | 320.0 Nm |

---

## Joint Names (Standard Order)

### Full Joint List (12 joints)

| Index | Joint Name | Type | Velocity Limit (rad/s) | Effort Limit (Nm) |
|-------|-----------|------|----------------------|------------------|
| 0 | `FL_hip` | Hip | 23.0 | 200.0 |
| 1 | `FL_thigh` | Thigh | 23.0 | 200.0 |
| 2 | `FL_calf` | Calf | 14.0 | 320.0 |
| 3 | `FR_hip` | Hip | 23.0 | 200.0 |
| 4 | `FR_thigh` | Thigh | 23.0 | 200.0 |
| 5 | `FR_calf` | Calf | 14.0 | 320.0 |
| 6 | `RL_hip` | Hip | 23.0 | 200.0 |
| 7 | `RL_thigh` | Thigh | 23.0 | 200.0 |
| 8 | `RL_calf` | Calf | 14.0 | 320.0 |
| 9 | `RR_hip` | Hip | 23.0 | 200.0 |
| 10 | `RR_thigh` | Thigh | 23.0 | 200.0 |
| 11 | `RR_calf` | Calf | 14.0 | 320.0 |

**Legend**: FL = Front Left, FR = Front Right, RL = Rear Left, RR = Rear Right

---

## PD Controller Gains

Default gains used in IsaacLab simulation:

| Joint Type | Stiffness (Kp) | Damping (Kd) |
|------------|----------------|--------------|
| **Hip** | 160.0 | 5.0 |
| **Thigh** | 160.0 | 5.0 |
| **Calf** | 160.0 | 5.0 |

**Note**: All joints use the same PD gains in the default configuration.

---

## Actuator Characteristics

### Motor Specifications

| Parameter | Hip/Thigh | Calf |
|-----------|-----------|------|
| **Peak Torque** | 200 Nm | 320 Nm |
| **Continuous Torque** | ~80-100 Nm | ~120-150 Nm |
| **Max Velocity** | 23 rad/s | 14 rad/s |
| **Gear Ratio** | ~9.1:1 | ~10.5:1 |

---

## Joint Position Limits

| Joint Type | Min Angle (rad) | Max Angle (rad) | Range (degrees) |
|------------|----------------|-----------------|-----------------|
| **Hip** | -1.047 | 1.047 | -60° to +60° |
| **Thigh** | -0.663 | 2.966 | -38° to +170° |
| **Calf** | -2.721 | -0.837 | -156° to -48° |

**Note**: These are mechanical limits. Safe operating range may be smaller.

---

## Default Standing Configuration

Default joint positions for standing pose:

```python
default_joint_pos = [
    0.0,  0.9, -1.8,  # FL: hip, thigh, calf
    0.0,  0.9, -1.8,  # FR: hip, thigh, calf
    0.0,  0.9, -1.8,  # RL: hip, thigh, calf
    0.0,  0.9, -1.8   # RR: hip, thigh, calf
]
```

- **Hip**: 0.0 rad (straight)
- **Thigh**: 0.9 rad (51.6°)
- **Calf**: -1.8 rad (-103.1°)

---

## Gazebo Configuration

### URDF/SDF Joint Limits

For Gazebo simulation, configure joint limits in URDF:

```xml
<!-- Hip joint example -->
<joint name="FL_hip" type="revolute">
  <limit effort="200" velocity="23.0" lower="-1.047" upper="1.047"/>
  <dynamics damping="5.0" friction="0.0"/>
</joint>

<!-- Thigh joint example -->
<joint name="FL_thigh" type="revolute">
  <limit effort="200" velocity="23.0" lower="-0.663" upper="2.966"/>
  <dynamics damping="5.0" friction="0.0"/>
</joint>

<!-- Calf joint example -->
<joint name="FL_calf" type="revolute">
  <limit effort="320" velocity="14.0" lower="-2.721" upper="-0.837"/>
  <dynamics damping="5.0" friction="0.0"/>
</joint>
```

---

## ROS 2 Control Configuration

### Position Controller (example)

```yaml
# ros2_controllers.yaml
go2_joint_controller:
  ros__parameters:
    type: position_controllers/JointGroupPositionController
    joints:
      - FL_hip
      - FL_thigh
      - FL_calf
      - FR_hip
      - FR_thigh
      - FR_calf
      - RL_hip
      - RL_thigh
      - RL_calf
      - RR_hip
      - RR_thigh
      - RR_calf

    # PD gains
    gains:
      FL_hip:  {p: 160.0, d: 5.0, i: 0.0}
      FL_thigh: {p: 160.0, d: 5.0, i: 0.0}
      FL_calf:  {p: 160.0, d: 5.0, i: 0.0}
      # ... (repeat for all joints)

    # Velocity limits (rad/s)
    velocity_limits:
      FL_hip: 23.0
      FL_thigh: 23.0
      FL_calf: 14.0
      # ... (repeat for all joints)
```

---

## Data Collection Considerations

### Joint Velocity Monitoring

When collecting data, ensure joint velocities stay within limits:

```python
import numpy as np

# Check if joint velocities are within limits
def check_velocity_limits(joint_vel):
    """
    joint_vel: array of 12 joint velocities (rad/s)
    Returns: True if all within limits
    """
    # Hip and thigh indices: 0,1,3,4,6,7,9,10
    # Calf indices: 2,5,8,11

    hip_thigh_mask = [0, 1, 3, 4, 6, 7, 9, 10]
    calf_mask = [2, 5, 8, 11]

    # Check hip/thigh limits (23 rad/s)
    if np.any(np.abs(joint_vel[hip_thigh_mask]) > 23.0):
        return False

    # Check calf limits (14 rad/s)
    if np.any(np.abs(joint_vel[calf_mask]) > 14.0):
        return False

    return True
```

### Velocity Saturation Detection

Track how often joints hit velocity limits:

```python
def count_saturated_joints(joint_vel):
    """Count number of joints at or near velocity limits."""
    hip_thigh_mask = [0, 1, 3, 4, 6, 7, 9, 10]
    calf_mask = [2, 5, 8, 11]

    saturated = 0
    saturated += np.sum(np.abs(joint_vel[hip_thigh_mask]) > 22.5)  # 98% of limit
    saturated += np.sum(np.abs(joint_vel[calf_mask]) > 13.5)      # 96% of limit

    return saturated
```

This metric is already collected in the 74-column data format as `torque_saturation_count`.

---

## Physical Constraints

### Maximum Achievable Velocities

In practice, maximum joint velocities depend on:

1. **Load**: Higher loads reduce max velocity
2. **Temperature**: Motors derate at high temperatures
3. **Battery voltage**: Lower voltage reduces performance
4. **Control frequency**: Higher frequencies may limit velocity

**Realistic operational limits** (with safety margin):
- Hip/Thigh: ~20 rad/s (87% of max)
- Calf: ~12 rad/s (86% of max)

---

## Comparison with Other Unitree Robots

| Robot | Hip/Thigh Vel (rad/s) | Calf Vel (rad/s) | Total DOF |
|-------|----------------------|-----------------|-----------|
| **Go2** | **23.0** | **14.0** | **12** |
| Go1 | 21.0 | 13.0 | 12 |
| A1 | 21.0 | 16.0 | 12 |
| B2 | 23.0 | 14.0 | 12 |
| H1 (humanoid) | Variable | Variable | 29 |
| G1 (humanoid) | Variable | Variable | 29 |

---

## References

- **Source**: `/home/drl-68/unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`
- **IsaacLab Documentation**: Go2 asset configuration
- **Unitree Official Specs**: Go2 product page

---

**Last Updated**: 2026-02-08
