# IsaacLab → Gazebo Parameter Matching Guide

## Critical Parameters to Match for Vistec_ex_ws

---

## 1. Robot Spawn Point & Initial State

### IsaacLab Current Settings:

```python
# From: unitree_rl_lab/assets/robots/unitree.py

init_state = ArticulationCfg.InitialStateCfg(
    # Spawn position (world frame)
    pos=(0.0, 0.0, 0.4),  # x, y, z in meters

    # Joint positions (radians)
    joint_pos={
        ".*R_hip_joint": -0.1,     # Right hip: -0.1 rad (-5.7°)
        ".*L_hip_joint": 0.1,      # Left hip: +0.1 rad (+5.7°)
        "F[L,R]_thigh_joint": 0.8, # Front thigh: 0.8 rad (45.8°)
        "R[L,R]_thigh_joint": 1.0, # Rear thigh: 1.0 rad (57.3°)
        ".*_calf_joint": -1.5,     # All calf: -1.5 rad (-85.9°)
    },

    # Initial velocities (all zero)
    joint_vel={".*": 0.0},
)
```

### Gazebo URDF/Launch File Should Match:

```xml
<!-- spawn_go2.launch.py or world file -->
<node pkg="gazebo_ros" type="spawn_model"
      args="-urdf -model go2
            -x 0.0    <!-- Match IsaacLab pos[0] -->
            -y 0.0    <!-- Match IsaacLab pos[1] -->
            -z 0.4    <!-- Match IsaacLab pos[2] = 0.4m height -->
            -R 0.0    <!-- Roll = 0 -->
            -P 0.0    <!-- Pitch = 0 -->
            -Y 0.0"/> <!-- Yaw = 0 -->

<!-- Initial joint states in URDF or controller -->
<joint name="FL_hip_joint">   <origin value="-0.1"/></joint>  <!-- Left hip -->
<joint name="FR_hip_joint">   <origin value="0.1"/></joint>   <!-- Right hip -->
<joint name="RL_hip_joint">   <origin value="-0.1"/></joint>
<joint name="RR_hip_joint">   <origin value="0.1"/></joint>

<joint name="FL_thigh_joint"> <origin value="0.8"/></joint>   <!-- Front thigh -->
<joint name="FR_thigh_joint"> <origin value="0.8"/></joint>
<joint name="RL_thigh_joint"> <origin value="1.0"/></joint>   <!-- Rear thigh -->
<joint name="RR_thigh_joint"> <origin value="1.0"/></joint>

<joint name="FL_calf_joint">  <origin value="-1.5"/></joint>  <!-- All calf -->
<joint name="FR_calf_joint">  <origin value="-1.5"/></joint>
<joint name="RL_calf_joint">  <origin value="-1.5"/></joint>
<joint name="RR_calf_joint">  <origin value="-1.5"/></joint>
```

---

## 2. Physics Timestep & Control Frequency

### IsaacLab Settings:

```python
# Physics timestep
sim.dt = 0.005  # 5ms = 200 Hz physics

# Control decimation
decimation = 4  # Control every 4 physics steps

# Effective control frequency
control_freq = 1 / (sim.dt * decimation)  # 50 Hz
control_dt = 0.02  # 20ms
```

### Gazebo World File Should Match:

```xml
<physics type="ode">
  <max_step_size>0.005</max_step_size>        <!-- Match IsaacLab sim.dt -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>200</real_time_update_rate>  <!-- 1/max_step_size -->

  <!-- ODE solver parameters -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### ROS 2 Controller Configuration:

```yaml
# controller_manager.yaml
update_rate: 50  # Match IsaacLab control frequency (50 Hz)

go2_joint_controller:
  type: position_controllers/JointGroupPositionController
  joints: [FL_hip_joint, FL_thigh_joint, ...] # All 12 joints
```

---

## 3. PD Controller Gains (ACTUATOR-SPECIFIC!)

### ⚠️ CRITICAL: Different Actuators Use Different PD Gains!

IsaacLab uses **THREE different actuator types**, each with different gains:

#### MLP/LSTM Actuators (Neural Network-Based)

```python
# From: unitree_actuators.py
# Used by: MLP + DR, MLP - DR, LSTM + DR, LSTM - DR

actuators = {
    "legs": UnitreeActuatorCfg_Go2_MLP(
        joint_names_expr=[".*"],
        stiffness={".*": 25.0},   # Kp = 25.0 (LOW)
        damping={".*": 0.5},      # Kd = 0.5 (LOW)
    )
}
```

**Why lower gains?** Neural network actuators learn actuator dynamics internally, so they need much lower PD gains.

#### Implicit Actuators (Physics-Based IdealPD)

```python
# From: unitree.py
# Used by: Implicit + DR, Implicit - DR

actuators = {
    "legs": IdealPDActuatorCfg(
        joint_names_expr=[".*_hip_.*", ".*_thigh_.*", ".*_calf_.*"],
        stiffness=160.0,  # Kp = 160.0 (HIGH)
        damping=5.0,      # Kd = 5.0 (HIGH)
    )
}
```

**Why higher gains?** Physics-based controllers need higher gains for tight position tracking.

---

### Policy-to-Gains Mapping for Gazebo

When replicating each policy in Gazebo, use these **exact PD gains**:

| Policy | Actuator Type | Kp (Stiffness) | Kd (Damping) |
|--------|---------------|----------------|--------------|
| **MLP + Custom DR** | Neural Network | **25.0** | **0.5** |
| **MLP - No DR** | Neural Network | **25.0** | **0.5** |
| **LSTM + DR** | Neural Network | **25.0** | **0.5** |
| **LSTM - No DR** | Neural Network | **25.0** | **0.5** |
| **Implicit + DR** | IdealPD | **160.0** | **5.0** |
| **Implicit - No DR** | IdealPD | **160.0** | **5.0** |

---

### Gazebo ros2_control Configuration

You'll need **TWO separate controller configurations**:

#### Configuration A: For MLP/LSTM Policies (LOW GAINS)

```yaml
# ros2_controllers_mlp_lstm.yaml
go2_joint_controller:
  ros__parameters:
    joints:
      - FL_hip_joint
      - FL_thigh_joint
      - FL_calf_joint
      # ... (all 12 joints)

    # LOW PD gains for neural network actuators
    gains:
      FL_hip_joint:   {p: 25.0, d: 0.5, i: 0.0}
      FL_thigh_joint: {p: 25.0, d: 0.5, i: 0.0}
      FL_calf_joint:  {p: 25.0, d: 0.5, i: 0.0}
      # ... (repeat for all joints)

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

#### Configuration B: For Implicit Policies (HIGH GAINS)

```yaml
# ros2_controllers_implicit.yaml
go2_joint_controller:
  ros__parameters:
    joints:
      - FL_hip_joint
      - FL_thigh_joint
      - FL_calf_joint
      # ... (all 12 joints)

    # HIGH PD gains for physics-based actuators
    gains:
      FL_hip_joint:   {p: 160.0, d: 5.0, i: 0.0}
      FL_thigh_joint: {p: 160.0, d: 5.0, i: 0.0}
      FL_calf_joint:  {p: 160.0, d: 5.0, i: 0.0}
      # ... (repeat for all joints)

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

**Usage**:
- Use Configuration A when testing MLP/LSTM policies
- Use Configuration B when testing Implicit policies

---

## 4. Ground/Terrain Properties

### IsaacLab Settings:

```python
# Fixed environment (no randomization)
physics_material = RigidBodyMaterialCfg(
    friction_combine_mode="multiply",
    restitution_combine_mode="multiply",
    static_friction=1.0,   # Default ground friction
    dynamic_friction=1.0,
    restitution=0.0,       # No bounce
)

# Terrain: Flat ground plane
terrain = TerrainImporterCfg(
    prim_path="/World/ground",
    terrain_type="plane",  # Flat plane
    collision_group=-1,
)
```

### Gazebo World/URDF:

```xml
<!-- Gazebo world file -->
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>      <!-- Static friction = 1.0 -->
            <mu2>1.0</mu2>    <!-- Dynamic friction = 1.0 -->
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.0</restitution_coefficient>
        </bounce>
      </surface>
    </collision>
  </link>
</model>
```

---

## 5. Robot Mass & Inertia

### IsaacLab Settings:

```python
# No mass randomization in fixed environment
# Base mass from URDF/USD: ~15 kg (Go2 specification)

# No additional mass added
add_base_mass = None  # Disabled for fixed env
```

### Gazebo URDF:

```xml
<!-- Go2 base link -->
<link name="base">
  <inertial>
    <mass value="15.0"/>  <!-- Match Go2 spec (~15 kg) -->
    <origin xyz="0 0 0"/>
    <inertia ixx="0.17" ixy="0" ixz="0"
             iyy="0.58" iyz="0"
             izz="0.64"/>
  </inertial>
  <!-- ... -->
</link>

<!-- Ensure all link masses match IsaacLab USD file -->
```

**Action Required**: Extract exact mass/inertia from IsaacLab USD file:
```bash
# Check USD file for exact values
# Or export from IsaacSim Inspector
```

---

## 6. Joint Limits

### IsaacLab Settings:

```python
# From GO2_JOINT_SPECIFICATIONS.md
joint_limits = {
    "hip":   {"lower": -1.047, "upper": 1.047},   # ±60°
    "thigh": {"lower": -0.663, "upper": 2.966},   # -38° to +170°
    "calf":  {"lower": -2.721, "upper": -0.837},  # -156° to -48°
}

# Velocity limits
velocity_limits = {
    "hip":   23.0,  # rad/s
    "thigh": 23.0,
    "calf":  14.0,
}

# Effort limits
effort_limits = {
    "hip":   200.0,  # Nm
    "thigh": 200.0,
    "calf":  320.0,
}
```

### Gazebo URDF:

```xml
<!-- Example for FL_hip -->
<joint name="FL_hip_joint" type="revolute">
  <limit effort="200" velocity="23.0" lower="-1.047" upper="1.047"/>
  <dynamics damping="5.0" friction="0.01"/>
</joint>

<!-- FL_thigh -->
<joint name="FL_thigh_joint" type="revolute">
  <limit effort="200" velocity="23.0" lower="-0.663" upper="2.966"/>
  <dynamics damping="5.0" friction="0.01"/>
</joint>

<!-- FL_calf -->
<joint name="FL_calf_joint" type="revolute">
  <limit effort="320" velocity="14.0" lower="-2.721" upper="-0.837"/>
  <dynamics damping="5.0" friction="0.01"/>
</joint>

<!-- Repeat for all 12 joints -->
```

---

## 7. Observation Noise (DISABLED for Fixed Env)

### IsaacLab Settings:

```python
# In play mode (data collection):
observations.policy.enable_corruption = False

# All observations clean (no noise):
# - base_ang_vel: No ±0.2 rad/s noise
# - projected_gravity: No ±0.05 noise
# - joint_pos: No ±0.01 rad noise
# - joint_vel: No ±1.5 rad/s noise
```

### Gazebo:

```python
# ROS 2 data logger: Do NOT add artificial noise
# Use raw sensor data from:
# - /joint_states (clean)
# - /imu (with real sensor noise characteristics)
# - /odom (clean)
```

**Note**: Real Gazebo sensors have inherent noise, but should be minimal.

---

## 8. Gravity

### IsaacLab Settings:

```python
gravity = (0.0, 0.0, -9.81)  # m/s² (Earth gravity, -Z direction)
```

### Gazebo World:

```xml
<physics>
  <gravity>0 0 -9.81</gravity>  <!-- Match IsaacLab exactly -->
</physics>
```

---

## 9. Domain Randomization (MUST BE DISABLED)

### IsaacLab Fixed Environment:

```python
# ALL events disabled:
events = {
    "physics_material": None,       # No friction randomization
    "add_base_mass": None,          # No mass randomization
    "push_robot": None,             # No velocity pushes
    "base_external_force": None,    # No force disturbances
    "randomize_motor_strength": None, # Fixed PD gains
    # ... all other events disabled
}
```

### Gazebo:

```yaml
# Do NOT use:
# - Randomization plugins
# - Disturbance generators
# - Variable friction
# - Variable mass
# - External forces

# Use FIXED, DETERMINISTIC physics only
```

---

## 10. Episode Configuration Matching

### IsaacLab:

Uses `episode_configs_4tasks.yaml`:
- 200 episodes (50 per task)
- Specific initial states per episode
- Time-varying velocity commands

### Gazebo:

```python
# Must load SAME episode config file
with open('episode_configs_4tasks.yaml', 'r') as f:
    episodes = yaml.safe_load(f)['episodes']

# For each episode:
# 1. Set initial state from config['initial_state']
# 2. Apply velocity commands from config['command_sequence']
# 3. Record same 74 columns
# 4. Save as matching filename
```

---

## Quick Verification Checklist

Use this to ensure IsaacLab and Gazebo match:

- [ ] **Spawn position**: (0.0, 0.0, 0.4) meters
- [ ] **Spawn orientation**: (0, 0, 0) roll/pitch/yaw
- [ ] **Initial joint positions**: Match table above
- [ ] **Physics timestep**: 0.005s (5ms)
- [ ] **Control frequency**: 50 Hz (20ms)
- [ ] **PD gains**: **ACTUATOR-SPECIFIC!**
  - [ ] MLP/LSTM policies: Kp=25.0, Kd=0.5
  - [ ] Implicit policies: Kp=160.0, Kd=5.0
- [ ] **Ground friction**: μ=1.0 (static/dynamic)
- [ ] **Ground restitution**: 0.0 (no bounce)
- [ ] **Joint velocity limits**: Hip/Thigh=23.0, Calf=14.0 rad/s
- [ ] **Joint effort limits**: Hip/Thigh=200, Calf=320 Nm
- [ ] **Gravity**: (0, 0, -9.81) m/s²
- [ ] **No randomization**: All DR disabled
- [ ] **No disturbances**: No pushes/forces
- [ ] **No observation noise**: Clean sensors
- [ ] **Same episode configs**: Use episode_configs_4tasks.yaml
- [ ] **Same velocity commands**: Match time-varying patterns

---

## Export IsaacLab Settings Script

```python
#!/usr/bin/env python3
"""Export IsaacLab environment parameters for Gazebo matching."""

import yaml

def export_isaaclab_params():
    """Export all critical parameters."""

    params = {
        'spawn_point': {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.4},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
        'initial_joint_positions': {
            'FL_hip': 0.1, 'FR_hip': -0.1, 'RL_hip': 0.1, 'RR_hip': -0.1,
            'FL_thigh': 0.8, 'FR_thigh': 0.8, 'RL_thigh': 1.0, 'RR_thigh': 1.0,
            'FL_calf': -1.5, 'FR_calf': -1.5, 'RL_calf': -1.5, 'RR_calf': -1.5
        },
        'physics': {
            'timestep': 0.005,
            'gravity': [0.0, 0.0, -9.81],
            'control_frequency': 50,
        },
        'pd_gains': {
            'mlp_lstm_actuators': {
                'description': 'For MLP and LSTM policies (neural network actuators)',
                'all_joints': {'p': 25.0, 'd': 0.5, 'i': 0.0}
            },
            'implicit_actuators': {
                'description': 'For Implicit policies (physics-based IdealPD)',
                'all_joints': {'p': 160.0, 'd': 5.0, 'i': 0.0}
            },
            'policy_mapping': {
                'MLP_Custom_DR': 'mlp_lstm_actuators',
                'MLP_No_DR': 'mlp_lstm_actuators',
                'LSTM_DR': 'mlp_lstm_actuators',
                'LSTM_No_DR': 'mlp_lstm_actuators',
                'Implicit_DR': 'implicit_actuators',
                'Implicit_No_DR': 'implicit_actuators'
            }
        },
        'ground': {
            'static_friction': 1.0,
            'dynamic_friction': 1.0,
            'restitution': 0.0
        },
        'joint_limits': {
            'hip': {'lower': -1.047, 'upper': 1.047, 'velocity': 23.0, 'effort': 200.0},
            'thigh': {'lower': -0.663, 'upper': 2.966, 'velocity': 23.0, 'effort': 200.0},
            'calf': {'lower': -2.721, 'upper': -0.837, 'velocity': 14.0, 'effort': 320.0}
        }
    }

    # Save to YAML
    with open('isaaclab_params_for_gazebo.yaml', 'w') as f:
        yaml.dump(params, f, default_flow_style=False)

    print("✅ Saved parameters to: isaaclab_params_for_gazebo.yaml")
    print("\nUse these parameters in your Gazebo Vistec_ex_ws setup!")
    print("\n⚠️  IMPORTANT: Different policies use different PD gains!")
    print("   - MLP/LSTM: Kp=25.0, Kd=0.5")
    print("   - Implicit: Kp=160.0, Kd=5.0")

if __name__ == '__main__':
    export_isaaclab_params()
```

Run this to generate configuration file:
```bash
python export_isaaclab_params.py
```

---

## Summary

**To match Vistec_ex_ws Gazebo setup**:

1. ✅ Use spawn point: (0, 0, 0.4) meters
2. ✅ Use same initial joint positions
3. ✅ Set physics timestep: 5ms
4. ✅ Set control frequency: 50 Hz
5. ⚠️ **Set PD gains (ACTUATOR-SPECIFIC!)**:
   - **MLP/LSTM policies**: Kp=25.0, Kd=0.5
   - **Implicit policies**: Kp=160.0, Kd=5.0
6. ✅ Set ground friction: μ=1.0
7. ✅ Disable ALL randomization
8. ✅ Use same episode configs
9. ✅ Apply same velocity commands

**Reference this document when configuring Gazebo Vistec_ex_ws!** 🤖

---

## ⚠️ CRITICAL WARNING

**DO NOT use the same PD gains for all policies!**

- MLP and LSTM policies were trained with **LOW gains** (Kp=25, Kd=0.5)
- Implicit policies were trained with **HIGH gains** (Kp=160, Kd=5)
- Using wrong gains will cause **significant performance degradation**

Match the gains to the policy type for accurate sim-to-sim comparison!
