# Gazebo Data Collection Setup - AI Assistant Prompt

This document provides instructions for setting up data collection in Gazebo simulator to match the IsaacLab configuration.

---

## 1. Test Commands & Conditions

### Overview
Collect telemetry data from 6 trained Go2 robot policies executing 4 specific locomotion tasks in Gazebo simulation.

### Test Configurations

Test **6 policy configurations** (all with ~25000 training iterations, except Implicit configs):

| Configuration | Model Type | Domain Randomization | Checkpoint | Iterations |
|--------------|------------|---------------------|------------|-----------|
| **MLP + Custom DR** | MLP (feedforward) | ✅ Comprehensive | model_24999.pt | ~25000 |
| **MLP - No DR** | MLP (feedforward) | ❌ None | model_24999.pt | ~25000 |
| **LSTM + DR** | LSTM (recurrent) | ✅ Comprehensive | model_25000.pt | 25000 |
| **LSTM - No DR** | LSTM (recurrent) | ❌ None | model_25000.pt | 25000 |
| **Implicit + DR** | Physics-based | ✅ Comprehensive | model_14600.pt | 14600 (latest) |
| **Implicit - No DR** | Physics-based | ❌ None | model_8100.pt | 8100 (latest) |

### Four Locomotion Tasks

Each task is 20 seconds long (1000 timesteps @ 50 Hz control frequency):

#### Task 1: Standing (Static Stability)
```python
# Stay completely still
velocity_command = [vx=0.0, vy=0.0, wz=0.0]  # For entire 20 seconds
```

**Expected behavior**: Robot maintains standing position with minimal drift

---

#### Task 2: Forward Walking with Speed Transitions
```python
# Time-varying forward velocity
t ∈ [0, 5):   velocity_command = [0.5, 0.0, 0.0]  # Slow walk
t ∈ [5, 10):  velocity_command = [1.0, 0.0, 0.0]  # Normal walk
t ∈ [10, 15): velocity_command = [1.5, 0.0, 0.0]  # Fast walk
t ∈ [15, 20]: velocity_command = [0.8, 0.0, 0.0]  # Moderate walk
```

**Expected behavior**:
- Smooth acceleration/deceleration between speeds
- Forward motion tracking commanded velocities
- Stable gait throughout

---

#### Task 3: Turn in Place with Direction Changes
```python
# Yaw rotation with direction reversal
t ∈ [0, 5):   velocity_command = [0.0, 0.0, +0.5]  # Slow CCW turn
t ∈ [5, 10):  velocity_command = [0.0, 0.0, +1.0]  # Normal CCW turn
t ∈ [10, 15): velocity_command = [0.0, 0.0, -1.0]  # Normal CW turn (direction change!)
t ∈ [15, 20]: velocity_command = [0.0, 0.0, +1.5]  # Fast CCW turn
```

**Expected behavior**:
- Minimal translation (turning in place)
- Smooth direction reversal at t=10s
- Stable rotation rate tracking

---

#### Task 4: Combined Walk + Turn Trajectories
```python
# Complex maneuvers combining translation and rotation
t ∈ [0, 5):   velocity_command = [0.8, 0.0, +0.6]  # Right arc
t ∈ [5, 7):   velocity_command = [1.0, 0.0,  0.0]  # Straight walk
t ∈ [7, 12):  velocity_command = [0.8, 0.0, -0.6]  # Left arc
t ∈ [12, 15): velocity_command = [1.2, 0.0,  0.0]  # Fast straight
t ∈ [15, 20]: velocity_command = [0.5, 0.0, +1.0]  # Tight turn
```

**Expected behavior**:
- Coordinated linear + angular motion
- Smooth trajectory following
- Most challenging task

---

### Test Matrix

- **Total episodes**: 200 per configuration (50 repeats × 4 tasks)
- **Total data files**: 1200 CSV files (6 configs × 200 episodes)
- **Episode duration**: 20 seconds each
- **Control frequency**: 50 Hz (20ms per control cycle)
- **Timesteps per episode**: 1000

---

## 2. Parameters to Collect

### Data Collection Requirements

**Output format**: CSV files with **74 columns** × **~1000 rows** per episode

**File naming**: `locomotion_log_gazebo_ep{000-199}_{timestamp}.csv`

### Column Structure (74 total)

#### A. Metadata (5 columns)
```
timestamp_sim        # Simulation time (seconds)
timestamp_wall       # Wall-clock time (seconds)
episode_id           # Episode number (0-199)
episode_seed         # Random seed for reproducibility
control_cycle        # Control iteration (0-999)
```

#### B. Performance Metrics (3 columns)
```
rtf                  # Real-time factor (sim_dt / wall_dt)
control_latency_ms   # Control loop latency (milliseconds)
actual_control_dt    # Actual control timestep (should be ~0.02s)
```

#### C. Base Position (7 columns)
```
base_pos_x           # Base position X (world frame, meters)
base_pos_y           # Base position Y (world frame, meters)
base_pos_z           # Base position Z (world frame, meters)
base_quat_x          # Quaternion X component
base_quat_y          # Quaternion Y component
base_quat_z          # Quaternion Z component
base_quat_w          # Quaternion W component
```

#### D. Base Orientation (4 columns)
```
base_roll            # Roll angle (radians)
base_pitch           # Pitch angle (radians)
base_yaw             # Yaw angle (radians)
base_height          # Height above ground (meters)
```

#### E. Base Velocity (6 columns)
```
base_lin_vel_x       # Linear velocity X (body frame, m/s)
base_lin_vel_y       # Linear velocity Y (body frame, m/s)
base_lin_vel_z       # Linear velocity Z (body frame, m/s)
base_ang_vel_x       # Angular velocity X (body frame, rad/s)
base_ang_vel_y       # Angular velocity Y (body frame, rad/s)
base_ang_vel_z       # Angular velocity Z (body frame, rad/s)
```

#### F. Commanded Velocities (3 columns)
```
cmd_vx               # Commanded forward velocity (m/s)
cmd_vy               # Commanded lateral velocity (m/s)
cmd_wz               # Commanded yaw rate (rad/s)
```

**CRITICAL**: These must match the task-specific time-varying commands!

#### G. Joint States (36 columns)

Go2 has 12 joints: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf

```
# Joint positions (12 columns)
joint_pos_0 ... joint_pos_11    # Radians

# Joint velocities (12 columns)
joint_vel_0 ... joint_vel_11    # Rad/s

# Joint accelerations (12 columns)
joint_acc_0 ... joint_acc_11    # Rad/s²
```

#### H. Actions & Commands (48 columns)

```
# Policy output (12 columns)
action_0 ... action_11          # Raw policy output (normalized)

# Commanded joint positions (12 columns)
cmd_joint_pos_0 ... cmd_joint_pos_11    # Target positions (radians)

# Commanded torques (12 columns)
cmd_torque_0 ... cmd_torque_11  # Desired torques (Nm)

# Actual measured torques (12 columns)
actual_torque_0 ... actual_torque_11    # Measured torques (Nm)
```

#### I. Action History (24 columns)

For temporal consistency analysis:

```
# Previous action (12 columns)
prev_action_1_0 ... prev_action_1_11

# Two steps ago (12 columns)
prev_action_2_0 ... prev_action_2_11
```

#### J. Contact & Foot State (12 columns)

```
# Binary contact (4 columns)
foot_contact_binary_FL          # 1.0 = contact, 0.0 = no contact
foot_contact_binary_FR
foot_contact_binary_RL
foot_contact_binary_RR

# Contact forces (4 columns, Newtons)
foot_contact_force_FL
foot_contact_force_FR
foot_contact_force_RL
foot_contact_force_RR

# Foot slip velocity (4 columns, m/s)
foot_slip_velocity_FL
foot_slip_velocity_FR
foot_slip_velocity_RL
foot_slip_velocity_RR
```

#### K. Derived Metrics (4 columns)

```
instantaneous_power     # Sum of |torque × velocity| (Watts)
torque_saturation_count # Number of saturated joints this cycle
action_smoothness       # ||action_t - action_{t-1}|| (L2 norm)
gravity_alignment       # Dot product of gravity projection with [0,0,-1]
```

---

## 3. ROS 2 Topics (Gazebo-Specific)

### Required ROS 2 Topics for Data Collection

#### Input Topics (Subscribe)

**Robot State**:
```bash
/joint_states                    # sensor_msgs/JointState
  - position[12]: Joint angles (rad)
  - velocity[12]: Joint velocities (rad/s)
  - effort[12]: Joint torques (Nm)

/imu                            # sensor_msgs/Imu
  - orientation: Quaternion (x, y, z, w)
  - angular_velocity: Body-frame gyro (rad/s)
  - linear_acceleration: Body-frame accelerometer (m/s²)

/odom                           # nav_msgs/Odometry
  - pose.pose.position: World-frame position (m)
  - pose.pose.orientation: Quaternion
  - twist.twist.linear: Body-frame linear velocity (m/s)
  - twist.twist.angular: Body-frame angular velocity (rad/s)
```

**Contact Sensing** (if available):
```bash
/foot_contact_FL                # Custom or gazebo_msgs/ContactsState
/foot_contact_FR
/foot_contact_RL
/foot_contact_RR
```

**Policy Commands**:
```bash
/policy_output                  # std_msgs/Float32MultiArray
  - data[12]: Raw policy actions

/joint_commands                 # Custom message
  - position[12]: Target joint positions
  - torque[12]: Desired torques
```

#### Output Topics (Publish)

**Velocity Commands** (to policy):
```bash
/cmd_vel                        # geometry_msgs/Twist
  - linear.x: Forward velocity (vx)
  - linear.y: Lateral velocity (vy)
  - angular.z: Yaw rate (wz)
```

**Control Commands** (to robot):
```bash
/joint_group_position_controller/commands    # std_msgs/Float64MultiArray
  - Joint position commands

/joint_torque_controller/commands            # Custom
  - Joint torque commands
```

---

### Topic Synchronization Requirements

**CRITICAL**: All topics must be synchronized to 50 Hz control loop

**Suggested approach**:
```python
import message_filters
from message_filters import ApproximateTimeSynchronizer

# Synchronize multiple topics
joint_sub = message_filters.Subscriber('/joint_states', JointState)
imu_sub = message_filters.Subscriber('/imu', Imu)
odom_sub = message_filters.Subscriber('/odom', Odometry)

ts = ApproximateTimeSynchronizer(
    [joint_sub, imu_sub, odom_sub],
    queue_size=10,
    slop=0.01  # 10ms tolerance
)
ts.registerCallback(data_collection_callback)
```

---

## 4. Gazebo-Specific Considerations

### Physics Configuration

Match IsaacLab simulation parameters:

```xml
<!-- Gazebo world file -->
<physics type="ode">
  <max_step_size>0.005</max_step_size>      <!-- 5ms physics timestep -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>200</real_time_update_rate>
</physics>
```

### Controller Configuration

Use `ros2_control` with PD controllers matching IsaacLab:

```yaml
# controller_config.yaml
go2_joint_controller:
  type: effort_controllers/JointGroupEffortController
  joints:
    - FL_hip_joint
    - FL_thigh_joint
    # ... (all 12 joints)

  # PD gains (tune to match IsaacLab)
  gains:
    FL_hip_joint:
      p: 20.0
      d: 0.5
    # ... (tune for each joint)
```

### Domain Randomization (Optional)

For configurations with DR, randomize Gazebo parameters at episode start:

- **Friction coefficients**: 0.4-1.25
- **Mass properties**: Base mass ±1-3 kg
- **Initial joint positions**: ±30% from default
- **Contact properties**: Restitution 0.0-0.15

---

## 5. Data Collection Workflow

### Step-by-Step Process

#### 1. Initialize Gazebo
```bash
ros2 launch go2_gazebo go2_world.launch.py
```

#### 2. Load Trained Policy
```bash
# Convert ONNX model to ROS 2 node
ros2 run go2_policy policy_node \
  --model model_25000.onnx \
  --config Unitree-Go2-Velocity-MLP-Custom
```

#### 3. Start Data Logger
```bash
ros2 run go2_data_collection logger_node \
  --output logs/data_collection_gazebo/mlp_custom/ \
  --episodes episode_configs_4tasks.yaml \
  --num_episodes 200
```

#### 4. Execute Episodes

For each episode (0-199):
1. Reset robot to initial state from `episode_configs_4tasks.yaml`
2. Publish time-varying velocity commands according to task
3. Record all 74 data columns at 50 Hz
4. Save to CSV after 1000 timesteps (20 seconds)

---

## 6. Validation Checklist

After data collection, verify:

- [ ] **File count**: 1200 CSV files (6 configs × 200 episodes)
- [ ] **File size**: Each file ~100-200 KB
- [ ] **Row count**: Each CSV has exactly 1000 data rows (+ 1 header)
- [ ] **Column count**: Each CSV has exactly 74 columns
- [ ] **No missing data**: No NaN or empty values
- [ ] **Velocity commands match**: `cmd_vx`, `cmd_vy`, `cmd_wz` match task specs
- [ ] **Real-time factor**: RTF > 0.9 (simulation running near real-time)
- [ ] **Timestamp continuity**: `timestamp_sim` increases by ~0.02s per row

### Sample Validation Script
```python
import pandas as pd
import glob

# Check Task 2 (Walking) velocity progression
files = glob.glob('logs/data_collection_gazebo/mlp_custom/locomotion_log_gazebo_ep001_*.csv')
df = pd.read_csv(files[0])

# Verify time-varying commands
assert df.iloc[0]['cmd_vx'] == 0.5    # t=0-5s: slow
assert df.iloc[250]['cmd_vx'] == 1.0  # t=5-10s: normal
assert df.iloc[500]['cmd_vx'] == 1.5  # t=10-15s: fast
assert df.iloc[750]['cmd_vx'] == 0.8  # t=15-20s: moderate

print("✅ Velocity commands correct!")
```

---

## 7. Example ROS 2 Data Logger Node (Python Skeleton)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pandas as pd
import yaml
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class GazeboDataCollector(Node):
    def __init__(self):
        super().__init__('gazebo_data_collector')

        # Load episode configs
        with open('episode_configs_4tasks.yaml', 'r') as f:
            self.episodes = yaml.safe_load(f)['episodes']

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers (use message_filters for sync)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        # ... (add other subscriptions)

        # Data buffer
        self.data_buffer = []
        self.control_cycle = 0

        # Timer at 50 Hz
        self.timer = self.create_timer(0.02, self.control_loop)

    def control_loop(self):
        # Get current command from episode config
        cmd = self.get_command_for_time(self.control_cycle * 0.02)

        # Publish velocity command
        twist = Twist()
        twist.linear.x = cmd[0]  # vx
        twist.linear.y = cmd[1]  # vy
        twist.angular.z = cmd[2]  # wz
        self.cmd_vel_pub.publish(twist)

        # Collect data row (all 74 columns)
        data_row = self.collect_data_row(cmd)
        self.data_buffer.append(data_row)

        self.control_cycle += 1

        # Save after 1000 steps
        if self.control_cycle >= 1000:
            self.save_episode()
            self.reset_episode()

    def collect_data_row(self, cmd):
        return {
            'timestamp_sim': self.control_cycle * 0.02,
            'cmd_vx': cmd[0],
            'cmd_vy': cmd[1],
            'cmd_wz': cmd[2],
            # ... (collect all 74 columns)
        }

    def save_episode(self):
        df = pd.DataFrame(self.data_buffer)
        filename = f'locomotion_log_gazebo_ep{self.episode_id:03d}.csv'
        df.to_csv(filename, index=False)
        self.get_logger().info(f'Saved {filename}')

def main():
    rclpy.init()
    collector = GazeboDataCollector()
    rclpy.spin(collector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 8. Expected Output Structure

```
logs/data_collection_gazebo/
├── mlp_custom/
│   ├── locomotion_log_gazebo_ep000_2026-02-08_15-30-00.csv (1000 rows × 74 cols)
│   ├── locomotion_log_gazebo_ep001_2026-02-08_15-30-25.csv
│   └── ... (200 files total)
├── mlp_no_dr/
│   └── ... (200 files)
├── lstm_dr/
│   └── ... (200 files)
├── lstm_no_dr/
│   └── ... (200 files)
├── implicit_dr/
│   └── ... (200 files)
└── implicit/
    └── ... (200 files)
```

**Total size**: ~2.4 GB (1200 files × ~2 MB each)

---

## 9. Key Differences: Gazebo vs IsaacLab

| Aspect | IsaacLab | Gazebo |
|--------|----------|--------|
| **Physics engine** | PhysX (GPU) | ODE/Bullet (CPU) |
| **Data access** | Direct tensor access | ROS 2 topics |
| **Control loop** | Synchronous stepping | ROS 2 timer (async) |
| **Observation** | Dict/tensor | Multiple ROS messages |
| **Velocity commands** | Direct command_manager | `/cmd_vel` topic |
| **Joint torques** | `robot.data.joint_torques` | `/joint_states.effort` |
| **Real-time factor** | Typically 1.0-10.0 | Typically 0.5-1.0 |

---

## 10. Troubleshooting

### Common Issues

**Issue**: Real-time factor < 0.9
- **Solution**: Reduce `max_step_size` or use faster computer

**Issue**: Topic synchronization failures
- **Solution**: Increase `ApproximateTimeSynchronizer` slop parameter

**Issue**: Missing contact data
- **Solution**: Add Gazebo contact sensors to URDF

**Issue**: Torque measurements noisy
- **Solution**: Apply low-pass filter or increase Gazebo `max_step_size`

---

## Summary

This setup replicates the IsaacLab data collection in Gazebo with:
- ✅ **6 trained policies** (MLP, LSTM, Implicit × DR/No-DR)
- ✅ **4 locomotion tasks** (Standing, Walking, Turning, Combined)
- ✅ **200 episodes per config** (50 repeats × 4 tasks)
- ✅ **74-column telemetry** at 50 Hz
- ✅ **Time-varying velocity commands** matching task specifications

**Total output**: 1200 CSV files with comprehensive robot state data for sim-to-sim transfer analysis.

---

**Good luck with your Gazebo data collection! 🚀**
