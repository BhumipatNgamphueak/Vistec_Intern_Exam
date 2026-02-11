#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_go2_gazebo_simulation = get_package_share_directory('go2_gazebo_simulation')

    # Paths to files
    urdf_file = os.path.join(pkg_go2_gazebo_simulation, 'urdf', 'go2_base.urdf')
    world_file = os.path.join(pkg_go2_gazebo_simulation, 'worlds', 'empty_fortress.sdf')

    # Set resource paths for Ignition Gazebo to find meshes
    install_dir = os.path.dirname(os.path.dirname(pkg_go2_gazebo_simulation))

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=pkg_go2_gazebo_simulation + ':' + install_dir + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_go2_gazebo_simulation + ':' + install_dir + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.5')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position to spawn the robot'
    )

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position to spawn the robot'
    )

    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.75',
        description='Z position to spawn the robot'
    )

    # Read URDF file and replace package:// URIs with absolute paths to actual mesh location
    unitree_lab_path = os.getenv('UNITREE_LAB', os.path.expanduser('~/Vistec_Intern_Exam/unitree_rl_lab'))
    go2_description_path = os.path.join(unitree_lab_path, 'unitree_ros', 'robots', 'go2_description')
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    urdf_content = urdf_content.replace(
        'package://go2_gazebo_simulation/',
        go2_description_path + '/'
    )
    robot_description_content = urdf_content

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }]
    )

    # Start Gazebo Ignition
    start_gazebo_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # Spawn entity - delayed to let Gazebo start
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'go2_robot',
            '-topic', '/robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ]
    )

    # Delay spawning for 8 seconds
    spawn_entity_delayed = TimerAction(
        period=8.0,
        actions=[spawn_entity]
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Joint state bridge
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/model/go2_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/go2_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # IMU bridge
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        remappings=[
            ('/imu', '/imu/data'),
        ],
        output='screen'
    )

    # Bridge odometry from Gazebo's OdometryPublisher plugin
    # The plugin publishes to /model/go2_robot/odometry_with_covariance in Gazebo
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/go2_robot/odometry_with_covariance@nav_msgs/msg/Odometry[ignition.msgs.OdometryWithCovariance',
        ],
        remappings=[
            ('/model/go2_robot/odometry_with_covariance', '/odom'),
        ],
        output='screen'
    )

    # Bridge pose from Gazebo (for ground truth 3D position including z)
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/go2_robot/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/model/go2_robot/pose', '/robot_pose'),
        ],
        output='screen'
    )

    # IMU to Odometry converter (DISABLED - using URDF plugin instead)
    # imu_to_odom_node = Node(
    #     package='deploy_policy',
    #     executable='imu_to_odom',
    #     name='imu_to_odom',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # Joint effort command bridges (12 joints)
    joint_names = [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ]

    # Effort command bridges
    joint_cmd_force_args = [
        f'/model/go2_robot/joint/{j}/cmd_force@std_msgs/msg/Float64]ignition.msgs.Double'
        for j in joint_names
    ]
    joint_cmd_force_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=joint_cmd_force_args,
        output='screen'
    )

    # Position command bridges (for chirp commander)
    joint_cmd_pos_args = [
        f'/model/go2_robot/joint/{j}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double'
        for j in joint_names
    ]
    joint_cmd_pos_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=joint_cmd_pos_args,
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)

    # Add environment variables first
    ld.add_action(ign_resource_path)
    ld.add_action(gz_sim_resource_path)

    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(clock_bridge)
    ld.add_action(spawn_entity_delayed)
    ld.add_action(joint_state_bridge)
    ld.add_action(imu_bridge)
    ld.add_action(odom_bridge)  # Bridge Gazebo odometry to ROS 2
    ld.add_action(pose_bridge)  # Bridge Gazebo pose for z position
    ld.add_action(joint_cmd_force_bridge)
    ld.add_action(joint_cmd_pos_bridge)

    return ld
