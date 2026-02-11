#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_deploy_policy = get_package_share_directory('deploy_policy')

    # Config file path
    config_file = os.path.join(pkg_deploy_policy, 'config', 'go2_deploy.yaml')

    # Launch arguments
    policy_path = LaunchConfiguration('policy_path')
    device = LaunchConfiguration('device')
    control_frequency = LaunchConfiguration('control_frequency')
    use_actuator_network = LaunchConfiguration('use_actuator_network')
    actuator_model_path = LaunchConfiguration('actuator_model_path')
    actuator_scaler_path = LaunchConfiguration('actuator_scaler_path')

    declare_policy_path = DeclareLaunchArgument(
        'policy_path',
        default_value=os.path.join(os.getenv('VISTEC_REPO', os.path.expanduser('~/Vistec_Intern_Exam')),
                                   'trained_models', 'mlp_dr.pt'),
        description='Path to the trained Go2 policy checkpoint'
    )

    declare_device = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Device for inference (cpu or cuda)'
    )

    declare_control_frequency = DeclareLaunchArgument(
        'control_frequency',
        default_value='50.0',
        description='Control loop frequency in Hz (matches IsaacLab: dt=0.005s, decimation=4)'
    )

    declare_use_actuator_network = DeclareLaunchArgument(
        'use_actuator_network',
        default_value='false',
        description='Use MLP actuator network (true) or simple PD control (false)'
    )

    declare_actuator_model_path = DeclareLaunchArgument(
        'actuator_model_path',
        default_value=os.path.join(os.getenv('ACTUATOR_NET', os.path.expanduser('~/Vistec_Intern_Exam/Actuator_net')),
                                   'app', 'resources', 'actuator.pth'),
        description='Path to trained MLP actuator network model'
    )

    declare_actuator_scaler_path = DeclareLaunchArgument(
        'actuator_scaler_path',
        default_value=os.path.join(os.getenv('ACTUATOR_NET', os.path.expanduser('~/Vistec_Intern_Exam/Actuator_net')),
                                   'app', 'resources', 'scaler.pkl'),
        description='Path to actuator network feature scaler'
    )

    # Go2 Policy deployment node
    go2_policy_deploy_node = Node(
        package='deploy_policy',
        executable='go2_deploy_node',
        name='go2_policy_deploy_node',
        output='screen',
        parameters=[{
            'policy_path': policy_path,
            'device': device,
            'control_frequency': control_frequency,
            'use_actuator_network': use_actuator_network,
            'actuator_model_path': actuator_model_path,
            'actuator_scaler_path': actuator_scaler_path,
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        declare_policy_path,
        declare_device,
        declare_control_frequency,
        declare_use_actuator_network,
        declare_actuator_model_path,
        declare_actuator_scaler_path,
        go2_policy_deploy_node,
    ])
