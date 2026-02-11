#!/usr/bin/env python3
"""Export IsaacLab environment parameters for Gazebo Vistec_ex_ws matching."""

import yaml
import json

def export_isaaclab_params():
    """Export all critical parameters for Gazebo matching."""

    params = {
        'metadata': {
            'source': 'IsaacLab Go2 Velocity Task',
            'purpose': 'Match Gazebo Vistec_ex_ws environment',
            'generated': '2026-02-09'
        },

        'spawn_point': {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.4},
            'orientation_euler': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'orientation_quat': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        },

        'initial_joint_positions_rad': {
            'FL_hip': 0.1,
            'FR_hip': -0.1,
            'RL_hip': 0.1,
            'RR_hip': -0.1,
            'FL_thigh': 0.8,
            'FR_thigh': 0.8,
            'RL_thigh': 1.0,
            'RR_thigh': 1.0,
            'FL_calf': -1.5,
            'FR_calf': -1.5,
            'RL_calf': -1.5,
            'RR_calf': -1.5
        },

        'initial_joint_positions_deg': {
            'FL_hip': 5.73,
            'FR_hip': -5.73,
            'RL_hip': 5.73,
            'RR_hip': -5.73,
            'FL_thigh': 45.84,
            'FR_thigh': 45.84,
            'RL_thigh': 57.30,
            'RR_thigh': 57.30,
            'FL_calf': -85.94,
            'FR_calf': -85.94,
            'RL_calf': -85.94,
            'RR_calf': -85.94
        },

        'physics': {
            'timestep_sec': 0.005,
            'timestep_ms': 5.0,
            'physics_update_rate_hz': 200,
            'control_decimation': 4,
            'control_frequency_hz': 50,
            'control_period_sec': 0.02,
            'control_period_ms': 20.0,
            'gravity_m_s2': [0.0, 0.0, -9.81],
            'gravity_magnitude': 9.81
        },

        'pd_controller': {
            'WARNING': '⚠️ DIFFERENT ACTUATORS USE DIFFERENT GAINS!',
            'actuator_types': {
                'mlp_lstm': {
                    'description': 'Neural network actuators (MLP/LSTM policies)',
                    'policies': ['MLP + Custom DR', 'MLP - No DR', 'LSTM + DR', 'LSTM - No DR'],
                    'all_joints': {'p': 25.0, 'd': 0.5, 'i': 0.0},
                    'individual': {
                        'FL_hip':   {'p': 25.0, 'd': 0.5},
                        'FR_hip':   {'p': 25.0, 'd': 0.5},
                        'RL_hip':   {'p': 25.0, 'd': 0.5},
                        'RR_hip':   {'p': 25.0, 'd': 0.5},
                        'FL_thigh': {'p': 25.0, 'd': 0.5},
                        'FR_thigh': {'p': 25.0, 'd': 0.5},
                        'RL_thigh': {'p': 25.0, 'd': 0.5},
                        'RR_thigh': {'p': 25.0, 'd': 0.5},
                        'FL_calf':  {'p': 25.0, 'd': 0.5},
                        'FR_calf':  {'p': 25.0, 'd': 0.5},
                        'RL_calf':  {'p': 25.0, 'd': 0.5},
                        'RR_calf':  {'p': 25.0, 'd': 0.5}
                    }
                },
                'implicit': {
                    'description': 'Physics-based IdealPD actuators (Implicit policies)',
                    'policies': ['Implicit + DR', 'Implicit - No DR'],
                    'all_joints': {'p': 160.0, 'd': 5.0, 'i': 0.0},
                    'individual': {
                        'FL_hip':   {'p': 160.0, 'd': 5.0},
                        'FR_hip':   {'p': 160.0, 'd': 5.0},
                        'RL_hip':   {'p': 160.0, 'd': 5.0},
                        'RR_hip':   {'p': 160.0, 'd': 5.0},
                        'FL_thigh': {'p': 160.0, 'd': 5.0},
                        'FR_thigh': {'p': 160.0, 'd': 5.0},
                        'RL_thigh': {'p': 160.0, 'd': 5.0},
                        'RR_thigh': {'p': 160.0, 'd': 5.0},
                        'FL_calf':  {'p': 160.0, 'd': 5.0},
                        'FR_calf':  {'p': 160.0, 'd': 5.0},
                        'RL_calf':  {'p': 160.0, 'd': 5.0},
                        'RR_calf':  {'p': 160.0, 'd': 5.0}
                    }
                }
            },
            'policy_to_actuator_mapping': {
                'Unitree-Go2-Velocity-MLP-Custom': 'mlp_lstm',
                'Unitree-Go2-Velocity-MLP-No-DR': 'mlp_lstm',
                'Unitree-Go2-Velocity-LSTM-DR': 'mlp_lstm',
                'Unitree-Go2-Velocity-LSTM-No-DR': 'mlp_lstm',
                'Unitree-Go2-Velocity-Implicit-DR': 'implicit',
                'Unitree-Go2-Velocity-Implicit': 'implicit'
            }
        },

        'ground_terrain': {
            'type': 'plane',
            'static_friction': 1.0,
            'dynamic_friction': 1.0,
            'restitution': 0.0,
            'friction_combine_mode': 'multiply',
            'restitution_combine_mode': 'multiply'
        },

        'joint_limits': {
            'hip': {
                'position_lower_rad': -1.047,
                'position_upper_rad': 1.047,
                'position_lower_deg': -60.0,
                'position_upper_deg': 60.0,
                'velocity_rad_s': 23.0,
                'velocity_deg_s': 1317.8,
                'effort_nm': 200.0
            },
            'thigh': {
                'position_lower_rad': -0.663,
                'position_upper_rad': 2.966,
                'position_lower_deg': -38.0,
                'position_upper_deg': 170.0,
                'velocity_rad_s': 23.0,
                'velocity_deg_s': 1317.8,
                'effort_nm': 200.0
            },
            'calf': {
                'position_lower_rad': -2.721,
                'position_upper_rad': -0.837,
                'position_lower_deg': -156.0,
                'position_upper_deg': -48.0,
                'velocity_rad_s': 14.0,
                'velocity_deg_s': 802.1,
                'effort_nm': 320.0
            }
        },

        'robot_properties': {
            'total_mass_kg': 15.0,
            'base_mass_kg': 15.0,
            'num_joints': 12,
            'num_legs': 4,
            'joints_per_leg': 3
        },

        'environment_settings': {
            'domain_randomization': 'DISABLED',
            'disturbances': 'DISABLED',
            'observation_noise': 'DISABLED',
            'deterministic': True,
            'episode_configs': 'episode_configs_4tasks.yaml',
            'num_episodes': 200,
            'episode_duration_sec': 20.0,
            'timesteps_per_episode': 1000
        }
    }

    # Save as YAML
    with open('isaaclab_params_for_gazebo.yaml', 'w') as f:
        yaml.dump(params, f, default_flow_style=False, sort_keys=False)

    # Save as JSON (easier for some tools)
    with open('isaaclab_params_for_gazebo.json', 'w') as f:
        json.dump(params, f, indent=2)

    print("✅ Exported IsaacLab parameters:")
    print("   - isaaclab_params_for_gazebo.yaml")
    print("   - isaaclab_params_for_gazebo.json")
    print("\n" + "="*60)
    print("Key Parameters for Vistec_ex_ws Gazebo Setup:")
    print("="*60)
    print(f"  Spawn point: ({params['spawn_point']['position']['x']}, "
          f"{params['spawn_point']['position']['y']}, "
          f"{params['spawn_point']['position']['z']}) m")
    print(f"  Physics timestep: {params['physics']['timestep_ms']} ms")
    print(f"  Control frequency: {params['physics']['control_frequency_hz']} Hz")
    print(f"  Ground friction: μ={params['ground_terrain']['static_friction']}")
    print(f"  Randomization: {params['environment_settings']['domain_randomization']}")
    print("="*60)
    print("\n⚠️  CRITICAL: PD GAINS ARE ACTUATOR-SPECIFIC!")
    print("="*60)
    mlp_lstm = params['pd_controller']['actuator_types']['mlp_lstm']
    implicit = params['pd_controller']['actuator_types']['implicit']
    print(f"  MLP/LSTM Policies: Kp={mlp_lstm['all_joints']['p']}, "
          f"Kd={mlp_lstm['all_joints']['d']}")
    print(f"  Implicit Policies: Kp={implicit['all_joints']['p']}, "
          f"Kd={implicit['all_joints']['d']}")
    print("="*60)
    print("\nPolicy-to-Gains Mapping:")
    for policy, actuator in params['pd_controller']['policy_to_actuator_mapping'].items():
        gains = params['pd_controller']['actuator_types'][actuator]['all_joints']
        print(f"  {policy}: Kp={gains['p']}, Kd={gains['d']}")
    print("="*60)

if __name__ == '__main__':
    export_isaaclab_params()
