"""Second-generation left arm: isolated, offline-only MoveIt stack."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    # Fail before creating any nodes if a caller requests real hardware.
    if LaunchConfiguration('use_fake_executor').perform(context).lower() != 'true':
        raise RuntimeError(
            'LH_MK2 is offline-only: hardware limits, zero offsets, joint signs '
            'and driver mapping must be verified before enabling real hardware.'
        )

    share = Path(get_package_share_directory('my_jearm_moveit_config'))
    config = share / 'config'

    def load_yaml(name):
        with (config / name).open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    description = (config / 'LH_MK2.urdf').read_text(encoding='utf-8')
    params = {
        'robot_description': description,
        'robot_description_semantic': (config / 'LH_MK2.srdf').read_text(encoding='utf-8'),
        'robot_description_kinematics': load_yaml('kinematics_single.yaml'),
        'robot_description_planning': load_yaml('joint_limits_mk2.yaml'),
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    params.update(load_yaml('ompl_planning_mk2.yaml'))
    params.update(load_yaml('moveit_controllers_single.yaml'))
    joint_names = [f'joint{i}' for i in range(1, 8)]

    return [
        LogInfo(msg=(
            'LH_MK2 OFFLINE ONLY: source +/-31.4 rad bounds are not hardware '
            'limits. Keep real robot controllers out of this ROS domain.'
        )),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description}], output='screen',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'world', '--child-frame-id', 'base_link'],
            output='screen',
        ),
        Node(
            package='my_jearm_moveit_config', executable='fake_trajectory_executor',
            parameters=[{'base_link': 'base_link', 'ee_link': 'Link7',
                         'controlled_arm': 'left', 'joint_names': joint_names}],
            output='screen',
        ),
        Node(
            package='my_jearm_moveit_config', executable='arm_state_logger',
            parameters=[{'base_link': 'base_link', 'ee_link': 'Link7',
                         'joint_names': joint_names, 'log_period_sec': 5.0}],
            output='screen',
        ),
        Node(
            package='my_jearm_moveit_config', executable='cartesian_path_diagnoser',
            parameters=[{
                'service_name': '/compute_cartesian_path',
                'raw_service_name': '/compute_cartesian_path_raw',
                'ik_service_name': '/compute_ik',
                'state_validity_service_name': '/check_state_validity',
            }], output='screen',
        ),
        TimerAction(period=2.0, actions=[Node(
            package='moveit_ros_move_group', executable='move_group',
            parameters=[params], output='screen',
            remappings=[('/compute_cartesian_path', '/compute_cartesian_path_raw')],
        )]),
        TimerAction(period=4.0, actions=[Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', str(share / 'rviz' / 'moveit.rviz')],
            parameters=[params], output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        )]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'use_fake_executor', default_value='true', choices=['true', 'false'],
            description='MK2 currently supports offline fake execution only.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])