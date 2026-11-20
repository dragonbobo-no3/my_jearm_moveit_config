"""Second-generation left arm: real feedback default, explicit offline/control."""

from pathlib import Path
import importlib.util

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    share = Path(get_package_share_directory('my_jearm_moveit_config'))
    if LaunchConfiguration('use_fake_executor').perform(context).lower() != 'true':
        prefix = Path(get_package_prefix('my_jearm_moveit_config'))
        required = [share / relative for relative in (
            'launch/lh_mk2_hardware.launch.py', 'launch/lh_mk2_support.py',
            'config/hardware_mk2.yaml',
        )]
        required.append(prefix / 'lib/my_jearm_moveit_config/mk2_state_bridge.py')
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            raise RuntimeError(
                'MK2 hardware resources are not installed: ' + ', '.join(missing) +
                '. Rebuild/install my_jearm_moveit_config and source the workspace setup. '
                'An existing launch-file symlink does not install newly added files.'
            )
        names = ('hardware_mode', 'confirm_enable', 'start_rviz', 'robot_ip',
                 'state_port', 'command_port', 'state_timeout', 'startup_timeout', 'model',
                 'waist', 'waist_port', 'waist_sign')
        mode = LaunchConfiguration('hardware_mode').perform(context)
        message = (
            'LH_MK2 SDK FEEDBACK: same Robot0 State protocol for physical or SDK simulation; no enable or motion. '
            'For offline CAD simulation, set use_fake_executor:=true.'
            if mode == 'feedback' else
            'LH_MK2 SDK SIMULATION CONTROL ONLY: isolated simulation control-capability '
            'preflight and confirm_enable are required; physical control is blocked.'
        )
        return [LogInfo(msg=message), IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/lh_mk2_hardware.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items(),
        )]

    config = share / 'config'

    def load_yaml(name):
        with (config / name).open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    if LaunchConfiguration('model').perform(context) == 'dual':
        spec = importlib.util.spec_from_file_location('lh_mk2_support', share / 'launch/lh_mk2_support.py')
        support = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(support)
        description, semantic = support.adapt_dual_model(
            (config / 'DUAL_JEARM.urdf').read_text(encoding='utf-8'),
            (config / 'DUAL_JEARM.srdf').read_text(encoding='utf-8'))
        ee_link = 'Left_Link7'
    else:
        description = (config / 'LH_MK2.urdf').read_text(encoding='utf-8')
        semantic = (config / 'LH_MK2.srdf').read_text(encoding='utf-8')
        ee_link = 'Link7'
    params = {
        'robot_description': description,
        'robot_description_semantic': semantic,
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
            parameters=[{'base_link': 'base_link', 'ee_link': ee_link,
                         'controlled_arm': 'left', 'joint_names': joint_names}],
            output='screen',
        ),
        Node(
            package='my_jearm_moveit_config', executable='arm_state_logger',
            parameters=[{'base_link': 'base_link', 'ee_link': ee_link,
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
        DeclareLaunchArgument('model', default_value='dual', choices=['dual', 'lh_mk2'],
            description='dual: DUAL_JEARM waist model (left arm only); lh_mk2: original single-arm model.'),
        DeclareLaunchArgument('waist', default_value='fixed', choices=['fixed', 'feedback'],
            description='Hardware only. feedback: RViz waist follows CAN waist feedback (read-only).'),
        DeclareLaunchArgument('waist_port', default_value='8005'),
        DeclareLaunchArgument('waist_sign', default_value='1', choices=['1', '-1']),
        DeclareLaunchArgument(
            'use_fake_executor', default_value='false', choices=['true', 'false'],
            description='false (default): real SDK feedback; true: explicit offline CAD simulation.',
        ),
        DeclareLaunchArgument('hardware_mode', default_value='control', choices=['feedback', 'control']),
        DeclareLaunchArgument('confirm_enable', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.99'),
        DeclareLaunchArgument('state_port', default_value='8000'),
        DeclareLaunchArgument('command_port', default_value='8001'),
        DeclareLaunchArgument('state_timeout', default_value='5.0'),
        DeclareLaunchArgument('startup_timeout', default_value='15.0'),
        OpaqueFunction(function=_launch_setup),
    ])