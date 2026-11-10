"""MK2 SDK coordinates: receive-only feedback by default, explicit control opt-in."""

import importlib.util
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('hardware_mode')
    if mode not in ('feedback', 'control'):
        raise ValueError('hardware_mode must be feedback or control')
    if mode == 'control' and value('confirm_enable') != 'true':
        raise RuntimeError(
            'Control requires confirm_enable:=true. JeZmqSystemHardware activates '
            'the robot with Switch=true and sends position hold commands. '
            'Feedback mode never enables the arm.'
        )
    # Control remains blocked for physical/unknown backends. The pure SUB probe
    # below must positively verify the simulation-only plugin's isolation and
    # ready state before any write-capable controller_manager is created.
    state_port, command_port = int(value('state_port')), int(value('command_port'))
    timeout = float(value('state_timeout'))
    startup_timeout = float(value('startup_timeout'))
    if not all(1 <= port <= 65535 for port in (state_port, command_port)):
        raise ValueError('ZMQ ports must be in 1..65535')
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('state_timeout must be finite and positive')
    if not math.isfinite(startup_timeout) or startup_timeout <= 0:
        raise ValueError('startup_timeout must be finite and positive')

    share = Path(get_package_share_directory('my_jearm_moveit_config'))
    config = share / 'config'

    def load_yaml(name):
        with (config / name).open(encoding='utf-8') as stream:
            return yaml.safe_load(stream)

    support_path = share / 'launch/lh_mk2_support.py'
    spec = importlib.util.spec_from_file_location('lh_mk2_support', support_path)
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    hardware = load_yaml('hardware_mk2.yaml')
    urdf, srdf, limits = support.build_hardware_description(
        (config / 'LH_MK2.urdf').read_text(encoding='utf-8'),
        (config / 'LH_MK2.srdf').read_text(encoding='utf-8'), hardware,
        value('robot_ip'), state_port, command_port,
    )
    params = {
        'robot_description': urdf,
        'robot_description_semantic': srdf,
        'robot_description_kinematics': load_yaml('kinematics_single.yaml'),
        'robot_description_planning': limits,
        'allow_trajectory_execution': mode == 'control',
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    params.update(load_yaml('ompl_planning_mk2.yaml'))
    if mode == 'control':
        params.update(load_yaml('moveit_controllers_single.yaml'))
        params['moveit_manage_controllers'] = False
        params['trajectory_execution'] = {
            'allowed_execution_duration_scaling': 1.5,
            'allowed_goal_duration_margin': 2.0,
            'allowed_start_tolerance': 0.02,
            'execution_duration_monitoring': True,
        }

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        parameters=[params], output='screen',
        remappings=[('/compute_cartesian_path', '/compute_cartesian_path_raw')],
    )
    visualization = [
        move_group,
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(share / 'rviz/moveit.rviz')], parameters=[params],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ]
    common = [
        LogInfo(msg=f'LH_MK2 hardware_mode={mode}; Robot0[0..6] -> joint1..joint7 '
                '(SDK radians unchanged); original teach-pendant URDF axes/origins preserved.'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}], output='screen',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
        ),
        Node(
            package='my_jearm_moveit_config', executable='arm_state_logger', output='screen',
            parameters=[{'base_link': 'base_link', 'ee_link': 'sdk_flange',
                         'joint_names': [f'joint{i}' for i in range(1, 8)], 'log_period_sec': 5.0}],
        ),
        Node(
            package='my_jearm_moveit_config', executable='cartesian_path_diagnoser',
            parameters=[{'service_name': '/compute_cartesian_path',
                         'raw_service_name': '/compute_cartesian_path_raw',
                         'ik_service_name': '/compute_ik',
                         'state_validity_service_name': '/check_state_validity'}], output='screen',
        ),
    ]

    def stop_on_exit(process, reason):
        return RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason=reason))],
        ))

    bridge_args = ['--robot-ip', value('robot_ip'), '--state-port', str(state_port),
                   '--timeout', str(timeout), '--startup-timeout', str(startup_timeout),
                   '--robot-key', 'Robot0']
    if mode == 'feedback':
        # No ros2_control_node, no fake executor, no publisher on command_port.
        bridge = Node(package='my_jearm_moveit_config', executable='mk2_state_bridge.py',
                      arguments=bridge_args, output='screen')
        readiness = support.FeedbackReadyGate()

        def after_feedback(event):
            # Humble OnProcessIO passes only the event; capture the actual
            # launch context from _launch_setup rather than adding a second arg.
            if context.is_shutdown:
                return []
            if not readiness.feed(event.text):
                return []
            return [LogInfo(msg='Valid SDK feedback received; starting model, MoveIt and RViz.'),
                    *common, *visualization]

        return [stop_on_exit(bridge, 'MK2 feedback lost or receiver stopped'),
                RegisterEventHandler(OnProcessIO(target_action=bridge, on_stdout=after_feedback)),
            LogInfo(msg='FEEDBACK / PLAN ONLY: trajectory execution is disabled. '
                'RViz Plan & Execute may report planning success without motion; '
                'no trajectory commands are sent. Isolated SDK simulation control '
                'requires an explicit relaunch with hardware_mode:=control '
                'confirm_enable:=true and a successful simulation preflight.'),
                LogInfo(msg='Waiting for controller State feedback before starting MoveIt/RViz; no fake fallback.'),
                bridge]

    # Preflight only subscribes to state. Never start the write-capable driver
    # unless complete, in-bounds Robot0 feedback has actually arrived.
    probe = Node(
        package='my_jearm_moveit_config', executable='mk2_state_bridge.py',
        arguments=bridge_args + ['--probe-only', '--require-command-bridge',
                     '--require-simulation-control',
                                 '--limits-file', str(config / 'hardware_mk2.yaml')],
        output='screen',
    )
    control = Node(
        package='my_jearm_moveit_config', executable='mk2_shared_clock_control_node',
        output='screen',
        parameters=[{'robot_description': urdf}, str(config / 'ros2_controllers_mk2.yaml')],
        additional_env={'JE_ARM_CONTROLLED_ARM': 'left'},
    )
    broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '35'],
    )
    trajectory = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['jearm_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '35'],
    )

    def after_success(actions, label):
        def callback(event, _context):
            if _context.is_shutdown:
                return []
            if event.returncode == 0:
                return actions
            return [LogInfo(msg=f'{label} failed; refusing to continue.'),
                    EmitEvent(event=Shutdown(reason=f'{label} failed'))]
        return callback

    return [
        RegisterEventHandler(OnProcessExit(target_action=probe,
            on_exit=after_success([*common, control], 'Read-only hardware preflight'))),
        RegisterEventHandler(OnProcessStart(target_action=control, on_start=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,
            on_exit=after_success([trajectory], 'Joint state broadcaster startup'))),
        RegisterEventHandler(OnProcessExit(target_action=trajectory,
            on_exit=after_success(visualization, 'Trajectory controller startup'))),
        stop_on_exit(control, 'MK2 controller manager stopped'),
        LogInfo(msg='SDK SIMULATION CONTROL ONLY: verified isolated simulator required; '
                'Switch=true requests validated servo hold, never physical enable. '
                'No automatic trajectory is submitted.'),
        probe,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_mode', default_value='feedback', choices=['feedback', 'control']),
        DeclareLaunchArgument('confirm_enable', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.99'),
        DeclareLaunchArgument('state_port', default_value='8000'),
        DeclareLaunchArgument('command_port', default_value='8001'),
        DeclareLaunchArgument('state_timeout', default_value='5.0'),
        DeclareLaunchArgument('startup_timeout', default_value='15.0'),
        OpaqueFunction(function=_launch_setup),
    ])