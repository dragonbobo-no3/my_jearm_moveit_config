"""MK2 SDK coordinates: receive-only feedback by default, explicit control opt-in."""

import importlib.util
import math
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('hardware_mode')
    backend_mode = value('mode')
    if mode not in ('feedback', 'control', 'shared'):
        raise ValueError('hardware_mode must be feedback, control or shared')
    if backend_mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    backend = 'physical' if backend_mode == 'real' else 'simulation'
    # Explicit mode:=real is the physical opt-in. The driver's internal
    # confirmation and exact native backend/configuration checks remain intact.
    physical_confirmed = backend_mode == 'real'
    if mode != 'feedback' and value('confirm_enable') != 'true':
        raise RuntimeError(
            'Control requires confirm_enable:=true. JeZmqSystemHardware activates '
            'the robot with Switch=true and sends position hold commands. '
            'Feedback mode never enables the arm.'
        )
    # Verify the selected backend before creating a write-capable manager.
    # Feedback mode creates no driver, regardless of the selected backend.
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
    model = value('model')
    waist = value('waist')
    if waist == 'feedback' and model != 'dual':
        raise ValueError('waist:=feedback requires model:=dual')
    if model == 'dual':
        source_urdf, source_srdf = support.adapt_dual_model(
            (config / 'DUAL_JEARM.urdf').read_text(encoding='utf-8'),
            (config / 'DUAL_JEARM.srdf').read_text(encoding='utf-8'),
            waist=waist == 'feedback')
        flange_parent = 'Left_Link7'
    else:
        source_urdf = (config / 'LH_MK2.urdf').read_text(encoding='utf-8')
        source_srdf = (config / 'LH_MK2.srdf').read_text(encoding='utf-8')
        flange_parent = 'Link7'
    urdf, srdf, limits = support.build_hardware_description(
        source_urdf, source_srdf, hardware,
        value('robot_ip'), state_port, command_port,
        control_backend=backend if mode != 'feedback' else 'simulation',
        confirm_physical_control=physical_confirmed if mode != 'feedback' else False,
        shared_ownership=mode == 'shared', flange_parent=flange_parent,
    )
    params = {
        'robot_description': urdf,
        'robot_description_semantic': srdf,
        'robot_description_kinematics': load_yaml('kinematics_single.yaml'),
        'robot_description_planning': limits,
        'allow_trajectory_execution': mode != 'feedback',
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    params.update(load_yaml('ompl_planning_mk2.yaml'))
    if mode != 'feedback':
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
    rviz_enabled = value('start_rviz') == 'true'
    visualization = [
        move_group,
    ]
    if rviz_enabled:
        visualization.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(share / 'rviz/moveit.rviz')], parameters=[params],
        ))

    def readonly_frontend():
        # A new process/context, not a dynamic parameter change on an executor
        # that may still hold old goals. Never pass controller configuration.
        readonly_params = {key: params[key] for key in (
            'robot_description', 'robot_description_semantic',
            'robot_description_kinematics', 'robot_description_planning',
            'publish_robot_description', 'publish_robot_description_semantic',
            'publish_planning_scene', 'publish_geometry_updates',
            'publish_state_updates', 'publish_transforms_updates')}
        readonly_params.update(load_yaml('ompl_planning_mk2.yaml'))
        readonly_params['allow_trajectory_execution'] = False
        # Humble MoveGroupInterface waits for execute_trajectory at startup.
        # Keep the action discoverable: MoveIt's allow=false context aborts
        # requests with CONTROL_FAILED before accessing an execution manager.
        # Disabling the capability makes the RViz client wait indefinitely.
        nodes = [Node(
            package='moveit_ros_move_group', executable='move_group',
            parameters=[readonly_params], output='screen',
            remappings=[('/compute_cartesian_path', '/compute_cartesian_path_raw')])]
        return nodes

    fallback = None
    if mode == 'control':
        fallback_spec = importlib.util.spec_from_file_location(
            'mk2_control_fallback', share / 'launch/mk2_control_fallback.py')
        fallback_module = importlib.util.module_from_spec(fallback_spec)
        fallback_spec.loader.exec_module(fallback_module)
        # RViz is a client, not the execution authority. Retain the window;
        # discard the old MoveGroup server and all of its execution context.
        fallback = fallback_module.ControlFallback([move_group], readonly_frontend, visualization[1:])
    common = [
        LogInfo(msg=f'LH_MK2 model={model} waist={waist} mode={backend_mode} hardware_mode={mode}; Robot0[0..6] -> joint1..joint7 '
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
    if waist == 'feedback':
        # Read-only CAN waist feedback (controller mk2_waist_server); never commands.
        common.append(Node(
            package='my_jearm_moveit_config', executable='mk2_waist_bridge.py', output='screen',
            arguments=['--robot-ip', value('robot_ip'), '--port', value('waist_port'),
                       '--sign', value('waist_sign')],
        ))

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
                'no trajectory commands are sent. SDK simulation/physical control '
                'requires an explicit relaunch with hardware_mode:=control '
                'confirm_enable:=true, mode:=sim or mode:=real, and matching backend preflight.'),
                LogInfo(msg='Waiting for controller State feedback before starting MoveIt/RViz; no fake fallback.'),
                bridge]

    # Preflight only subscribes to state. Never start the write-capable driver
    # unless complete, in-bounds Robot0 feedback has actually arrived.
    probe = Node(
        package='my_jearm_moveit_config', executable='mk2_state_bridge.py',
        arguments=bridge_args + ['--probe-only', '--require-command-bridge',
                     ('--require-physical-control' if backend == 'physical'
                      else '--require-simulation-control'),
                                 '--limits-file', str(config / 'hardware_mk2.yaml')],
        output='screen',
    )
    # Keep measured positions independent of the write driver's lifecycle.
    # Native faults revoke control, not the validity of finite joint feedback.
    # This SUB-only bridge owns /joint_states; the controller broadcaster uses
    # local diagnostic topics so stopped driver state cannot masquerade as live.
    feedback = Node(
        package='my_jearm_moveit_config', executable='mk2_state_bridge.py',
        arguments=bridge_args, output='screen',
    )
    controller_parameters = [{'robot_description': urdf}, str(config / 'ros2_controllers_mk2.yaml')]
    if mode == 'shared':
        controller_parameters.append({
            'mk2_shared_ownership': True,
            'hardware_components_initial_state.inactive': ['lh_mk2_system'],
        })
    if backend == 'simulation':
        # SDK simulation only: native sim following tolerance is 0.5 rad. Let
        # JTC abort just before it (0.45) instead of the physical 0.22. A param
        # file is required: dict params would only target controller_manager.
        with tempfile.NamedTemporaryFile('w', prefix='mk2_sim_jtc_', suffix='.yaml',
                                         delete=False, encoding='utf-8') as stream:
            yaml.safe_dump({'jearm_controller': {'ros__parameters': {'constraints': {
                f'joint{i}': {'trajectory': 0.45, 'goal': 0.03} for i in range(1, 8)}}}},
                stream)
        controller_parameters.append(stream.name)
    control = Node(
        package='my_jearm_moveit_config', executable='mk2_shared_clock_control_node',
        output='screen',
        parameters=controller_parameters,
        additional_env={'JE_ARM_CONTROLLED_ARM': 'left'},
    )
    broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '35'] + (['--inactive'] if mode == 'shared' else []),
    )
    trajectory = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['jearm_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '35'] + (['--inactive'] if mode == 'shared' else []),
    )

    def after_success(actions, label):
        def callback(event, _context):
            if _context.is_shutdown:
                return []
            if fallback is not None and fallback.control_lost:
                return []  # Late spawner responses must not revive execution.
            if event.returncode == 0:
                if fallback is not None and actions is visualization:
                    return fallback.start_frontend(_context)
                return actions
            return [LogInfo(msg=f'{label} failed; refusing to continue.'),
                    EmitEvent(event=Shutdown(reason=f'{label} failed'))]
        return callback

    def after_control_exit(event, _context):
        if _context.is_shutdown:
            return []
        if fallback is not None:
            return fallback.controller_exited(event, _context)
        return [
            LogInfo(msg=f'MK2 control process exited with code {event.returncode}; '
                'shutting down this control stack to prevent stale-state execution. '
                'No automatic clear/recover/restart. For pendant jogging with PC observation, '
                'relaunch with mode:=' + backend_mode + ' hardware_mode:=feedback; '
                'do not jog while hardware_mode:=control owns servo control.'),
            EmitEvent(event=Shutdown(reason='MK2 controller manager stopped')),
        ]

    if mode == 'shared':
        # No ready/control-capability preflight: axes can be disabled or moving
        # under pendant ownership at startup. Acquire service performs strict
        # capability, idle, freshness and configuration checks before activation.
        owner = Node(package='my_jearm_moveit_config', executable='mk2_control_owner.py',
            parameters=[{'robot_ip': value('robot_ip'), 'state_port': state_port,
                         'backend': 'physical' if backend_mode == 'real' else 'sdk_simulation',
                         'limits_file': str(config / 'hardware_mk2.yaml')}], output='screen')
        return [
            RegisterEventHandler(OnProcessStart(target_action=control, on_start=[broadcaster])),
            RegisterEventHandler(OnProcessExit(target_action=broadcaster,
                on_exit=after_success([trajectory], 'Inactive broadcaster startup'))),
            RegisterEventHandler(OnProcessExit(target_action=trajectory,
                on_exit=after_success([owner, *visualization], 'Inactive trajectory controller startup'))),
            stop_on_exit(owner, 'MK2 ownership supervisor stopped; refusing unowned control'),
            stop_on_exit(feedback, 'MK2 shared feedback stopped'),
            RegisterEventHandler(OnProcessExit(target_action=control, on_exit=after_control_exit)),
            LogInfo(msg='SHARED: initial PENDANT, no automatic servo takeover. '
                '/mk2/control_owner/set_moveit requests ownership; wait for state confirmation. '
                'Do not jog while MOVEIT/ACQUIRING/RELEASING. Old firmware refuses acquire.'),
            feedback, *common, control,
        ]

    return [
        *fallback.handlers(),
        RegisterEventHandler(OnProcessExit(target_action=probe,
            on_exit=after_success([feedback, *common, control], 'Read-only hardware preflight'))),
        RegisterEventHandler(OnProcessStart(target_action=control, on_start=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,
            on_exit=after_success([trajectory], 'Joint state broadcaster startup'))),
        RegisterEventHandler(OnProcessExit(target_action=trajectory,
            on_exit=after_success(visualization, 'Trajectory controller startup'))),
        stop_on_exit(feedback, 'MK2 independent feedback lost or receiver stopped'),
        RegisterEventHandler(OnProcessExit(target_action=control, on_exit=after_control_exit)),
        LogInfo(msg=f'MK2 CONTROL mode={backend_mode} backend={backend}: exact backend preflight required; '
            'Switch=true requests validated servo hold, never power/clear/recover. '
            'No automatic trajectory is submitted. Physical exit uses latched stop; '
            'simulation clean release is not enabled for physical hardware.'),
        LogInfo(msg='hardware_mode:=control is PC servo ownership, NOT pendant-follow mode. '
            'For pendant jogging with live RViz feedback, use hardware_mode:=feedback. '
            'On control loss the write process stops; after the execution server exits, '
            'a plan-only server starts while RViz stays open. No control recovery.'),
        LogInfo(msg='Live /joint_states comes from the independent read-only SDK receiver; '
            'controller state is diagnostic-only under /joint_state_broadcaster. '
            'Live feedback does not mean control is available or a fault has cleared. '
            'Do not jog from the pendant while PC servo control owns the arm.'),
        probe,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_mode', default_value='feedback', choices=['feedback', 'control', 'shared']),
        DeclareLaunchArgument('confirm_enable', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real'],
            description='sim: isolated SDK simulation; real: explicit physical servo takeover. No automatic power changes.'),
        DeclareLaunchArgument('start_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('model', default_value='dual', choices=['dual', 'lh_mk2'],
            description='dual: DUAL_JEARM waist model, left arm = joint1..7, waist/right arm fixed; '
                        'lh_mk2: original single-arm LH_MK2 model.'),
        DeclareLaunchArgument('waist', default_value='fixed', choices=['fixed', 'feedback'],
            description='fixed: waist at zero; feedback: waist_joint follows CAN feedback '
                        '(controller mk2_waist_server, read-only). Not planned by MoveIt.'),
        DeclareLaunchArgument('waist_port', default_value='8005'),
        DeclareLaunchArgument('waist_sign', default_value='1', choices=['1', '-1'],
            description='URDF waist angle = waist_sign * protocol angle'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.99'),
        DeclareLaunchArgument('state_port', default_value='8000'),
        DeclareLaunchArgument('command_port', default_value='8001'),
        DeclareLaunchArgument('state_timeout', default_value='5.0'),
        DeclareLaunchArgument('startup_timeout', default_value='15.0'),
        OpaqueFunction(function=_launch_setup),
    ])