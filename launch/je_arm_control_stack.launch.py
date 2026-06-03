from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_yaml(file_path):
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    use_fake_executor = LaunchConfiguration("use_fake_executor")
    start_rviz = LaunchConfiguration("start_rviz")
    arm_mode = LaunchConfiguration("arm_mode")

    pkg_share = get_package_share_directory("my_jearm_moveit_config")

    # Conda-clean environment for all child nodes
    runtime_env = {
        "PYTHONPATH": "",
        "PYTHONHOME": "",
        "CONDA_PREFIX": "",
        "CONDA_DEFAULT_ENV": "",
        "CONDA_PROMPT_MODIFIER": "",
        "CONDA_SHLVL": "",
        "LD_PRELOAD": "",
        "PATH": ["/usr/bin:/bin:/usr/sbin:/sbin:", EnvironmentVariable("PATH", default_value="")],
        "LD_LIBRARY_PATH": [
            "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
        ],
    }

    # ── real hardware: delegate to existing moveit_rviz.launch.py ──────────────
    moveit_launch = os.path.join(pkg_share, "launch", "moveit_rviz.launch.py")
    real_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=UnlessCondition(use_fake_executor),
        launch_arguments={"start_rviz": start_rviz, "arm_mode": arm_mode}.items(),
    )

    # ── fake mode: minimal stack without ros2_control ───────────────────────────
    def build_fake_params(urdf_name, srdf_name, kinematics_file, joint_limits_file, pipelines_file, controllers_file):
        urdf_file = os.path.join(pkg_share, 'config', urdf_name)
        with open(urdf_file, 'r') as f:
            robot_description = f.read()

        srdf_file = os.path.join(pkg_share, 'config', srdf_name)
        with open(srdf_file, 'r') as f:
            robot_description_semantic = f.read()

        kinematics_yaml = _load_yaml(os.path.join(pkg_share, 'config', kinematics_file))
        joint_limits_yaml = _load_yaml(os.path.join(pkg_share, 'config', joint_limits_file))
        pipelines_yaml = _load_yaml(os.path.join(pkg_share, 'config', pipelines_file))
        controllers_yaml = _load_yaml(os.path.join(pkg_share, 'config', controllers_file))

        fake_params = {
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'robot_description_kinematics': kinematics_yaml,
            'robot_description_planning': joint_limits_yaml,
        }
        if pipelines_yaml:
            fake_params.update(pipelines_yaml)
        if controllers_yaml:
            fake_params.update(controllers_yaml)
        fake_params['planning_plugin'] = 'ompl_interface/OMPLPlanner'
        fake_params['trajectory_execution'] = {
            'manage_controllers': True,
            'allowed_execution_timeout_scaling': 100.0,
            'execution_duration_monitoring': False,
        }
        return fake_params, robot_description

    fake_params_dual, robot_description_dual = build_fake_params(
        'DUAL_JEARM.urdf',
        'DUAL_JEARM.srdf',
        'kinematics.yaml',
        'joint_limits.yaml',
        'planning_pipelines.yaml',
        'moveit_controllers.yaml')

    fake_params_single, robot_description_single = build_fake_params(
        'L_JEARM.urdf',
        'L_JEARM.srdf',
        'kinematics_single.yaml',
        'joint_limits_single.yaml',
        'planning_pipelines_single.yaml',
        'moveit_controllers_single.yaml')

    fake_dual_condition = IfCondition(PythonExpression(["'", use_fake_executor, "' == 'true' and '", arm_mode, "' == 'dual'"]))
    fake_single_condition = IfCondition(PythonExpression(["'", use_fake_executor, "' == 'true' and '", arm_mode, "' == 'single'"]))
    fake_dual_with_rviz = IfCondition(PythonExpression(["'", use_fake_executor, "' == 'true' and '", arm_mode, "' == 'dual' and '", start_rviz, "' == 'true'"]))
    fake_single_with_rviz = IfCondition(PythonExpression(["'", use_fake_executor, "' == 'true' and '", arm_mode, "' == 'single' and '", start_rviz, "' == 'true'"]))

    robot_state_publisher_dual = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_dual}],
        additional_env=runtime_env,
        condition=fake_dual_condition,
    )

    robot_state_publisher_single = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_single}],
        additional_env=runtime_env,
        condition=fake_single_condition,
    )

    static_tf_dual = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        additional_env=runtime_env,
        condition=fake_dual_condition,
    )

    static_tf_single = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        additional_env=runtime_env,
        condition=fake_single_condition,
    )

    fake_executor_dual = Node(
        package='my_jearm_moveit_config',
        executable='fake_trajectory_executor',
        parameters=[{
            'ee_link': 'Link17',
            'joint_names': [
                'joint11', 'joint12', 'joint13', 'joint14', 'joint15', 'joint16', 'joint17',
                'joint21', 'joint22', 'joint23', 'joint24', 'joint25', 'joint26', 'joint27'
            ]
        }],
        output='screen',
        additional_env=runtime_env,
        condition=fake_dual_condition,
    )

    fake_executor_single = Node(
        package='my_jearm_moveit_config',
        executable='fake_trajectory_executor',
        parameters=[{
            'ee_link': 'Link7',
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        }],
        output='screen',
        additional_env=runtime_env,
        condition=fake_single_condition,
    )

    move_group_fake_dual = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[fake_params_dual],
                additional_env=runtime_env,
                remappings=[('/compute_cartesian_path', '/compute_cartesian_path_raw')],
                condition=fake_dual_condition,
            )
        ],
    )

    move_group_fake_single = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[fake_params_single],
                additional_env=runtime_env,
                remappings=[('/compute_cartesian_path', '/compute_cartesian_path_raw')],
                condition=fake_single_condition,
            )
        ],
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'moveit.rviz')
    rviz_fake_dual = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[fake_params_dual],
                additional_env=runtime_env,
                condition=fake_dual_condition,
            )
        ],
        condition=fake_dual_with_rviz,
    )

    rviz_fake_single = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[fake_params_single],
                additional_env=runtime_env,
                condition=fake_single_condition,
            )
        ],
        condition=fake_single_with_rviz,
    )

    cartesian_path_diagnoser = Node(
        package='my_jearm_moveit_config',
        executable='cartesian_path_diagnoser',
        name='cartesian_path_diagnoser',
        output='screen',
        additional_env=runtime_env,
        parameters=[{
            'service_name': '/compute_cartesian_path',
            'raw_service_name': '/compute_cartesian_path_raw',
            'ik_service_name': '/compute_ik',
            'state_validity_service_name': '/check_state_validity',
        }],
    )

    arm_state_logger_dual = Node(
        package='my_jearm_moveit_config',
        executable='arm_state_logger',
        name='arm_state_logger',
        output='screen',
        additional_env=runtime_env,
        parameters=[{
            'base_link': 'base_link',
            'ee_link': 'Link17',
            'log_period_sec': 5.0,
            'joint_names': [
                'joint11', 'joint12', 'joint13', 'joint14', 'joint15', 'joint16', 'joint17',
                'joint21', 'joint22', 'joint23', 'joint24', 'joint25', 'joint26', 'joint27'
            ],
        }],
        condition=fake_dual_condition,
    )

    arm_state_logger_single = Node(
        package='my_jearm_moveit_config',
        executable='arm_state_logger',
        name='arm_state_logger',
        output='screen',
        additional_env=runtime_env,
        parameters=[{
            'base_link': 'base_link',
            'ee_link': 'Link7',
            'log_period_sec': 5.0,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'],
        }],
        condition=fake_single_condition,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_executor",
            default_value="true",
            description="true = fake mode (no ros2_control); false = real hardware",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Whether to start RViz",
        ),
        DeclareLaunchArgument(
            "arm_mode",
            default_value="dual",
            description="arm mode: dual or single",
        ),
        cartesian_path_diagnoser,
        real_stack,
        robot_state_publisher_dual,
        robot_state_publisher_single,
        static_tf_dual,
        static_tf_single,
        arm_state_logger_dual,
        arm_state_logger_single,
        fake_executor_dual,
        fake_executor_single,
        move_group_fake_dual,
        move_group_fake_single,
        rviz_fake_dual,
        rviz_fake_single,
    ])
