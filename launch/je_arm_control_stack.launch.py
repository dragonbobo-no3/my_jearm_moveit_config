from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
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
        launch_arguments={"start_rviz": start_rviz}.items(),
    )

    # ── fake mode: minimal stack without ros2_control ───────────────────────────
    urdf_file = os.path.join(pkg_share, 'config', 'L_JEARM.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    srdf_file = os.path.join(pkg_share, 'config', 'L_JEARM.srdf.xacro')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    kinematics_yaml    = _load_yaml(os.path.join(pkg_share, 'config', 'kinematics.yaml'))
    joint_limits_yaml  = _load_yaml(os.path.join(pkg_share, 'config', 'joint_limits.yaml'))
    pipelines_yaml     = _load_yaml(os.path.join(pkg_share, 'config', 'planning_pipelines.yaml'))
    controllers_yaml   = _load_yaml(os.path.join(pkg_share, 'config', 'moveit_controllers.yaml'))

    fake_params = {
        'robot_description':          robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning':   joint_limits_yaml,
    }
    if pipelines_yaml:   fake_params.update(pipelines_yaml)
    if controllers_yaml: fake_params.update(controllers_yaml)
    fake_params['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    fake_params['trajectory_execution'] = {
        'manage_controllers': True,
        'allowed_execution_timeout_scaling': 100.0,
        'execution_duration_monitoring': False,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}],
        additional_env=runtime_env,
        condition=IfCondition(use_fake_executor),
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        additional_env=runtime_env,
        condition=IfCondition(use_fake_executor),
    )

    fake_executor = Node(
        package='my_jearm_moveit_config',
        executable='fake_trajectory_executor',
        output='screen',
        additional_env=runtime_env,
        condition=IfCondition(use_fake_executor),
    )

    move_group_fake = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[fake_params],
                additional_env=runtime_env,
                condition=IfCondition(use_fake_executor),
            )
        ],
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'moveit.rviz')
    rviz_fake = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[fake_params],
                additional_env=runtime_env,
                condition=IfCondition(use_fake_executor),
            )
        ],
        condition=IfCondition(start_rviz),
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
        real_stack,
        robot_state_publisher,
        static_tf,
        fake_executor,
        move_group_fake,
        rviz_fake,
    ])
