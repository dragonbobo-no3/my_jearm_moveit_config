import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import yaml

def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    arm_mode = LaunchConfiguration('arm_mode')
    
    # Use absolute path instead of package discovery
    pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    def build_moveit_config(urdf_name, srdf_name, kinematics_file, joint_limits_file, pipelines_file, controllers_file):
        urdf_file = os.path.join(pkg_share, 'config', urdf_name)
        with open(urdf_file, 'r') as f:
            robot_description = f.read()

        srdf_file = os.path.join(pkg_share, 'config', srdf_name)
        with open(srdf_file, 'r') as f:
            robot_description_semantic = f.read()

        kinematics_yaml = load_yaml(os.path.join(pkg_share, 'config', kinematics_file))
        joint_limits_yaml = load_yaml(os.path.join(pkg_share, 'config', joint_limits_file))
        planning_pipelines_yaml = load_yaml(os.path.join(pkg_share, 'config', pipelines_file))
        controllers_yaml = load_yaml(os.path.join(pkg_share, 'config', controllers_file))

        moveit_config = {
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'robot_description_kinematics': kinematics_yaml,
            'robot_description_planning': joint_limits_yaml,
        }

        if planning_pipelines_yaml:
            moveit_config.update(planning_pipelines_yaml)

        if controllers_yaml:
            moveit_config.update(controllers_yaml)

        moveit_config['planning_plugin'] = 'ompl_interface/OMPLPlanner'
        moveit_config['trajectory_execution'] = {
            'manage_controllers': True,
            'allowed_execution_timeout_scaling': 100.0,
            'execution_duration_monitoring': False
        }

        return moveit_config, robot_description

    moveit_config_dual, robot_description_dual = build_moveit_config(
        'DUAL_JEARM.urdf',
        'DUAL_JEARM.srdf',
        'kinematics.yaml',
        'joint_limits.yaml',
        'planning_pipelines.yaml',
        'moveit_controllers.yaml')

    moveit_config_single, robot_description_single = build_moveit_config(
        'L_JEARM.urdf',
        'L_JEARM.srdf',
        'kinematics_single.yaml',
        'joint_limits_single.yaml',
        'planning_pipelines_single.yaml',
        'moveit_controllers_single.yaml')

    ros2_controllers_file_dual = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    ros2_controllers_file_single = os.path.join(pkg_share, 'config', 'ros2_controllers_single.yaml')

    is_dual = IfCondition(PythonExpression(["'", arm_mode, "' == 'dual'"]))
    is_single = IfCondition(PythonExpression(["'", arm_mode, "' == 'single'"]))
    is_dual_with_rviz = IfCondition(PythonExpression(["'", arm_mode, "' == 'dual' and '", start_rviz, "' == 'true'"]))
    is_single_with_rviz = IfCondition(PythonExpression(["'", arm_mode, "' == 'single' and '", start_rviz, "' == 'true'"]))

    move_group_node_dual = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dual],
        remappings=[("/compute_cartesian_path", "/compute_cartesian_path_raw")],
        condition=is_dual,
    )

    move_group_node_single = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_single],
        remappings=[("/compute_cartesian_path", "/compute_cartesian_path_raw")],
        condition=is_single,
    )

    cartesian_path_diagnoser = Node(
        package="my_jearm_moveit_config",
        executable="cartesian_path_diagnoser",
        name="cartesian_path_diagnoser",
        output="screen",
        parameters=[{
            'service_name': '/compute_cartesian_path',
            'raw_service_name': '/compute_cartesian_path_raw',
            'ik_service_name': '/compute_ik',
            'state_validity_service_name': '/check_state_validity',
        }],
    )

    ros2_control_node_dual = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {'robot_description': robot_description_dual},
            ros2_controllers_file_dual,
        ],
        condition=is_dual,
    )

    ros2_control_node_single = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {'robot_description': robot_description_single},
            ros2_controllers_file_single,
        ],
        condition=is_single,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    jearm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jearm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delayed_controller_spawners_dual = TimerAction(
        period=6.0,
        actions=[joint_state_broadcaster_spawner, jearm_controller_spawner],
        condition=is_dual,
    )

    delayed_controller_spawners_single = TimerAction(
        period=6.0,
        actions=[joint_state_broadcaster_spawner, jearm_controller_spawner],
        condition=is_single,
    )

    # RViz node
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'moveit.rviz')

    rviz_node_dual = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config_dual],
        condition=is_dual,
    )

    rviz_node_single = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config_single],
        condition=is_single,
    )

    # 延迟启动 RViz，确保 /joint_states 已在发布，
    # 这样 RViz 在启动时能读取到实时的机械臂位姿，而不是默认值
    delayed_rviz_dual = TimerAction(
        period=8.0,  # 等待 8 秒，确保 joint_state_broadcaster 已启动并发布状态
        actions=[rviz_node_dual],
        condition=is_dual_with_rviz,
    )

    delayed_rviz_single = TimerAction(
        period=8.0,
        actions=[rviz_node_single],
        condition=is_single_with_rviz,
    )

    # 延迟启动 move_group，确保硬件已初始化
    delayed_move_group_dual = TimerAction(
        period=7.0,  # 等待 7 秒，在 controller spawners 之后但在 RViz 之前
        actions=[move_group_node_dual],
        condition=is_dual,
    )

    delayed_move_group_single = TimerAction(
        period=7.0,
        actions=[move_group_node_single],
        condition=is_single,
    )

    # Robot state publisher
    robot_state_publisher_dual = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_dual}],
        condition=is_dual,
    )

    robot_state_publisher_single = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_single}],
        condition=is_single,
    )

    # Static TF for base link
    static_tf_dual = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        condition=is_dual,
    )

    static_tf_single = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        condition=is_single,
    )

    # Joint state publisher GUI - DISABLED (using fake_trajectory_executor instead)
    # joint_state_publisher_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz',
        ),
        DeclareLaunchArgument(
            'arm_mode',
            default_value='dual',
            description='arm mode: dual or single',
        ),
        cartesian_path_diagnoser,
        ros2_control_node_dual,
        ros2_control_node_single,
        delayed_controller_spawners_dual,
        delayed_controller_spawners_single,
        robot_state_publisher_dual,
        robot_state_publisher_single,
        static_tf_dual,
        static_tf_single,
        delayed_move_group_dual,
        delayed_move_group_single,
        delayed_rviz_dual,
        delayed_rviz_single,
        # joint_state_publisher_gui,  # Disabled - fake_trajectory_executor publishes joint states
    ])
