import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    
    # Use absolute path instead of package discovery
    pkg_share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Load URDF
    urdf_file = os.path.join(pkg_share, 'config', 'L_JEARM.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Load SRDF
    srdf_file = os.path.join(pkg_share, 'config', 'L_JEARM.srdf.xacro')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()
    
    # Load kinematics
    kinematics_yaml = load_yaml(os.path.join(pkg_share, 'config', 'kinematics.yaml'))
    
    # Load joint limits
    joint_limits_yaml = load_yaml(os.path.join(pkg_share, 'config', 'joint_limits.yaml'))
    
    # Load planning pipelines (single source for OMPL + pipeline params)
    planning_pipelines_yaml = load_yaml(os.path.join(pkg_share, 'config', 'planning_pipelines.yaml'))
    
    # Load controllers
    controllers_yaml = load_yaml(os.path.join(pkg_share, 'config', 'moveit_controllers.yaml'))
    ros2_controllers_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    
    # Combine parameters
    moveit_config_dict = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
    }
    
    # 加载规划配置（包含 pipeline 和 OMPL 细节）
    if planning_pipelines_yaml:
        moveit_config_dict.update(planning_pipelines_yaml)
    
    # 加载控制器配置
    if controllers_yaml:
        moveit_config_dict.update(controllers_yaml)
    
    # 强制指定使用OMPL规划器（确保不被覆盖）
    moveit_config_dict['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    
    # 确保轨迹执行超时参数被正确加载
    # 允许执行时间为轨迹声明时间的100倍
    moveit_config_dict['trajectory_execution'] = {
        'manage_controllers': True,
        'allowed_execution_timeout_scaling': 100.0,
        'execution_duration_monitoring': False  # Disable strict monitoring
    }
    
    # Start move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {'robot_description': robot_description},
            ros2_controllers_file,
        ],
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

    delayed_controller_spawners = TimerAction(
        period=6.0,
        actions=[joint_state_broadcaster_spawner, jearm_controller_spawner],
    )

    # RViz node
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'moveit.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config_dict],
    )

    # 延迟启动 RViz，确保 /joint_states 已在发布，
    # 这样 RViz 在启动时能读取到实时的机械臂位姿，而不是默认值
    delayed_rviz = TimerAction(
        period=8.0,  # 等待 8 秒，确保 joint_state_broadcaster 已启动并发布状态
        actions=[rviz_node],
        condition=IfCondition(start_rviz),
    )

    # 延迟启动 move_group，确保硬件已初始化
    delayed_move_group = TimerAction(
        period=7.0,  # 等待 7 秒，在 controller spawners 之后但在 RViz 之前
        actions=[move_group_node],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description}],
    )

    # Static TF for base link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
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
        ros2_control_node,
        delayed_controller_spawners,
        robot_state_publisher,
        static_tf,
        delayed_move_group,
        delayed_rviz,
        # joint_state_publisher_gui,  # Disabled - fake_trajectory_executor publishes joint states
    ])
