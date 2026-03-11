import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        robot_state_publisher,
        static_tf,
        move_group_node,
        rviz_node,
        # joint_state_publisher_gui,  # Disabled - fake_trajectory_executor publishes joint states
    ])
