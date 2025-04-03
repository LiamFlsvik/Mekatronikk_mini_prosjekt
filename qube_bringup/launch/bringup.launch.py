from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    device_arg = DeclareLaunchArgument("device", default_value="/dev/ttyACM0")
    baud_rate_arg = DeclareLaunchArgument("baud_rate", default_value="115200")
    simulation_arg = DeclareLaunchArgument("simulation", default_value="false")
    gui_arg = DeclareLaunchArgument("gui", default_value="false")  

    device = LaunchConfiguration("device")
    baud_rate = LaunchConfiguration("baud_rate")
    simulation = LaunchConfiguration("simulation")
    gui = LaunchConfiguration("gui")
    
    qube_bringup_dir = get_package_share_directory("qube_bringup")
    urdf_file = os.path.join(qube_bringup_dir, "urdf", "controlled_qube.urdf.xacro")
    controllers_yaml = PathJoinSubstitution([FindPackageShare("qube_driver"), "config", "joint_controllers.yaml"])
    
    robot_description = Command([
        "xacro ", urdf_file,    
        " device:=", device,
        " baud_rate:=", baud_rate,
        " simulation:=", simulation
    ])

    
    view_qube_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("qube_description"),
                "launch",
                "view_qube.launch.py"
            ])
        ]),
        launch_arguments={
            "gui": gui,
            "robot_description": robot_description}.items()
    )

    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            ParameterFile(controllers_yaml, allow_substs=True)
        ],
        output="screen",
        condition=UnlessCondition(simulation)
    )

    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    
    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    simulation_node = Node(
        package="qube_controller_node",
        executable="qube_simulator_node",
        output="screen",
        condition = IfCondition(simulation)
    )

    
    return LaunchDescription([
        device_arg,
        baud_rate_arg,
        simulation_arg,
        gui_arg,
        view_qube_launch,
        ros2_control_node,
        joint_state_broadcaster,
        velocity_controller,
        simulation_node
    ])