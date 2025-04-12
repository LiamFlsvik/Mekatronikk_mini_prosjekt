from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    device_arg = DeclareLaunchArgument("device", default_value="/dev/ttyACM0")
    baud_rate_arg = DeclareLaunchArgument("baud_rate", default_value="115200")
    simulation_arg = DeclareLaunchArgument("simulation", default_value="false")
    gui_arg = DeclareLaunchArgument("gui", default_value="false")

    device = LaunchConfiguration("device")
    baud_rate = LaunchConfiguration("baud_rate")
    simulation = LaunchConfiguration("simulation")
    gui = LaunchConfiguration("gui")

    # Robot Description
    urdf_file = os.path.join(
        get_package_share_directory("qube_bringup"), "urdf", "controlled_qube.urdf.xacro"
    )
    robot_description = Command([
        "xacro ", urdf_file,
        " device:=", device,
        " baud_rate:=", baud_rate,
        " simulation:=", simulation
    ])

    # Include RViz + robot_state_publisher
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
            "robot_description": robot_description
        }.items()
    )

    # Include the qube_driver launch (which handles ros2_control + controllers)
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("qube_driver"),
                "launch",
                "qube_driver.launch.py"
            ])
        ]),
        
    )

    return LaunchDescription([
        device_arg,
        baud_rate_arg,
        simulation_arg,
        gui_arg,
        view_qube_launch,
        qube_driver_launch,
    ])
