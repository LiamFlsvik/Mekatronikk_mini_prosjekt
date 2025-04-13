import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="True",
        description="Enable joint_state_publisher_gui"
    )

    pkg_path = get_package_share_directory("qube_description")
    xacro_file = os.path.join(pkg_path, "urdf", "qube.urdf.xacro")
    
    # Process the xacro
    robot_description_config = xacro.process_file(xacro_file).toxml()
    params = {"robot_description": robot_description_config}

    return launch.LaunchDescription([
        gui_arg,

        launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[params]
        ),

        launch_ros.actions.Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
            parameters=[params]
        ),

        launch_ros.actions.Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
            parameters=[params]
        ),

        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(pkg_path, "config", "qube.rviz")],
            output="screen",
        ),
    ])
