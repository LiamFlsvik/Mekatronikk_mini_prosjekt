
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
import xacro


def generate_launch_description():
    robot_description = LaunchConfiguration("robot_description")
    params = {"robot_description": robot_description}
    pkg_path = get_package_share_directory('qube_description')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params] 
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[params],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'config', 'qube.rviz')],
        output='screen',
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='GUI to change angle'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
