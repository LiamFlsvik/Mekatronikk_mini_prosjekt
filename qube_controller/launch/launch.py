import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('qube_controller'),
        'config',
        'parameters.yaml'
    )

    declare_kp = DeclareLaunchArgument('kp', default_value='1.0', description='kp')
    declare_ki = DeclareLaunchArgument('ki', default_value='0.8', description='ki')
    declare_kd = DeclareLaunchArgument('kd', default_value='0.0000', description='kd')
    declare_reference = DeclareLaunchArgument('reference', default_value='0', description='reference')

    pid_controller_node = Node(
        package='qube_controller',
        executable='qube_controller_node',
        parameters=[config]
    )

    return LaunchDescription([
        declare_kp,
        declare_ki,
        declare_kd,
        declare_reference,
        pid_controller_node
    ])