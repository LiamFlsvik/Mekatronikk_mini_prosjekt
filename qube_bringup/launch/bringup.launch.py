from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    device_arg = DeclareLaunchArgument("device", default_value="/dev/ttyACM0")
    baud_rate_arg = DeclareLaunchArgument("baud_rate", default_value="115200")
    simulation_arg = DeclareLaunchArgument("simulation", default_value="false")

    
    device = LaunchConfiguration("device")
    baud_rate = LaunchConfiguration("baud_rate")
    simulation = LaunchConfiguration("simulation")


    qube_bringup_dir = get_package_share_directory("qube_bringup")
    urdf_file = os.path.join(qube_bringup_dir, "urdf", "controlled_qube.urdf.xacro")

    robot_description = Command([
        "xacro ", urdf_file,
        " device:=", device,
        " baud_rate:=", baud_rate,
        " simulation:=", simulation
    ])

    # Noder
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters= [{"robot_description": robot_description}]
    )


    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    controller_broadcaster_joint = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    velocity_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

        
    return LaunchDescription([
        device_arg,
        baud_rate_arg,
        simulation_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        ros2_control_node,  
        controller_broadcaster_joint,   
        velocity_controller_node,
        rviz_node,
    ])