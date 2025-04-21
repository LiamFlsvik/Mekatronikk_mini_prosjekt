from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    declare_device = DeclareLaunchArgument("device", default_value="/dev/ttyACM0")
    declare_baud = DeclareLaunchArgument("baud_rate", default_value="115200")
    declare_sim = DeclareLaunchArgument("simulation", default_value="false")
    declare_gui = DeclareLaunchArgument("gui", default_value="false")

    # Substitutions
    device = LaunchConfiguration("device")
    baud_rate = LaunchConfiguration("baud_rate")
    simulation = LaunchConfiguration("simulation")
    gui = LaunchConfiguration("gui")

    # Generate robot_description from xacro
    urdf_file = PathJoinSubstitution([
        FindPackageShare("qube_bringup"),
        "urdf",
        "controlled_qube.urdf.xacro"
    ])
    
    robot_description = Command([
        "xacro ", urdf_file,
        " device:=", device,
        " baud_rate:=", baud_rate,
        " simulation:=", simulation
    ])

    robot_description_param = {"robot_description": robot_description}

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
        output="screen"
    )

    # Joint State Publisher (standard and GUI)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(gui),
        parameters=[robot_description_param]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
        parameters=[robot_description_param]
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("qube_description"),
            "config",
            "qube.rviz"
        ])],
        output="screen"
    )

    # Include qube_driver.launch.py
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("qube_driver"),
            "launch",
            "qube_driver.launch.py"
        ]))
    )
    
    return LaunchDescription([
        declare_device,
        declare_baud,
        declare_sim,
        declare_gui,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz,
        qube_driver_launch
    ])
