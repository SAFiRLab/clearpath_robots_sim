from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
    # Argument to load the right robot model using xacro
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='a200',
        choices=['a200', 'a300', 'dd100', 'dd150', 'do100', 'do150', 'generic', 'j100', 'r100', 'w200'],
        description='Robot model to use'
    )

    robot_model = LaunchConfiguration("robot_model")
    
    # Properly build path to xacro
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("clearpath_robots_sim"),
            "urdf",
            robot_model,
            [robot_model, ".urdf.xacro"]
        ])
    ])
    
    # ros2_control node
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([
                FindPackageShare("clearpath_robots_sim"),
                "config",
                robot_model,
                "control.yaml"
            ])
        ],
        output="screen"
    )

    
    return LaunchDescription([
        robot_model_arg,
        ros2_control,
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["platform_velocity_controller"]
        )
    ])
