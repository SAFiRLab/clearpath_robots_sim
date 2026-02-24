from launch import LaunchDescription 
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
    # Robot model argument
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='a200',
        choices=['a200', 'a300', 'dd100', 'dd150', 'do100', 'do150', 'generic', 'j100', 'r100', 'w200'],
        description='Robot model to use'
    )

    # use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    robot_model = LaunchConfiguration("robot_model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Build robot_description from xacro
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

    # robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }],
        output="screen"
    )

    # ros2_control node
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
            "use_sim_time": True},
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
        use_sim_time_arg,
        robot_state_publisher,
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