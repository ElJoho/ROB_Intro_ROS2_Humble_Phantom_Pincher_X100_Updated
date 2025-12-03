from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # --- Paths ----------------------------------------------------------------
    # URDF (xacro) path
    urdf_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_description"),
        "urdf",
        "phantomx_pincher.urdf.xacro",
    ])

    # Controller config (position controllers)
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_moveit_config"),
        "config",
        "controllers_position.yaml",
    ])

    # MoveIt move_group launch file
    move_group_launch_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_moveit_config"),
        "launch",
        "move_group.launch.py",
    ])

    # --- Robot description (shared between nodes) -----------------------------
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str,
    )

    # --- Robot State Publisher (ours) ----------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
        }],
    )

    # --- ros2_control controller_manager (ours) -------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            # Give controller_manager the same robot_description
            {"robot_description": robot_description},
            # Load controllers from the position-based config
            controllers_yaml,
        ],
    )

    # --- Controller spawners --------------------------------------------------
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    # Arm joint trajectory controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    # Gripper trajectory controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_trajectory_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    # --- MoveIt move_group (without its own ros2_control_node) ---------------
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([move_group_launch_path]),
        launch_arguments={
            # IMPORTANT: empty string -> bool('') == False in move_group.launch.py
            # so MoveIt will NOT start its own ros2_control_node
            "ros2_control": "",
            # We manage controllers ourselves in this bringup
            "manage_controllers": "false",
            # Keep MoveIt's RViz window
            "enable_rviz": "true",
            # If your move_group.launch.py has this argument and you want Servo:
            # "enable_servo": "true",
            "enable_servo": "false",
        }.items(),
    )

    # NOTE: we intentionally do NOT start our own RViz here,
    # so only MoveIt's RViz window will appear.

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_launch,
    ])
