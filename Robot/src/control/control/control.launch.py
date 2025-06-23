from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hardware", 
            default_value="true",
            description="Use hardware system"
        )
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("control"),
        "config",
        "ers6_controller.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    joint_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
    )

    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_spawner,
    #    robot_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
