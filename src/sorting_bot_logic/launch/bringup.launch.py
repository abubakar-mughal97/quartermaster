import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_logic = get_package_share_directory("sorting_bot_logic")
    pkg_moveit_config = get_package_share_directory("sorting_bot_moveit_config")

    # 1. Include your existing Week 3 Gazebo Simulation world
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_logic, "launch", "simulation.launch.py")
        )
    )

    # 2. Programmatically parse the configuration data from the pristine assistant package
    moveit_config = MoveItConfigsBuilder(
        "sorting_robot", package_name="sorting_bot_moveit_config"
    ).to_moveit_configs()

    # 3. Explicitly configure the move_group node with forced simulation clock sync
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},  # Enforces clock synchronization with Gazebo
        ],
    )

    # 4. Explicitly configure the rviz2 node using the assistant's visual profile layout
    rviz_config_file = os.path.join(pkg_moveit_config, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True
            },  # Ensures interactive markers sync to the simulation timeline
        ],
    )

    return LaunchDescription([simulation_launch, move_group_node, rviz_node])
