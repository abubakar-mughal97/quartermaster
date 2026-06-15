import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("quartermaster_gazebo")
    pkg_moveit_config = get_package_share_directory("quartermaster_moveit_config")

    # Gazebo + robot + controller_manager (in the gz plugin) + rsp + spawners.
    # This provides everything MoveIt needs to talk to.
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "simulation.launch.py")
        )
    )

    # Build MoveIt's config. Its robot_description uses the mock-hardware wrapper,
    # but move_group only uses it for planning geometry/kinematics — the hardware
    # block is irrelevant to the planner. This object also carries moveit_controllers.yaml.
    moveit_config = MoveItConfigsBuilder(
        "quartermaster", package_name="quartermaster_moveit_config"
    ).to_moveit_configs()

    # The planner. use_sim_time=True is CRITICAL here (see below).
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    rviz_config_file = os.path.join(pkg_moveit_config, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    return LaunchDescription([simulation_launch, move_group_node, rviz_node])
