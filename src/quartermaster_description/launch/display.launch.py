import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro  # Import the native xacro parsing library


def generate_launch_description():
    # 1. Locate the installed package directory
    pkg_description = get_package_share_directory("quartermaster_description")

    # 2. Define the path to our main xacro file
    xacro_file = os.path.join(pkg_description, "urdf", "quartermaster.urdf.xacro")

    # 3. Parse the xacro file natively via Python (bypasses fragile shell pipes)
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # 4. Configure the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_xml}
        ],  # Pass the raw XML string directly
    )

    # 5. Configure the joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # 6. Configure the RViz2 node
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen")

    return LaunchDescription(
        [robot_state_publisher_node, joint_state_publisher_gui_node, rviz_node]
    )
