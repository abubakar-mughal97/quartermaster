import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # 1. Locate package paths
    pkg_description = get_package_share_directory("sorting_bot_description")
    pkg_logic = get_package_share_directory("sorting_bot_logic")

    # 2. Parse the URDF/Xacro file natively via Python
    xacro_file = os.path.join(pkg_description, "urdf", "sorting_bot.urdf.xacro")
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # 3. Include the official Gazebo ROS system launch file (Spawns a blank world)
    # ROS 2 Jazzy uses the modern Gazebo Harmonic simulator ecosystem
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items(),  # '-r' boots the simulator running immediately
    )

    # 4. Node: Spawns the parsed URDF model inside the running Gazebo environment
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_xml,
            "-name",
            "sorting_bot",
            "-z",
            "0.0",  # Place on the ground level
        ],
    )

    # 5. Node: Standard Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_xml,
                "use_sim_time": True,  # Ensures node synchronizes clock cycles with the physics simulation
            }
        ],
    )

    # 6. Node: Spawners for the controllers defined in our YAML file
    # These call the Controller Manager daemon to activate our control loops
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # 7. Use Event Handlers to enforce execution order
    # The controllers will crash if they try to launch BEFORE the robot finishes spawning in Gazebo.
    delay_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node, on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Configure the ROS <-> Gazebo Bridge
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # This line bridges the clock from Gazebo Sim (gz) to ROS 2 (ros)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # ... keep your other existing bridge arguments here (like /robot_description, etc.) ...
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo_launch,
            spawn_robot_node,
            robot_state_publisher_node,
            delay_broadcaster,
            delay_arm_controller,
            bridge_node
        ]
    )
