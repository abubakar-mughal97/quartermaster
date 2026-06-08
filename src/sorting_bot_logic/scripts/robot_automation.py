#!/usr/bin/env python3
import threading
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion


class RobotAutomationInterface(Node):

    def __init__(self):
        super().__init__("robot_automation_interface")

        # Initialize communication channel with MoveGroup
        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Connecting to MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup online. Cartesian navigation active.")

        # FIX 3: Background thread to prevent locking the ROS 2 executor.
        # This allows __init__ to finish so rclpy.spin() can process network data.
        self.automation_thread = threading.Thread(target=self.run_automation_loop)
        self.automation_thread.daemon = True
        self.automation_thread.start()

    def send_cartesian_goal(self, x, y, z, qx, qy, qz, qw, pose_name=""):
        """Compiles a 3D target posture request and submits it to the IK engine."""

        # PREVENT EIGEN MATH CRASHES: Normalize the quaternion to exactly 1.0
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm == 0:
            self.get_logger().error("Invalid zero-length quaternion provided!")
            return False
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        goal_msg = MoveGroup.Goal()

        # 1. Configure standard global pipeline parameters
        request = MotionPlanRequest()
        request.group_name = "arm"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.4
        request.max_acceleration_scaling_factor = 0.3

        workspace = WorkspaceParameters()
        workspace.header.frame_id = "world"
        workspace.min_corner.x = -2.0
        workspace.min_corner.y = -2.0
        workspace.min_corner.z = -0.5
        workspace.max_corner.x = 2.0
        workspace.max_corner.y = 2.0
        workspace.max_corner.z = 2.5
        request.workspace_parameters = workspace

        # 2. Build the Position Constraint
        constraints = Constraints()
        constraints.name = "position_and_orientation_goal"

        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "link_6"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        # FIX 1: Widen to 5cm (0.05m) for KDL IK mathematical convergence
        primitive.dimensions = [0.05]

        primitive_pose = Pose()
        primitive_pose.position.x = x
        primitive_pose.position.y = y
        primitive_pose.position.z = z
        primitive_pose.orientation.w = 1.0

        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(primitive_pose)
        pc.weight = 1.0

        # 3. Build the Orientation Constraint
        oc = OrientationConstraint()
        oc.header.frame_id = "world"
        oc.link_name = "link_6"
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        # FIX 1: Loosen angular constraint to ~11 degrees (0.20 rad) to prevent IK timeout
        oc.absolute_x_axis_tolerance = 0.20
        oc.absolute_y_axis_tolerance = 0.20
        oc.absolute_z_axis_tolerance = 0.20
        oc.weight = 1.0

        # 4. Bind constraints to the master request frame
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        request.goal_constraints.append(constraints)

        # 5. Pack goal and dispatch asynchronously
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False

        self.get_logger().info(
            f"Requesting Cartesian path to {pose_name}: [X:{x}, Y:{y}, Z:{z}]"
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)

        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.05)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Cartesian position target rejected by pipeline!")
            return False

        get_result_future = goal_handle.get_result_async()

        while rclpy.ok() and not get_result_future.done():
            time.sleep(0.05)

        result = get_result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info(f"Successfully executed path to: {pose_name}")
            return True
        else:
            self.get_logger().error(
                f"IK or path generation failed. MoveIt Error Code: {result.error_code.val}"
            )
            return False

    def run_automation_loop(self):
        """Sequences spatial coordinates to mock a picking operation."""

        # Give the system 1 second to register topics before spamming goals
        time.sleep(1.0)

        # Orientation Quaternions: Tool facing straight down
        downward_orientation = {"qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}

        try:
            while rclpy.ok():
                # Target Pose A: High Standby Hover Pose above sorting table
                # FIX 2: Z coordinates shifted up by 1.0m to account for the global 'world' frame
                self.send_cartesian_goal(
                    x=0.3,
                    y=0.0,
                    z=1.5,
                    **downward_orientation,
                    pose_name="HOVER_SCAN_ZONE",
                )
                time.sleep(3.0)

                # Target Pose B: Low Pick Target
                self.send_cartesian_goal(
                    x=0.4,
                    y=0.1,
                    z=1.2,
                    **downward_orientation,
                    pose_name="TABLE_PICK_ZONE",
                )
                time.sleep(3.0)

                # Target Pose C: Sorting Drop Bin
                self.send_cartesian_goal(
                    x=0.1,
                    y=-0.3,
                    z=1.4,
                    **downward_orientation,
                    pose_name="SORTING_BIN_1",
                )
                time.sleep(3.0)

        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotAutomationInterface()

    try:
        # Crucial: This explicitly spins the node to process incoming action results
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Autonomous sequence closed down by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
