#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint


class RobotAutomationInterface(Node):
    def __init__(self):
        super().__init__("robot_automation_interface")

        self._action_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server ..")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup server online. Automation active.")

        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        self.run_automation_loop()

    def send_joint_goal(self, target_joints, pose_name=""):
        goal_msg = MoveGroup.Goal()

        request = MotionPlanRequest()
        request.group_name = "arm"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request._max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        for name, joint_pos in zip(self.joint_names, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joint_pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)

        goal_msg.request = request
        goal_msg.planning_options.plan_only = (
            False  # Set to False to enforce automatic execution
        )

        self.get_logger().info(f"Dispatching command trajectory to target: {pose_name}")

        # 4. Asynchronously send the goal request
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                "Trajectory goal rejected by MoveIt planning pipeline!"
            )
            return False

        # 5. Await physical trajectory execution feedback from Gazebo
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        if result.error_code.val == 1:  # SUCCESS = 1
            self.get_logger().info(f"Successfully reached target: {pose_name}")
            return True
        else:
            self.get_logger().error(
                f"Trajectory execution failed with error code: {result.error_code.val}"
            )
            return False

    def run_automation_loop(self):
        """Alternates the arm between home and ready postures."""
        # Joint targets mapped in radians matching the Setup Assistant specifications
        home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ready_pose = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]

        try:
            while rclpy.ok():
                # Step A: Move to Ready postura
                success = self.send_joint_goal(ready_pose, "READY_POSTURE")
                if success:
                    self.get_logger().info(
                        "Simulating object scanning window... Idle for 4 seconds."
                    )
                    import time

                    time.sleep(4.0)

                # Step B: Revert back to Home positions
                self.send_joint_goal(home_pose, "HOME_POSTURE")
                self.get_logger().info(
                    "Cycle complete. Idle for 4 seconds before looping."
                )
                time.sleep(4.0)

        except KeyboardInterrupt:
            self.get_logger().info("Automation loop stopped by operator.")


def main(args=None):
    rclpy.init(args=args)
    node = RobotAutomationInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
