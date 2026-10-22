import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive


def normalize_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm == 0.0:
        return False
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return True


class PoseToMoveitBridge(Node):
    def __init__(self):
        super().__init__("pose_to_moveit_bridge")

        self.declare_parameter("action_name", "/move_action")
        self.declare_parameter("pose_topic", "/my_topic/simulink/target_pose")
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("link_name", "link_6")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_enabled", False)
        self.declare_parameter("orientation_tolerance", 0.2)
        self.declare_parameter("max_velocity_scaling", 0.3)
        self.declare_parameter("max_acceleration_scaling", 0.3)
        self.declare_parameter("allowed_planning_time", 5.0)

        self.action_name = self.get_parameter("action_name").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.group_name = self.get_parameter("group_name").value
        self.link_name = self.get_parameter("link_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_enabled = bool(self.get_parameter("orientation_enabled").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self.max_velocity_scaling = float(self.get_parameter("max_velocity_scaling").value)
        self.max_acceleration_scaling = float(self.get_parameter("max_acceleration_scaling").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)

        self._busy = False
        self._action_client = ActionClient(self, MoveGroup, self.action_name)

        self.get_logger().info(f"Waiting for MoveGroup action server: {self.action_name}")
        while rclpy.ok() and not self._action_client.wait_for_server(timeout_sec=1.0):
            pass
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown before MoveGroup action server became available")
        self.get_logger().info("MoveGroup action server connected")

        self.create_subscription(Pose, self.pose_topic, self.on_pose, 10)
        self.get_logger().info(f"Subscribed to pose topic: {self.pose_topic}")

    def on_pose(self, pose_msg: Pose):
        if self._busy:
            return

        if not normalize_quaternion(pose_msg.orientation):
            if self.orientation_enabled:
                self.get_logger().error("Received invalid quaternion (zero norm)")
                return
            pose_msg.orientation.w = 1.0

        self._busy = True

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.frame_id
        pos_constraint.link_name = self.link_name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            2.0 * self.position_tolerance,
            2.0 * self.position_tolerance,
            2.0 * self.position_tolerance,
        ]

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose_msg)
        pos_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        if self.orientation_enabled:
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = self.frame_id
            ori_constraint.link_name = self.link_name
            ori_constraint.orientation = pose_msg.orientation
            ori_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
            ori_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
            ori_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
            ori_constraint.weight = 1.0
            constraints.orientation_constraints.append(ori_constraint)

        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = self.allowed_planning_time
        request.max_velocity_scaling_factor = self.max_velocity_scaling
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling
        request.goal_constraints.append(constraints)

        request.start_state = RobotState()
        request.start_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        if future.exception() is not None:
            self.get_logger().error(f"MoveGroup send_goal failed: {future.exception()}")
            self._busy = False
            return

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            self._busy = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        if future.exception() is not None:
            self.get_logger().error(f"MoveGroup result failed: {future.exception()}")
        else:
            self.get_logger().info(f"MoveIt result code: {future.result().result.error_code.val}")
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PoseToMoveitBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
