#!/usr/bin/env python3
import asyncio
import copy

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import JointConstraint


class MoveGroupActionCoupler(Node):
    def __init__(self):
        super().__init__("move_group_action_coupler")

        self.declare_parameter("action_in", "/move_action")
        self.declare_parameter("action_out", "/move_action_raw")
        self.declare_parameter("joint2_name", "joint2")
        self.declare_parameter("joint3_name", "joint3")
        self.declare_parameter("joint4_name", "joint4")
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("default_tolerance", 1e-4)

        self.action_in = self.get_parameter("action_in").value
        self.action_out = self.get_parameter("action_out").value
        self.j2 = self.get_parameter("joint2_name").value
        self.j3 = self.get_parameter("joint3_name").value
        self.j4 = self.get_parameter("joint4_name").value
        self.offset = float(self.get_parameter("offset").value)
        self.default_tol = float(self.get_parameter("default_tolerance").value)

        self.client = ActionClient(self, MoveGroup, self.action_out)
        self.server = ActionServer(
            self,
            MoveGroup,
            self.action_in,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            execute_callback=self.execute_cb,
        )

    def destroy_node(self):
        self.server.destroy()
        self.client.destroy()
        super().destroy_node()

    def goal_cb(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _compute_q4(self, q2: float, q3: float) -> float:
        return q2 - q3 + self.offset

    def _couple_joint_constraints(self, joint_constraints):
        idx = {jc.joint_name: i for i, jc in enumerate(joint_constraints)}
        if self.j2 not in idx or self.j3 not in idx:
            return

        j2c = joint_constraints[idx[self.j2]]
        j3c = joint_constraints[idx[self.j3]]
        q4 = self._compute_q4(j2c.position, j3c.position)

        tol = max(
            self.default_tol,
            float(j2c.tolerance_above),
            float(j2c.tolerance_below),
            float(j3c.tolerance_above),
            float(j3c.tolerance_below),
        )

        if self.j4 in idx:
            j4c = joint_constraints[idx[self.j4]]
            j4c.position = q4
            j4c.tolerance_above = tol
            j4c.tolerance_below = tol
            j4c.weight = 1.0
        else:
            j4c = JointConstraint()
            j4c.joint_name = self.j4
            j4c.position = q4
            j4c.tolerance_above = tol
            j4c.tolerance_below = tol
            j4c.weight = 1.0
            joint_constraints.append(j4c)

    def _couple_robot_state_joint_state(self, joint_state):
        idx = {n: i for i, n in enumerate(joint_state.name)}
        if self.j2 not in idx or self.j3 not in idx:
            return
        if len(joint_state.position) != len(joint_state.name):
            return

        q2 = joint_state.position[idx[self.j2]]
        q3 = joint_state.position[idx[self.j3]]
        q4 = self._compute_q4(q2, q3)

        if self.j4 in idx:
            joint_state.position[idx[self.j4]] = q4
        else:
            joint_state.name.append(self.j4)
            joint_state.position.append(q4)

    def _couple_goal(self, goal_msg: MoveGroup.Goal):
        req = goal_msg.request

        for gc in req.goal_constraints:
            self._couple_joint_constraints(gc.joint_constraints)

        self._couple_joint_constraints(req.path_constraints.joint_constraints)
        self._couple_robot_state_joint_state(req.start_state.joint_state)

    async def execute_cb(self, goal_handle):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/move_action_raw not available")
            goal_handle.abort()
            return MoveGroup.Result()

        out_goal = copy.deepcopy(goal_handle.request)
        self._couple_goal(out_goal)

        def feedback_cb(feedback_msg):
            goal_handle.publish_feedback(feedback_msg.feedback)

        send_future = self.client.send_goal_async(out_goal, feedback_callback=feedback_cb)
        client_goal_handle = await send_future

        if not client_goal_handle.accepted:
            self.get_logger().error("Underlying /move_action_raw rejected goal")
            goal_handle.abort()
            return MoveGroup.Result()

        result_future = client_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await client_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                return MoveGroup.Result()
            await asyncio.sleep(0.02)

        wrapped = result_future.result()

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        return wrapped.result


def main():
    rclpy.init()
    node = MoveGroupActionCoupler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
