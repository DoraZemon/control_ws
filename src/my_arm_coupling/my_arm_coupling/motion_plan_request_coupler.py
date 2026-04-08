#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest, JointConstraint


class MotionPlanRequestCoupler(Node):
    def __init__(self):
        super().__init__("motion_plan_request_coupler")

        self.declare_parameter("in_topic", "/motion_plan_request_raw")
        self.declare_parameter("out_topic", "/motion_plan_request")
        self.declare_parameter("joint2_name", "joint2")
        self.declare_parameter("joint3_name", "joint3")
        self.declare_parameter("joint4_name", "joint4")
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("default_tolerance", 1e-4)

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.j2 = self.get_parameter("joint2_name").value
        self.j3 = self.get_parameter("joint3_name").value
        self.j4 = self.get_parameter("joint4_name").value
        self.offset = float(self.get_parameter("offset").value)
        self.default_tol = float(self.get_parameter("default_tolerance").value)

        self.pub = self.create_publisher(MotionPlanRequest, self.out_topic, 10)
        self.sub = self.create_subscription(MotionPlanRequest, self.in_topic, self.cb, 10)

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

    def cb(self, msg: MotionPlanRequest):
        out = copy.deepcopy(msg)

        # Couple goal joint constraints (typical when using Goal State sliders)
        for gc in out.goal_constraints:
            self._couple_joint_constraints(gc.joint_constraints)

        # Couple path constraints joint constraints if present
        self._couple_joint_constraints(out.path_constraints.joint_constraints)

        # Couple explicit start_state if provided
        self._couple_robot_state_joint_state(out.start_state.joint_state)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = MotionPlanRequestCoupler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
