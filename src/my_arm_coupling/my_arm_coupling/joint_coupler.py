#!/usr/bin/env python3
from typing import Dict, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointCoupler(Node):
    def __init__(self):
        super().__init__('joint_coupler')

        self.declare_parameter('in_topic', '/joint_states_raw')
        self.declare_parameter('out_topic', '/joint_states')
        self.declare_parameter('joint2_name', 'joint2')
        self.declare_parameter('joint3_name', 'joint3')
        self.declare_parameter('joint4_name', 'joint4')
        self.declare_parameter('offset', 0.0)  # 若有零位偏置，在此加

        self.in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        self.out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.j2 = self.get_parameter('joint2_name').get_parameter_value().string_value
        self.j3 = self.get_parameter('joint3_name').get_parameter_value().string_value
        self.j4 = self.get_parameter('joint4_name').get_parameter_value().string_value
        self.offset = self.get_parameter('offset').get_parameter_value().double_value

        self.pub = self.create_publisher(JointState, self.out_topic, 10)
        self.sub = self.create_subscription(JointState, self.in_topic, self.cb, 10)

    def cb(self, msg: JointState):
        name_to_index: Dict[str, int] = {n: i for i, n in enumerate(msg.name)}

        if self.j2 not in name_to_index or self.j3 not in name_to_index:
            self.get_logger().warn(f"Input JointState missing {self.j2} or {self.j3}")
            return

        q2 = msg.position[name_to_index[self.j2]]
        q3 = msg.position[name_to_index[self.j3]]

        q4 = (-q3 + q2) + self.offset

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()

        # 复制原来的关节（保持顺序）
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity) if len(msg.velocity) == len(msg.name) else []
        out.effort = list(msg.effort) if len(msg.effort) == len(msg.name) else []

        # 如果 joint4 已存在，就覆盖；否则追加
        if self.j4 in name_to_index:
            idx = name_to_index[self.j4]
            out.position[idx] = q4
        else:
            out.name.append(self.j4)
            out.position.append(q4)
            # velocity/effort 这里保持空或不追加，robot_state_publisher 也能用

        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointCoupler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
