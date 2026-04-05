#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateFilter(Node):
    def __init__(self):
        super().__init__("joint_state_filter")
        self.declare_parameter("in_topic", "/joint_states")
        self.declare_parameter("out_topic", "/joint_states_moveit")
        self.declare_parameter("allow", ["joint1","joint2","joint3","joint5","joint6","joint7"])

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.allow = set(self.get_parameter("allow").value)

        self.pub = self.create_publisher(JointState, self.out_topic, 10)
        self.sub = self.create_subscription(JointState, self.in_topic, self.cb, 10)

    def cb(self, msg: JointState):
        out = JointState()
        out.header = msg.header

        idx = {n:i for i,n in enumerate(msg.name)}
        names = [n for n in msg.name if n in self.allow]

        out.name = names
        out.position = [msg.position[idx[n]] for n in names] if len(msg.position)==len(msg.name) else []
        out.velocity = [msg.velocity[idx[n]] for n in names] if len(msg.velocity)==len(msg.name) else []
        out.effort = [msg.effort[idx[n]] for n in names] if len(msg.effort)==len(msg.name) else []

        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
