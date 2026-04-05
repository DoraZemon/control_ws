#!/usr/bin/env python3
import copy
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class DisplayTrajectoryCoupler(Node):
    def __init__(self):
        super().__init__("display_trajectory_coupler")
        self.declare_parameter("in_topic", "/display_planned_path")
        self.declare_parameter("out_topic", "/display_planned_path_coupled")
        self.declare_parameter("joint2_name", "joint2")
        self.declare_parameter("joint3_name", "joint3")
        self.declare_parameter("joint4_name", "joint4")
        self.declare_parameter("offset", 0.0)

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.j2 = self.get_parameter("joint2_name").value
        self.j3 = self.get_parameter("joint3_name").value
        self.j4 = self.get_parameter("joint4_name").value
        self.offset = float(self.get_parameter("offset").value)

        self.pub = self.create_publisher(DisplayTrajectory, self.out_topic, 10)
        self.sub = self.create_subscription(DisplayTrajectory, self.in_topic, self.cb, 10)

    @staticmethod
    def _insert_pad(arr, index, value=0.0):
        """Insert value at index; if arr is empty, keep empty (meaning 'not provided')."""
        if not arr:
            return []
        arr = list(arr)
        arr.insert(index, value)
        return arr

    def cb(self, msg: DisplayTrajectory):
        out = copy.deepcopy(msg)

        for traj in out.trajectory:
            jt = traj.joint_trajectory
            names = list(jt.joint_names)

            # must have joint2/joint3 to compute
            if self.j2 not in names or self.j3 not in names:
                continue

            # If joint4 missing, insert it right after joint3 in joint_names
            inserted = False
            if self.j4 not in names:
                j3_pos = names.index(self.j3)
                names.insert(j3_pos + 1, self.j4)
                inserted = True

            # recompute indices after potential insert
            j2i = names.index(self.j2)
            j3i = names.index(self.j3)
            j4i = names.index(self.j4)

            jt.joint_names = names

            new_points = []
            for p in jt.points:
                np = JointTrajectoryPoint()
                np.time_from_start = p.time_from_start

                pos = list(p.positions)

                # If we inserted a joint name, we must also insert into arrays at same index,
                # otherwise name-position mapping can break in consumers.
                if inserted:
                    # If the producer didn't provide positions (shouldn't happen), treat as zeros.
                    if len(pos) == len(names) - 1:
                        pos.insert(j4i, 0.0)
                    elif len(pos) < len(names) - 1:
                        # pad to old length then insert
                        pos.extend([0.0] * ((len(names) - 1) - len(pos)))
                        pos.insert(j4i, 0.0)
                    else:
                        # already longer/equal; just insert to keep alignment
                        pos.insert(j4i, 0.0)
                else:
                    # ensure length is enough to write j4i
                    if len(pos) < len(names):
                        pos.extend([0.0] * (len(names) - len(pos)))

                q2 = pos[j2i]
                q3 = pos[j3i]
                pos[j4i] = (-q3 + q2) + self.offset
                np.positions = pos

                # keep optional arrays aligned if they exist
                np.velocities = self._insert_pad(p.velocities, j4i, 0.0) if inserted else list(p.velocities) if p.velocities else []
                np.accelerations = self._insert_pad(p.accelerations, j4i, 0.0) if inserted else list(p.accelerations) if p.accelerations else []
                np.effort = self._insert_pad(p.effort, j4i, 0.0) if inserted else list(p.effort) if p.effort else []

                new_points.append(np)

            jt.points = new_points

        self.pub.publish(out)


def main():
    rclpy.init()
    node = DisplayTrajectoryCoupler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
