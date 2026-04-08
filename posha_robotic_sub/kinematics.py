from dataclasses import dataclass
from typing import List

from posha_robotic_sub.workspace import Pose


@dataclass(frozen=True)
class JointSolution:
    joints: List[float]


class SimpleKinematics:
    def inverse_kinematics(self, pose: Pose) -> JointSolution:
        joint1 = pose.x
        joint2 = pose.y
        joint3 = pose.z
        joint4 = pose.roll
        joint5 = pose.pitch
        joint6 = pose.yaw

        return JointSolution([joint1, joint2, joint3, joint4, joint5, joint6])


if __name__ == "__main__":
    sample_pose = Pose(0.2, -0.1, 0.05)
    solver = SimpleKinematics()
    solution = solver.inverse_kinematics(sample_pose)
