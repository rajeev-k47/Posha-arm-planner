from dataclasses import dataclass
from math import acos, atan2, cos, sin, sqrt
from typing import List

import numpy as np
from scipy.optimize import least_squares

from posha_robotic_sub.workspace import Pose


@dataclass(frozen=True)
class JointSolution:
    joints: List[float]


class SimpleKinematics:
    def __init__(self):
        self.base_height = 0.42
        self.link1 = 0.22
        self.link2 = 0.20
        self.wrist_offset = 0.08

    def inverse_kinematics(self, pose: Pose) -> JointSolution:

        joint1 = atan2(pose.y, pose.x)

        radial_distance = sqrt(pose.x ** 2 + pose.y ** 2)
        z_offset = pose.z - self.base_height

        effective_radius = max(0.001, radial_distance - self.wrist_offset)
        reach = sqrt(effective_radius ** 2 + z_offset ** 2)

        max_reach = self.link1 + self.link2 - 1e-6
        min_reach = abs(self.link1 - self.link2) + 1e-6
        reach = min(max(reach, min_reach), max_reach)

        cos_elbow = (
            (reach ** 2 - self.link1 ** 2 - self.link2 ** 2)
            / (2.0 * self.link1 * self.link2)
        )
        cos_elbow = min(1.0, max(-1.0, cos_elbow))
        joint3 = acos(cos_elbow)

        shoulder_to_target = atan2(z_offset, effective_radius)
        elbow_helper = atan2(
            self.link2 * sin(joint3),
            self.link1 + self.link2 * cos(joint3),
        )
        joint2 = shoulder_to_target - elbow_helper

        joint4 = pose.roll
        joint5 = pose.pitch
        joint6 = pose.yaw

        return JointSolution([joint1, joint2, joint3, joint4, joint5, joint6])

    def forward_kinematics(self, joints: List[float]) -> Pose:
        joint1, joint2, joint3, joint4, joint5, joint6 = joints

        planar_reach = (
            self.link1 * cos(joint2)
            + self.link2 * cos(joint2 + joint3)
            + self.wrist_offset
        )
        x = planar_reach * cos(joint1)
        y = planar_reach * sin(joint1)
        z = (
            self.base_height
            + self.link1 * sin(joint2)
            + self.link2 * sin(joint2 + joint3)
        )
        return Pose(x, y, z, joint4, joint5, joint6)


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = cos(angle)
    s = sin(angle)
    v = 1.0 - c
    return np.array(
        [
            [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
        ]
    )


@dataclass(frozen=True)
class JointSpec:
    name: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


PIPER_JOINTS = [
    JointSpec("joint1", np.array([0.0, 0.0, 0.123]), np.array(
        [0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), -2.618, 2.618),
    JointSpec("joint2", np.array([0.0, 0.0, 0.0]), np.array(
        [1.5708, -0.1359, -3.1416]), np.array([0.0, 0.0, 1.0]), 0.0, 3.14),
    JointSpec("joint3", np.array([0.28503, 0.0, 0.0]), np.array(
        [0.0, 0.0, -1.7939]), np.array([0.0, 0.0, 1.0]), -2.967, 0.0),
    JointSpec("joint4", np.array([-0.021984, -0.25075, 0.0]), np.array(
        [1.5708, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), -1.745, 1.745),
    JointSpec("joint5", np.array([0.0, 0.0, 0.0]), np.array(
        [-1.5708, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), -1.22, 1.22),
    JointSpec("joint6", np.array([8.8259e-05, -0.091, 0.0]), np.array(
        [1.5708, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), -2.0944, 2.0944),
]


class PiperKinematics:
    def __init__(self) -> None:
        self.gripper_offset = np.array([0.0, 0.0, 0.1358])
        self.lower_bounds = np.array(
            [joint.lower for joint in PIPER_JOINTS], dtype=float)
        self.upper_bounds = np.array(
            [joint.upper for joint in PIPER_JOINTS], dtype=float)

    def forward_kinematics(self, joints: List[float]) -> np.ndarray:
        transform = np.eye(4)
        for joint, angle in zip(PIPER_JOINTS, joints):
            origin = np.eye(4)
            origin[:3, :3] = rotation_matrix_from_rpy(*joint.rpy)
            origin[:3, 3] = joint.xyz
            transform = transform @ origin

            rotation = np.eye(4)
            rotation[:3, :3] = rotation_about_axis(joint.axis, angle)
            transform = transform @ rotation

        tool = transform.copy()
        tool[:3, 3] = tool[:3, 3] + tool[:3, :3] @ self.gripper_offset
        return tool

    def tool_position(self, joints: List[float]) -> np.ndarray:
        return self.forward_kinematics(joints)[:3, 3]

    def inverse_kinematics(self, pose: Pose, seed: List[float] | None = None) -> JointSolution:
        target = np.array([pose.x, pose.y, pose.z], dtype=float)
        if seed is None:
            seed = [0.0, 1.0, -1.4, 0.0, 0.4, 0.0]
        seed_array = np.clip(np.array(seed, dtype=float),
                             self.lower_bounds, self.upper_bounds)

        def residual(joints: np.ndarray) -> np.ndarray:
            position_error = 10.0 * \
                (self.tool_position(joints.tolist()) - target)
            seed_error = 0.02 * (joints - seed_array)
            return np.concatenate([position_error, seed_error])

        candidate_seeds = [
            seed_array,
            np.array([0.0, 1.0, -1.4, 0.0, 0.4, 0.0], dtype=float),
            np.array([atan2(pose.y, pose.x), 1.2, -
                     1.6, 0.0, 0.4, 0.0], dtype=float),
            np.array([atan2(pose.y, pose.x), 0.8, -
                     1.0, 0.0, 0.2, 0.0], dtype=float),
        ]

        best_joints = seed_array
        best_error = float("inf")
        for candidate_seed in candidate_seeds:
            result = least_squares(
                residual,
                x0=np.clip(candidate_seed, self.lower_bounds,
                           self.upper_bounds),
                bounds=(self.lower_bounds, self.upper_bounds),
                xtol=1e-6,
                ftol=1e-6,
                gtol=1e-6,
                max_nfev=400,
            )
            error = float(np.linalg.norm(
                self.tool_position(result.x.tolist()) - target))
            if error < best_error:
                best_error = error
                best_joints = result.x
            if result.success and error <= 0.04:
                break

        if best_error > 0.08:
            raise RuntimeError(
                f"Pipererror {best_error:.3f} mfor target {pose}")
        return JointSolution(best_joints.tolist())
