from dataclasses import dataclass
from math import atan2
from typing import List

from posha_robotic_sub.workspace import Workspace, Pose, Task


@dataclass(frozen=True)
class Waypoint:
    name: str
    pose: Pose


@dataclass(frozen=True)
class TaskPlan:
    task_name: str
    waypoints: List[Waypoint]


class Planner:
    def __init__(self, workspace: Workspace):
        self.workspace = workspace

    @staticmethod
    def _pose_with_offset(
        base_pose: Pose,
        dz: float = 0.0,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
    ) -> Pose:
        return Pose(
            base_pose.x,
            base_pose.y,
            base_pose.z + dz,
            base_pose.roll if roll is None else roll,
            base_pose.pitch if pitch is None else pitch,
            base_pose.yaw if yaw is None else yaw,
        )

    def plan_task(self, task: Task) -> TaskPlan:
        source = self.workspace.targets[task.source_name]
        target = self.workspace.targets[task.target_name]

        home = Pose(0.0, 0.0, 0.30, roll=0.0, pitch=0.0, yaw=0.0)
        source_yaw = atan2(
            source.pose.y, source.pose.x) if source.pose.x != 0.0 or source.pose.y != 0.0 else source.pose.yaw
        target_yaw = atan2(
            target.pose.y, target.pose.x) if target.pose.x != 0.0 or target.pose.y != 0.0 else target.pose.yaw
        pickup_pitch = -0.2 if source.kind == "container" else -0.1
        dispense_pitch = 0.4 if source.kind == "container" else 0.2

        source_above = self._pose_with_offset(
            source.pose,
            dz=task.transfer_height,
            pitch=pickup_pitch,
            yaw=source_yaw,
        )

        grasp = self._pose_with_offset(
            source.pose,
            dz=task.grasp_clearance,
            pitch=pickup_pitch,
            yaw=source_yaw,
        )

        lift = self._pose_with_offset(
            source.pose,
            dz=task.transfer_height,
            pitch=pickup_pitch,
            yaw=source_yaw,
        )

        park = Pose(
            (source.pose.x + target.pose.x) / 2.0,
            (source.pose.y + target.pose.y) / 2.0,
            max(source.pose.z, target.pose.z) + task.transfer_height + 0.05,
            roll=0.0,
            pitch=0.0,
            yaw=(source_yaw + target_yaw) / 2.0,
        )

        target_above = self._pose_with_offset(
            target.pose,
            dz=task.dispense_height + 0.08,
            pitch=0.0,
            yaw=target_yaw,
        )

        dispense = self._pose_with_offset(
            target.pose,
            dz=task.dispense_height,
            pitch=dispense_pitch,
            yaw=target_yaw,
        )

        target_retreat = self._pose_with_offset(
            target.pose,
            dz=task.dispense_height + 0.08,
            pitch=0.0,
            yaw=target_yaw,
        )

        home_return = Pose(0.0, 0.0, 0.30, roll=0.0, pitch=0.0, yaw=0.0)

        waypoints = [
            Waypoint("home", home),
            Waypoint("source_above", source_above),
            Waypoint("grasp", grasp),
            Waypoint("lift", lift),
            Waypoint("park", park),
            Waypoint("target_above", target_above),
            Waypoint("dispense", dispense),
            Waypoint("target_retreat", target_retreat),
            Waypoint("home_return", home_return),
        ]

        return TaskPlan(task.name, waypoints)
