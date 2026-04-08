from dataclasses import dataclass
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

    def plan_task(self, task: Task) -> TaskPlan:
        source = self.workspace.targets[task.source_name]
        target = self.workspace.targets[task.target_name]

        source_above = Pose(
            source.pose.x,
            source.pose.y,
            source.pose.z + task.transfer_height
        )

        grasp = Pose(
            source.pose.x,
            source.pose.y,
            source.pose.z + task.grasp_clearance
        )

        lift = Pose(
            source.pose.x,
            source.pose.y,
            source.pose.z + task.transfer_height
        )

        target_above = Pose(
            target.pose.x,
            target.pose.y,
            target.pose.z + task.dispense_height
        )

        dispense = Pose(
            target.pose.x,
            target.pose.y,
            target.pose.z + task.dispense_height
        )

        return_pose = Pose(
            source.pose.x,
            source.pose.y,
            source.pose.z + task.transfer_height
        )

        waypoints = [
            Waypoint("source_above", source_above),
            Waypoint("grasp", grasp),
            Waypoint("lift", lift),
            Waypoint("target_above", target_above),
            Waypoint("dispense", dispense),
            Waypoint("return", return_pose),
        ]

        return TaskPlan(task.name, waypoints)


if __name__ == "__main__":
    workspace = Workspace()
    planner = Planner(workspace)
