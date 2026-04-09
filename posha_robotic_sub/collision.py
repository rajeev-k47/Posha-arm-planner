from dataclasses import dataclass
from typing import List

from posha_robotic_sub.workspace import Pose, Workspace


@dataclass(frozen=True)
class BoxObstacle:
    name: str
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    def contains(self, pose: Pose) -> bool:
        return (
            self.min_x <= pose.x <= self.max_x
            and self.min_y <= pose.y <= self.max_y
            and self.min_z <= pose.z <= self.max_z
        )


class CollisionChecker:
    def __init__(self, workspace: Workspace):
        self.workspace = workspace
        self.obstacles = self.build_obstacles()

    def build_obstacles(self) -> List[BoxObstacle]:
        pan_1 = self.workspace.targets["pan_1"].pose
        pan_2 = self.workspace.targets["pan_2"].pose

        container_1 = self.workspace.targets["container_1"].pose
        container_5 = self.workspace.targets["container_5"].pose

        pod_1 = self.workspace.targets["pod_1"].pose
        pod_19 = self.workspace.targets["pod_19"].pose

        return [
            BoxObstacle("counter_surface", -1.0, 1.0, -1.0, 1.0, -0.05, 0.0),
            BoxObstacle("robot_base_keepout", -0.10,
                        0.10, -0.10, 0.10, 0.0, 0.20),
            BoxObstacle(
                "macro_zone",
                min(container_1.x, container_5.x) - 0.08,
                max(container_1.x, container_5.x) + 0.08,
                min(container_1.y, container_5.y) - 0.05,
                max(container_1.y, container_5.y) + 0.05,
                0.0,
                max(container_1.z, container_5.z) + 0.05,
            ),
            BoxObstacle(
                "pod_wall_negative_y",
                min(pod_1.x, pod_19.x) - 0.05,
                max(pod_1.x, pod_19.x) + 0.10,
                -0.40,
                -0.15,
                0.15,
                max(pod_1.z, pod_19.z) + 0.10,
            ),
            BoxObstacle(
                "pod_wall_positive_y",
                min(pod_1.x, pod_19.x) - 0.05,
                max(pod_1.x, pod_19.x) + 0.10,
                0.15,
                0.30,
                0.15,
                max(pod_1.z, pod_19.z) + 0.10,
            ),
            BoxObstacle(
                "pan_1_body",
                pan_1.x - 0.14,
                pan_1.x + 0.14,
                pan_1.y - 0.14,
                pan_1.y + 0.14,
                0.0,
                pan_1.z + 0.05,
            ),
            BoxObstacle(
                "pan_2_body",
                pan_2.x - 0.14,
                pan_2.x + 0.14,
                pan_2.y - 0.14,
                pan_2.y + 0.14,
                0.0,
                pan_2.z + 0.05,
            ),
            BoxObstacle("rear_wall", -0.10, 0.02, -0.50, 0.50, 0.0, 1.20),
        ]

    def colliding_obstacles(self, pose: Pose) -> List[str]:
        collisions = []
        for obstacle in self.obstacles:
            if obstacle.contains(pose):
                collisions.append(obstacle.name)
        return collisions

    def path_collisions(self, start: Pose, end: Pose, samples: int = 20) -> List[str]:
        collisions = set()
        for i in range(samples + 1):
            t = i / samples
            sample_pose = Pose(
                x=start.x + (end.x - start.x) * t,
                y=start.y + (end.y - start.y) * t,
                z=start.z + (end.z - start.z) * t,
                roll=start.roll + (end.roll - start.roll) * t,
                pitch=start.pitch + (end.pitch - start.pitch) * t,
                yaw=start.yaw + (end.yaw - start.yaw) * t,
            )
            for obstacle in self.colliding_obstacles(sample_pose):
                collisions.add(obstacle)
        return sorted(collisions)
