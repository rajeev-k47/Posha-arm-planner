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
        return [
            BoxObstacle("counter_surface", -1.0, 1.0, -1.0, 1.0, -0.05, 0.0),
            BoxObstacle("pod_wall_negative_y", -0.05,
                        0.05, -0.40, -0.15, 0.15, 1.10),
            BoxObstacle("pod_wall_positive_y", -0.05,
                        0.05, 0.15, 0.30, 0.15, 1.10),
        ]

    def colliding_obstacles(self, pose: Pose) -> List[str]:
        collisions = []
        for obstacle in self.obstacles:
            if obstacle.contains(pose):
                collisions.append(obstacle.name)
        return collisions


if __name__ == "__main__":
    workspace = Workspace()
    checker = CollisionChecker(workspace)
