from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Target:
    name: str
    kind: str
    pose: Pose


@dataclass(frozen=True)
class Task:
    name: str
    source_name: str
    target_name: str
    grasp_clearance: float
    transfer_height: float
    dispense_height: float


class Workspace:
    def __init__(self):

        self.targets: Dict[str, Target] = {
            "container_1": Target("container_1", "container", Pose(0.10, -0.10, 0.05)),
            "container_5": Target("container_5", "container", Pose(0.60, 0.10, 0.05)),
            "pod_1": Target("pod_1", "pod", Pose(0.00, -0.30, 0.95)),
            "pod_19": Target("pod_19", "pod", Pose(0.00, 0.20, 0.25)),
            "pan_1": Target("pan_1", "pan", Pose(0.20, -0.25, 0.05)),
            "pan_2": Target("pan_2", "pan", Pose(0.20, 0.25, 0.05)),
        }

        self.tasks: List[Task] = [
            Task("macro_container_5_to_pan_2",
                 "container_5", "pan_2", 0.005, 0.14, 0.10),
            Task("macro_container_1_to_pan_1",
                 "container_1", "pan_1", 0.005, 0.14, 0.10),
            Task("micro_pod_1_to_pan_2", "pod_1",
                 "pan_2", 0.002, 0.12, 0.09),
            Task("micro_pod_19_to_pan_1", "pod_19",
                 "pan_1", 0.002, 0.12, 0.09),
        ]


if __name__ == "__main__":
    workspace = Workspace()
