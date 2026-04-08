import json
from dataclasses import asdict
from pathlib import Path

from posha_robotic_sub.collision import SimpleCollisionChecker
from posha_robotic_sub.kinematics import SimpleKinematics
from posha_robotic_sub.motion_planner import Planner
from posha_robotic_sub.workspace import Workspace


def main():
    workspace = Workspace()
    planner = Planner(workspace)
    kinematics = SimpleKinematics()
    collision_checker = SimpleCollisionChecker(workspace)

    all_plans = {}

    for task in workspace.tasks:
        plan = planner.plan_task(task)
        all_plans[task.name] = {
            "task_name": plan.task_name,
            "source_name": task.source_name,
            "target_name": task.target_name,
            "waypoints": [
                {
                    "name": waypoint.name,
                    "pose": asdict(waypoint.pose),
                    "joints": kinematics.inverse_kinematics(waypoint.pose).joints,
                    "collisions": collision_checker.colliding_obstacles(waypoint.pose),
                }
                for waypoint in plan.waypoints
            ],
        }

    output_dir = Path.cwd() / "outputs"
    output_dir.mkdir(exist_ok=True)

    output_file = output_dir / "plans.json"
    output_file.write_text(json.dumps(all_plans, indent=2))

    print(f"Saved plans to {output_file}")


if __name__ == "__main__":
    main()
