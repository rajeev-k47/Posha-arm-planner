import json
from dataclasses import asdict
from pathlib import Path

from posha_robotic_sub.collision import CollisionChecker
from posha_robotic_sub.kinematics import SimpleKinematics
from posha_robotic_sub.motion_planner import Planner
from posha_robotic_sub.workspace import Workspace


def main():
    workspace = Workspace()
    planner = Planner(workspace)
    kinematics = SimpleKinematics()
    collision_checker = CollisionChecker(workspace)

    all_plans = {}

    for task in workspace.tasks:
        plan = planner.plan_task(task)
        waypoint_entries = []
        for index, waypoint in enumerate(plan.waypoints):
            transition_collisions = []
            if index > 0:
                previous_waypoint = plan.waypoints[index - 1]
                transition_collisions = collision_checker.path_collisions(
                    previous_waypoint.pose,
                    waypoint.pose,
                )
            waypoint_entries.append(
                {
                    "name": waypoint.name,
                    "pose": asdict(waypoint.pose),
                    "joints": kinematics.inverse_kinematics(waypoint.pose).joints,
                    "collisions": collision_checker.colliding_obstacles(waypoint.pose),
                    "transition_collisions": transition_collisions,
                }
            )

        all_plans[task.name] = {
            "task_name": plan.task_name,
            "source_name": task.source_name,
            "target_name": task.target_name,
            "waypoints": waypoint_entries,
        }

    output_dir = Path.cwd() / "outputs"
    output_dir.mkdir(exist_ok=True)

    output_file = output_dir / "plans.json"
    output_file.write_text(json.dumps(all_plans, indent=2))
#    generate_report(all_plans, output_dir)
