from dataclasses import dataclass
from typing import Dict, List

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from posha_robotic_sub.collision import CollisionChecker
from posha_robotic_sub.motion_planner import TaskPlan
from posha_robotic_sub.workspace import Workspace


@dataclass(frozen=True)
class Rgba:
    r: float
    g: float
    b: float
    a: float


def _color(rgba: Rgba) -> ColorRGBA:
    return ColorRGBA(r=rgba.r, g=rgba.g, b=rgba.b, a=rgba.a)


def _point(x: float, y: float, z: float) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def build_workspace_markers(workspace: Workspace, plans: Dict[str, TaskPlan], frame_id: str = "base_link") -> MarkerArray:
    markers: List[Marker] = []
    marker_id = 0
    collision_checker = CollisionChecker(workspace)

    for obstacle in collision_checker.obstacles:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "obstacles"
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = _point(
            (obstacle.min_x + obstacle.max_x) / 2.0,
            (obstacle.min_y + obstacle.max_y) / 2.0,
            (obstacle.min_z + obstacle.max_z) / 2.0,
        )
        marker.pose.orientation.w = 1.0
        marker.scale.x = obstacle.max_x - obstacle.min_x
        marker.scale.y = obstacle.max_y - obstacle.min_y
        marker.scale.z = obstacle.max_z - obstacle.min_z
        marker.color = _color(Rgba(0.5, 0.5, 0.5, 0.45))
        markers.append(marker)

    for name, target in workspace.targets.items():
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "targets"
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.CYLINDER if target.kind == "pan" else Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = _point(
            target.pose.x, target.pose.y, target.pose.z)
        marker.pose.orientation.w = 1.0
        if target.kind == "pan":
            marker.scale.x = 0.26
            marker.scale.y = 0.26
            marker.scale.z = 0.05
        elif target.kind == "container":
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.10
        else:
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.08
        if target.kind == "pan":
            marker.color = _color(Rgba(1.0, 0.5, 0.1, 0.9))
        elif target.kind == "container":
            marker.color = _color(Rgba(0.2, 0.9, 0.2, 0.9))
        else:
            marker.color = _color(Rgba(0.2, 0.6, 1.0, 0.9))
        markers.append(marker)

        label = Marker()
        label.header.frame_id = frame_id
        label.ns = "labels"
        label.id = marker_id
        marker_id += 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = _point(
            target.pose.x, target.pose.y, target.pose.z + 0.06)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.035
        label.color = _color(Rgba(1.0, 1.0, 1.0, 1.0))
        label.text = name
        markers.append(label)

    colors = [
        Rgba(1.0, 0.2, 0.2, 0.9),
        Rgba(0.2, 0.8, 1.0, 0.9),
        Rgba(0.4, 1.0, 0.4, 0.9),
        Rgba(1.0, 0.9, 0.2, 0.9),
    ]
    for color, (task_name, plan) in zip(colors, plans.items()):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "paths"
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.color = _color(color)
        marker.points = [
            _point(waypoint.pose.x, waypoint.pose.y, waypoint.pose.z)
            for waypoint in plan.waypoints
        ]
        markers.append(marker)

    return MarkerArray(markers=markers)
