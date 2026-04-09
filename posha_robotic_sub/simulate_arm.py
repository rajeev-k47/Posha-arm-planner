from pathlib import Path
from math import atan2, hypot
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from posha_robotic_sub.kinematics import PiperKinematics
from posha_robotic_sub.motion_planner import Planner, TaskPlan, Waypoint
from posha_robotic_sub.visualization import build_workspace_markers
from posha_robotic_sub.workspace import Workspace


ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT_NAMES = ["joint7", "joint8"]
JOINT_NAMES = ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES
GRIPPER_OPEN_POSITIONS = [0.025, -0.025]
GRIPPER_CLOSED_POSITIONS = [0.004, -0.004]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def piper_demo_joints(waypoint: Waypoint) -> List[float]:

    pose = waypoint.pose
    radius = hypot(pose.x, pose.y)
    base_yaw = atan2(pose.y, pose.x) if radius > 1e-6 else 0.0

    if waypoint.name in ("home", "home_return"):
        arm_joints = [0.0, 1.00, -1.35, 0.0, 0.35, 0.0]
    else:
        reach = clamp(radius, 0.05, 0.70)
        height = clamp(pose.z, 0.05, 1.10)
        shoulder = 0.65 + 0.95 * reach + 0.25 * height
        elbow = -1.10 - 0.75 * reach + 0.20 * (0.35 - height)
        wrist_pitch = clamp(0.25 + pose.pitch - 0.20 * height, -1.00, 1.00)
        wrist_yaw = clamp(pose.yaw - base_yaw, -1.40, 1.40)
        arm_joints = [
            clamp(base_yaw, -2.60, 2.60),
            clamp(shoulder, 0.05, 2.20),
            clamp(elbow, -2.50, -0.10),
            0.0,
            wrist_pitch,
            wrist_yaw,
        ]

    gripper = GRIPPER_CLOSED_POSITIONS if waypoint.name not in (
        "home",
        "home_return",
        "source_above",
        "place",
    ) else GRIPPER_OPEN_POSITIONS
    return arm_joints + gripper


class PlaybackNode(Node):
    def __init__(self) -> None:
        super().__init__("posha_robotic_sub_playback")
        self.declare_parameter("task_name", "macro_container_5_to_pan_2")
        self.declare_parameter("hold_frames", 6)
        self.declare_parameter("interpolation_steps", 12)

        self.workspace = Workspace()
        self.planner = Planner(self.workspace)
        self.kinematics = PiperKinematics()
        all_plans: Dict[str, TaskPlan] = {
            task.name: self.planner.plan_task(task) for task in self.workspace.tasks
        }
        selected_task = str(self.get_parameter("task_name").value)
        if selected_task == "all":
            self.plans = all_plans
        else:
            self.plans = {selected_task: all_plans[selected_task]}
        self.frames = self._build_frames()
        self.frame_index = 0

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/posha_robotic_sub/markers", latched_qos)
        self.create_timer(0.1, self.publish_frame)
        self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(
            f"Loaded {len(self.plans)} task plans and {len(self.frames)} playback frames."
        )

    def _build_frames(self) -> List[tuple[str, str, List[float]]]:
        frames: List[tuple[str, str, List[float]]] = []
        hold_frames = int(self.get_parameter("hold_frames").value)
        interpolation_steps = int(
            self.get_parameter("interpolation_steps").value)
        for task_name, plan in self.plans.items():
            waypoint_joints = []
            seed = piper_demo_joints(plan.waypoints[0])[:6]
            for waypoint in plan.waypoints:
                try:
                    arm_joints = self.kinematics.inverse_kinematics(
                        waypoint.pose, seed=seed).joints
                    seed = arm_joints
                except RuntimeError as exc:
                    self.get_logger().warn(
                        f"{task_name}:{waypoint.name}: {exc}. Using demo pose.")
                    arm_joints = piper_demo_joints(waypoint)[:6]
                    seed = arm_joints
                gripper = GRIPPER_CLOSED_POSITIONS if waypoint.name not in (
                    "home",
                    "home_return",
                    "source_above",
                    "place",
                ) else GRIPPER_OPEN_POSITIONS
                waypoint_joints.append((waypoint.name, arm_joints + gripper))
            for index, (waypoint_name, joints) in enumerate(waypoint_joints):
                for _ in range(hold_frames):
                    frames.append((task_name, waypoint_name, joints))
                if index + 1 >= len(waypoint_joints):
                    continue
                next_name, next_joints = waypoint_joints[index + 1]
                for step in range(1, interpolation_steps + 1):
                    blend = step / (interpolation_steps + 1)
                    interpolated = [
                        joints_value * (1.0 - blend) + next_value * blend
                        for joints_value, next_value in zip(joints, next_joints)
                    ]
                    frames.append(
                        (task_name, f"{waypoint_name}_to_{next_name}", interpolated))
        return frames

    def publish_frame(self) -> None:
        task_name, waypoint_name, joints = self.frames[self.frame_index]
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = joints
        self.joint_pub.publish(message)
        hold_frames = int(self.get_parameter("hold_frames").value)
        if self.frame_index % max(1, hold_frames) == 0:
            self.get_logger().info(
                f"{task_name}: {waypoint_name}", throttle_duration_sec=2.0)
        self.frame_index = (self.frame_index + 1) % len(self.frames)

    def publish_markers(self) -> None:
        self.marker_pub.publish(build_workspace_markers(
            self.workspace, self.plans, frame_id="world"))


def load_robot_description() -> str:
    share_dir = Path(get_package_share_directory("posha_robotic_sub"))
    urdf_path = share_dir / "resource" / "piper_description.urdf"
    content = urdf_path.read_text()
    return content.replace("package://piper_description/meshes/", "package://posha_robotic_sub/resource/meshes/")


def main() -> None:
    rclpy.init()
    node = PlaybackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
