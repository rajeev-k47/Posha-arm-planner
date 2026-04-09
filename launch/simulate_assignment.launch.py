from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from posha_robotic_sub.simulate_arm import load_robot_description


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("posha_robotic_sub")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    task_name = LaunchConfiguration("task_name")
    hold_frames = LaunchConfiguration("hold_frames")
    interpolation_steps = LaunchConfiguration("interpolation_steps")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config", default_value=f"{share_dir}/config/assignment.rviz"),
            DeclareLaunchArgument(
                "task_name", default_value="macro_container_5_to_pan_2"),
            DeclareLaunchArgument("hold_frames", default_value="6"),
            DeclareLaunchArgument("interpolation_steps", default_value="12"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": load_robot_description()}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_dummy_link",
                arguments=["0", "0", "0", "0", "0",
                           "0", "world", "dummy_link"],
            ),
            Node(
                package="posha_robotic_sub",
                executable="simulate_assignment",
                name="simulate_assignment",
                output="screen",
                parameters=[{
                    "task_name": task_name,
                    "hold_frames": hold_frames,
                    "interpolation_steps": interpolation_steps,
                }],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
