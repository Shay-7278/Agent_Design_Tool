# abb_irb120_description/launch/view_irb120.launch.py
#
# Non-GUI RViz visualization of the ABB IRB 120.
# Starts:
#   - robot_state_publisher (robot_description from xacro)
#   - rviz2 (optional config)
#
# Usage:
#   ros2 launch abb_irb120_description view_irb120.launch.py
#   ros2 launch abb_irb120_description view_irb120.launch.py use_rviz:=false
#   ros2 launch abb_irb120_description view_irb120.launch.py rviz_config:=/abs/path/to/file.rviz

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    pkg_share = FindPackageShare("abb_irb120_description")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "irb120_3_60.xacro"])
    default_rviz = PathJoinSubstitution([pkg_share, "rviz", "irb120.rviz"])

    robot_description = Command(["xacro ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="Absolute path to an RViz2 config file.",
            ),
            robot_state_publisher,
            rviz2,
        ]
    )
