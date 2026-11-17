import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory("abb_irb4600_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    default_xacro = os.path.join(
        gazebo_share, "urdf", "abb_irb4600_assembly.gazebo.xacro"
    )
    default_world = os.path.join(
        gazebo_share, "worlds", "irb4600_workcell.sdf"
    )

    xacro_file = LaunchConfiguration("xacro_file")
    world_file = LaunchConfiguration("world_file")
    start_gui = LaunchConfiguration("start_gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pickup_file = LaunchConfiguration("pickup_file")
    placement_file = LaunchConfiguration("placement_file")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str,
        )
    }

    resource_prefixes = [
        os.path.join(get_package_prefix(package_name), "share")
        for package_name in (
            "abb_irb4600_description",
            "abb_irb4600_irbt4004_description",
            "abb_irb4600_workcell_description",
            "abb_irb4600_gazebo",
        )
    ]
    resource_path = os.pathsep.join(
        [*resource_prefixes, os.environ.get("GZ_SIM_RESOURCE_PATH", "")]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-s -r -v 2 ", world_file]}.items(),
    )

    gazebo_gui = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", "-g"],
        output="screen",
        condition=IfCondition(start_gui),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    spawn_workcell = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_irb4600_workcell",
        arguments=[
            "-name",
            "abb_irb4600_workcell",
            "-topic",
            "robot_description",
            "-allow_renaming",
            "false",
        ],
        output="screen",
    )

    spawn_pickup_grid = Node(
        package="abb_irb4600_gazebo",
        executable="spawn_pickup_grid",
        name="spawn_pickup_grid",
        arguments=[
            "--pickup-file", pickup_file,
            "--placement-file", placement_file,
            "--model-dir", os.path.join(gazebo_share, "models"),
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    left_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "LL_manipulator_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "LL_gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "30",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "xacro_file",
                default_value=default_xacro,
                description="Controlled workcell Xacro (default: LL assembly cell)",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=default_world,
                description="Absolute path to the Gazebo world",
            ),
            DeclareLaunchArgument(
                "start_gui",
                default_value="false",
                description="Start a separate Gazebo GUI client",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "pickup_file",
                default_value=os.path.join(
                    get_package_share_directory("abb_irb4600_workcell_description"),
                    "config", "fixed_pickup_layout.json"),
            ),
            DeclareLaunchArgument(
                "placement_file",
                default_value="/ros2_ws/inputs/brick_frames_numeric.json",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            gazebo_server,
            gazebo_gui,
            clock_bridge,
            robot_state_publisher,
            spawn_workcell,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_workcell,
                    on_exit=[joint_state_broadcaster, spawn_pickup_grid],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[left_controller, gripper_controller],
                )
            ),
        ]
    )
