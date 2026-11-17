#!/usr/bin/env python3
"""Validate the installed dual-IRB4600 Gazebo control configuration."""

from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml


LEFT_JOINTS = ["joint_LL_ext_1", *[f"joint_LL_{index}" for index in range(1, 7)]]
RIGHT_JOINTS = ["joint_RR_ext_1", *[f"joint_RR_{index}" for index in range(1, 7)]]
GRIPPER_JOINTS = ['LL_schunk_left_finger_joint','LL_schunk_right_finger_joint']
EXPECTED_JOINTS = LEFT_JOINTS + GRIPPER_JOINTS + RIGHT_JOINTS


def validate_configuration(assembly_only=False) -> None:
    package_share = Path(get_package_share_directory("abb_irb4600_gazebo"))
    xacro_file = package_share / "urdf" / (
        "abb_irb4600_assembly.gazebo.xacro" if assembly_only else "abb_irb4600_workcell.gazebo.xacro")
    controllers_file = package_share / "config" / (
        "assembly_controllers.yaml" if assembly_only else "ros2_controllers.yaml")
    positions_file = package_share / "config" / (
        "assembly_initial_positions.yaml" if assembly_only else "initial_positions.yaml")
    expected_joints = LEFT_JOINTS + GRIPPER_JOINTS + ([] if assembly_only else RIGHT_JOINTS)

    with tempfile.TemporaryDirectory() as temporary_directory:
        urdf_file = Path(temporary_directory) / "abb_irb4600_workcell.urdf"
        subprocess.run(
            ["xacro", str(xacro_file), "-o", str(urdf_file)],
            check=True,
        )
        robot = ET.parse(urdf_file).getroot()

    physical_joint_names = [joint.attrib["name"] for joint in robot.findall("joint")]
    assert len(physical_joint_names) == len(set(physical_joint_names)), (
        "Generated URDF contains duplicate physical joint names"
    )
    assert set(expected_joints).issubset(physical_joint_names), (
        "A controlled joint is missing from the generated workcell"
    )

    ros2_control = robot.find("ros2_control")
    assert ros2_control is not None, "Generated URDF has no ros2_control system"
    controlled_joint_names = [
        joint.attrib["name"] for joint in ros2_control.findall("joint")
    ]
    assert controlled_joint_names == expected_joints, (
        "ros2_control joint order or membership is incorrect"
    )

    physical_joints = {
        joint.attrib["name"]: joint for joint in robot.findall("joint")
    }
    for controlled_joint in ros2_control.findall("joint"):
        joint_name = controlled_joint.attrib["name"]
        command_interfaces = [
            interface.attrib["name"]
            for interface in controlled_joint.findall("command_interface")
        ]
        state_interfaces = [
            interface.attrib["name"]
            for interface in controlled_joint.findall("state_interface")
        ]
        assert command_interfaces == ["position"], (
            f"{joint_name} must expose one position command interface"
        )
        assert state_interfaces == ["position", "velocity"], (
            f"{joint_name} must expose position and velocity state interfaces"
        )

        initial_value_element = controlled_joint.find(
            "state_interface[@name='position']/param[@name='initial_value']"
        )
        assert initial_value_element is not None
        initial_value = float(initial_value_element.text)
        joint_limit = physical_joints[joint_name].find("limit")
        assert joint_limit is not None
        lower = float(joint_limit.attrib["lower"])
        upper = float(joint_limit.attrib["upper"])
        assert lower <= initial_value <= upper, (
            f"{joint_name} initial position {initial_value} is outside "
            f"[{lower}, {upper}]"
        )

    hardware_plugin = ros2_control.find("hardware/plugin")
    assert hardware_plugin is not None
    assert hardware_plugin.text == "gz_ros2_control/GazeboSimSystem"

    gazebo_plugin = robot.find("gazebo/plugin")
    assert gazebo_plugin is not None, "Gazebo control plugin is missing"
    assert gazebo_plugin.attrib["filename"] == "libgz_ros2_control-system.so"

    controllers = yaml.safe_load(controllers_file.read_text(encoding="utf-8"))
    manager_parameters = controllers["controller_manager"]["ros__parameters"]
    assert manager_parameters["joint_state_broadcaster"]["type"] == (
        "joint_state_broadcaster/JointStateBroadcaster"
    )
    assert manager_parameters["LL_manipulator_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    if not assembly_only:
        assert manager_parameters["RR_manipulator_controller"]["type"] == (
            "joint_trajectory_controller/JointTrajectoryController"
        )
    left_mapping = controllers["LL_manipulator_controller"]["ros__parameters"][
        "joints"
    ]
    right_mapping = [] if assembly_only else controllers["RR_manipulator_controller"]["ros__parameters"]["joints"]
    assert left_mapping == LEFT_JOINTS, "Left controller joint mapping is incorrect"
    assert right_mapping == ([] if assembly_only else RIGHT_JOINTS), "Right controller joint mapping is incorrect"
    assert controllers['LL_gripper_controller']['ros__parameters']['joints'] == GRIPPER_JOINTS
    assert manager_parameters['LL_gripper_controller']['type'] == 'joint_trajectory_controller/JointTrajectoryController'
    assert set(left_mapping).isdisjoint(right_mapping), (
        "Left and right controllers must not claim the same joint"
    )

    initial_positions = yaml.safe_load(positions_file.read_text(encoding="utf-8"))[
        "initial_positions"
    ]
    assert set(initial_positions) == set(expected_joints), (
        "Initial-position keys do not exactly match the controlled joints"
    )

    print(
        f"PASS: {'assembly' if assembly_only else 'full'} URDF has unique joints, "
        f"{len(expected_joints)} exact control mappings, and the Gazebo control plugin"
    )


def main() -> None:
    validate_configuration()
    validate_configuration(assembly_only=True)


if __name__ == "__main__":
    main()
