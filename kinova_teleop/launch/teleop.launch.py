"""Launch perception, guarded teleoperation, and MoveIt Servo."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    config_package = LaunchConfiguration("moveit_config_package").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    command_frame = LaunchConfiguration("command_frame").perform(context)
    ee_frame = LaunchConfiguration("ee_frame").perform(context)
    move_group = LaunchConfiguration("move_group_name").perform(context)

    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=config_package).to_moveit_configs()
    servo_path = os.path.join(
        get_package_share_directory("kinova_teleop"), "config", "servo.yaml")
    with open(servo_path, encoding="utf-8") as stream:
        servo_params = yaml.safe_load(stream)["moveit_servo"]["ros__parameters"]
    servo_params.update({
        "move_group_name": move_group,
        "planning_frame": base_frame,
        "ee_frame_name": ee_frame,
        "robot_link_command_frame": command_frame,
        "command_out_topic": LaunchConfiguration("controller_topic").perform(context),
        "cartesian_command_in_topic": LaunchConfiguration("servo_topic").perform(context),
    })

    common = {
        "base_frame": base_frame,
        "command_frame": command_frame,
        "ee_frame": ee_frame,
        "servo_topic": LaunchConfiguration("servo_topic").perform(context),
        "gripper_action": LaunchConfiguration("gripper_action").perform(context),
        "axis_map": LaunchConfiguration("axis_map").perform(context),
        "x_limits": LaunchConfiguration("x_limits").perform(context),
        "y_limits": LaunchConfiguration("y_limits").perform(context),
        "z_limits": LaunchConfiguration("z_limits").perform(context),
    }
    camera = {
        "color_topic": LaunchConfiguration("color_topic").perform(context),
        "depth_topic": LaunchConfiguration("depth_topic").perform(context),
        "camera_info_topic": LaunchConfiguration("camera_info_topic").perform(context),
        "camera_frame": LaunchConfiguration("camera_frame").perform(context),
    }

    return [
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            output="screen",
            parameters=[
                {"moveit_servo": servo_params},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
        Node(
            package="kinova_hand_perception",
            executable="hand_tracker",
            output="screen",
            parameters=[camera],
        ),
        Node(
            package="kinova_teleop",
            executable="teleop_bridge",
            output="screen",
            parameters=[common],
        ),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        DeclareLaunchArgument("robot_name", default_value="kinova_gen3_7dof_robotiq_2f_85"),
        DeclareLaunchArgument("move_group_name", default_value="manipulator"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("command_frame", default_value="base_link"),
        DeclareLaunchArgument("ee_frame", default_value="tool_frame"),
        DeclareLaunchArgument("servo_topic", default_value="/servo_node/delta_twist_cmds"),
        DeclareLaunchArgument(
            "controller_topic", default_value="/joint_trajectory_controller/joint_trajectory"),
        DeclareLaunchArgument(
            "gripper_action", default_value="/robotiq_gripper_controller/gripper_cmd"),
        DeclareLaunchArgument("axis_map", default_value="z,-x,-y"),
        DeclareLaunchArgument("x_limits", default_value="0.20,0.70"),
        DeclareLaunchArgument("y_limits", default_value="-0.40,0.40"),
        DeclareLaunchArgument("z_limits", default_value="0.10,0.75"),
        DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument(
            "depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])

