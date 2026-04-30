from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import launch
from launch import LaunchDescription


def generate_launch_description():

    tracking_base_frame = DeclareLaunchArgument(
        "tracking_base_frame", default_value="", description="e.g. camera frame"
    )
    tracking_marker_frame = DeclareLaunchArgument("tracking_marker_frame", default_value="")
    robot_base_frame = DeclareLaunchArgument("robot_base_frame", default_value="")
    robot_effector_frame = DeclareLaunchArgument("robot_effector_frame", default_value="")
    calibration_type = DeclareLaunchArgument(
        "calibration_type", default_value="", description="Options are eye-in-hand or eye-on-base"
    )
    node_namespace = DeclareLaunchArgument("node_namespace", default_value="")
    camera_calibration_file = DeclareLaunchArgument("camera_calibration_file", default_value="")
    calibration_transform_key = DeclareLaunchArgument("calibration_transform_key", default_value="")
    calibration_parent_frame = DeclareLaunchArgument("calibration_parent_frame", default_value="")
    calibration_child_frame = DeclareLaunchArgument("calibration_child_frame", default_value="")

    calibration_node = Node(
        package="hand_eye_calibration",
        executable="hand_eye_calibration",
        name="hand_eye_calibration",
        namespace=launch.substitutions.LaunchConfiguration("node_namespace"),
        output="screen",
        parameters=[
            {"tracking_base_frame": launch.substitutions.LaunchConfiguration("tracking_base_frame")},
            {"tracking_marker_frame": launch.substitutions.LaunchConfiguration("tracking_marker_frame")},
            {"robot_base_frame": launch.substitutions.LaunchConfiguration("robot_base_frame")},
            {"robot_effector_frame": launch.substitutions.LaunchConfiguration("robot_effector_frame")},
            {"calibration_type": launch.substitutions.LaunchConfiguration("calibration_type")},
            {"camera_calibration_file": launch.substitutions.LaunchConfiguration("camera_calibration_file")},
            {"calibration_transform_key": launch.substitutions.LaunchConfiguration("calibration_transform_key")},
            {"calibration_parent_frame": launch.substitutions.LaunchConfiguration("calibration_parent_frame")},
            {"calibration_child_frame": launch.substitutions.LaunchConfiguration("calibration_child_frame")},
        ],
    )

    ll = list()
    ll.append(tracking_base_frame)
    ll.append(tracking_marker_frame)
    ll.append(robot_base_frame)
    ll.append(robot_effector_frame)
    ll.append(calibration_type)
    ll.append(node_namespace)
    ll.append(camera_calibration_file)
    ll.append(calibration_transform_key)
    ll.append(calibration_parent_frame)
    ll.append(calibration_child_frame)
    ll.append(calibration_node)

    return LaunchDescription(ll)
