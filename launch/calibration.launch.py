from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import launch
from launch import LaunchDescription


def generate_launch_description():

    subscribed_tf_parent_frame = DeclareLaunchArgument(
        "subscribed_tf_parent_frame", default_value="", description="e.g. camera frame"
    )
    subscribed_tf_child_frame = DeclareLaunchArgument("subscribed_tf_child_frame", default_value="")
    robot_base_frame = DeclareLaunchArgument("robot_base_frame", default_value="")
    robot_effector_frame = DeclareLaunchArgument("robot_effector_frame", default_value="")
    calibration_type = DeclareLaunchArgument(
        "calibration_type", default_value="", description="Options are eye-in-hand or eye-on-base"
    )
    node_namespace = DeclareLaunchArgument("node_namespace", default_value="")
    calibrated_camera_transforms_filepath = DeclareLaunchArgument(
        "calibrated_camera_transforms_filepath", default_value=""
    )

    calibration_node = Node(
        package="hand_eye_calibration",
        executable="hand_eye_calibration",
        name="hand_eye_calibration",
        namespace=launch.substitutions.LaunchConfiguration("node_namespace"),
        output="screen",
        parameters=[
            {"subscribed_tf_parent_frame": launch.substitutions.LaunchConfiguration("subscribed_tf_parent_frame")},
            {"subscribed_tf_child_frame": launch.substitutions.LaunchConfiguration("subscribed_tf_child_frame")},
            {"robot_base_frame": launch.substitutions.LaunchConfiguration("robot_base_frame")},
            {"robot_effector_frame": launch.substitutions.LaunchConfiguration("robot_effector_frame")},
            {"calibration_type": launch.substitutions.LaunchConfiguration("calibration_type")},
            {
                "calibrated_camera_transforms_filepath": launch.substitutions.LaunchConfiguration(
                    "calibrated_camera_transforms_filepath"
                )
            },
        ],
    )

    ll = list()
    ll.append(subscribed_tf_parent_frame)
    ll.append(subscribed_tf_child_frame)
    ll.append(robot_base_frame)
    ll.append(robot_effector_frame)
    ll.append(calibration_type)
    ll.append(node_namespace)
    ll.append(calibrated_camera_transforms_filepath)
    ll.append(calibration_node)

    return LaunchDescription(ll)
