#!/usr/bin/env python3
"""
Collect poses and perform calibration
"""

import enum
from contextlib import suppress
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Transform
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from .calibration_backend import CalibrationBackend
from .utilities import tf_to_pos_quat, tf_to_string


class CalibrationType(enum.Enum):
    EYE_ON_BASE = enum.auto()
    EYE_IN_HAND = enum.auto()


class CameraExtrinsicsCalibration(Node):
    def __init__(self):
        mname = "hand_eye_calibration"
        super().__init__(mname)

        self.declare_parameter("subscribed_tf_parent_frame", "")
        self.declare_parameter("subscribed_tf_child_frame", "")
        self.declare_parameter("robot_base_frame", "")
        self.declare_parameter("robot_effector_frame", "")
        self.declare_parameter("calibration_type", "eye-on-base")
        self.declare_parameter("calibrated_camera_transforms_filepath", "")

        self.subscribed_tf_parent_frame = str(self.get_parameter("subscribed_tf_parent_frame").value)
        self.subscribed_tf_child_frame = str(self.get_parameter("subscribed_tf_child_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.robot_effector_frame = str(self.get_parameter("robot_effector_frame").value)
        calibration_type = str(self.get_parameter("calibration_type").value)
        self.calibrated_camera_transforms_filepath = str(
            self.get_parameter("calibrated_camera_transforms_filepath").value
        )

        if calibration_type == "eye-in-hand":
            self.calibration_type = CalibrationType.EYE_IN_HAND
        elif calibration_type == "eye-on-base":
            self.calibration_type = CalibrationType.EYE_ON_BASE
        else:
            self.get_logger().error(f"Unknown calibration type: {calibration_type}")
            exit(1)

        self.capture_point_service = self.create_service(
            Trigger, mname + "/capture_point", self.capture_point_service_callback
        )

        self.tf_buffer = Buffer()
        self._listener = TransformListener(self.tf_buffer, self)

        self.robot_base_to_effector_samples = list()
        self.subscribed_parent_to_child_samples = list()

        self.get_logger().info("Hand-Eye Calibration Node initialized — purely reading TFs, not publishing anything")

    def capture_point_service_callback(self, request, response):
        robot_base_to_effector, subscribed_parent_to_child = self.retrieve_transforms()
        self.get_logger().info("robot: " + tf_to_string(robot_base_to_effector))
        self.get_logger().info("subscribed tf: " + tf_to_string(subscribed_parent_to_child))
        self.robot_base_to_effector_samples.append(robot_base_to_effector)
        self.subscribed_parent_to_child_samples.append(subscribed_parent_to_child)
        cal = self.calibrate()

        save_message = ""
        if cal is not None:
            save_message = self.persist_camera_calibration(cal)

        response.success = True
        response.message = self.generate_response_message(cal, save_message)
        self.get_logger().info(response.message)
        return response

    def retrieve_transforms(self) -> tuple[Transform, Transform]:
        try:
            robot_base_to_effector = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.robot_effector_frame, Time()
            )
            subscribed_parent_to_child = self.tf_buffer.lookup_transform(
                self.subscribed_tf_parent_frame, self.subscribed_tf_child_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().error("Could not get transforms")
            self.get_logger().error(str(ex))
            exit(1)
        return robot_base_to_effector.transform, subscribed_parent_to_child.transform

    def calibrate(self) -> Transform | None:
        if len(self.robot_base_to_effector_samples) < 4:
            return None
        self.get_logger().info("Estimating calibration...")
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            return CalibrationBackend.calibrate_eye_in_hand(
                self.robot_base_to_effector_samples, self.subscribed_parent_to_child_samples
            )
        if self.calibration_type == CalibrationType.EYE_ON_BASE:
            return CalibrationBackend.calibrate_eye_on_base(
                self.robot_base_to_effector_samples, self.subscribed_parent_to_child_samples
            )
        return None

    def resolve_transform_mapping(self) -> tuple[str, str, str] | None:
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            return self.subscribed_tf_parent_frame, self.robot_effector_frame, self.subscribed_tf_parent_frame
        if self.calibration_type == CalibrationType.EYE_ON_BASE:
            return self.subscribed_tf_parent_frame, self.robot_base_frame, self.subscribed_tf_parent_frame
        return None

    def resolve_calibration_file(self) -> Path:
        configured_path = self.calibrated_camera_transforms_filepath.strip()
        if configured_path:
            return Path(configured_path).expanduser().resolve()

        if get_package_share_directory is not None:
            try:
                return (
                    Path(get_package_share_directory("computer_vision")) / "config" / "calibrated_camera_transforms.yaml"
                ).resolve()
            except Exception:
                pass

        fallback_source_path = Path(
            "/workspace/ros2/src/sbtc-ros2-cv/computer_vision/config/calibrated_camera_transforms.yaml"
        )
        if fallback_source_path.exists():
            return fallback_source_path.resolve()

        return (Path.cwd() / "calibrated_camera_transforms.yaml").resolve()

    def calibration_file_targets(self) -> list[Path]:
        configured_path = self.calibrated_camera_transforms_filepath.strip()
        if configured_path:
            return [Path(configured_path).expanduser().resolve()]

        targets = [
            Path("/workspace/ros2/src/sbtc-ros2-cv/computer_vision/config/calibrated_camera_transforms.yaml"),
            Path("/workspace/ros2/install/computer_vision/share/computer_vision/config/calibrated_camera_transforms.yaml"),
        ]

        if get_package_share_directory is not None:
            with suppress(Exception):
                targets.append(
                    Path(get_package_share_directory("computer_vision"))
                    / "config"
                    / "calibrated_camera_transforms.yaml"
                )

        unique_targets = []
        for target in targets:
            resolved = target.resolve()
            if resolved not in unique_targets:
                unique_targets.append(resolved)
        return unique_targets

    def update_transform_file(
        self,
        path: Path,
        transform_key: str,
        parent_frame: str,
        child_frame: str,
        translation: list[float],
        rotation: list[float],
    ) -> None:
        calibration_data = {}
        if path.exists():
            with path.open("r", encoding="utf-8") as calibration_file:
                calibration_data = yaml.safe_load(calibration_file) or {}

        static_transforms = calibration_data.setdefault("static_transforms", {})
        transform_entry = static_transforms.setdefault(transform_key, {})
        transform_entry["translation"] = [f"{value:.4f}" for value in translation]
        transform_entry["rotation_xyzw"] = [f"{value:.4f}" for value in rotation]
        transform_entry["parent_frame"] = parent_frame
        transform_entry["child_frame"] = child_frame

        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as calibration_file:
            yaml.safe_dump(calibration_data, calibration_file, sort_keys=False)

    def persist_camera_calibration(self, calibration_transform: Transform) -> str:
        mapping = self.resolve_transform_mapping()
        if mapping is None:
            return "Calibration result computed; no camera calibration mapping configured, skipping YAML update."

        transform_key, parent_frame, child_frame = mapping
        translation, rotation = tf_to_pos_quat(calibration_transform)

        output_paths = self.calibration_file_targets()

        for path in output_paths:
            self.update_transform_file(path, transform_key, parent_frame, child_frame, translation, rotation)

        updated_paths_str = ", ".join(str(path) for path in output_paths)
        return f"Updated {updated_paths_str} -> static_transforms.{transform_key}"

    def generate_response_message(self, cal: Transform | None, save_message: str = "") -> str:
        if cal is None:
            return (
                f"Measurement taken ({len(self.robot_base_to_effector_samples)}/4 minimum). Not enough samples yet..."
            )

        calibration_message = ""
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            calibration_message = (
                f"Eye-in-Hand result — {self.robot_effector_frame} -> {self.subscribed_tf_parent_frame}:\n"
                f"{tf_to_string(cal)}\n"
            )
        elif self.calibration_type == CalibrationType.EYE_ON_BASE:
            calibration_message = (
                f"Eye-on-Base result — {self.robot_base_frame} -> {self.subscribed_tf_parent_frame}:\n{tf_to_string(cal)}\n"
            )
        else:
            calibration_message = "Calibration complete."

        if save_message:
            calibration_message = f"{calibration_message}{save_message}"
        return calibration_message


def main():
    rclpy.init()
    node = CameraExtrinsicsCalibration()
    with suppress(KeyboardInterrupt):
        rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
