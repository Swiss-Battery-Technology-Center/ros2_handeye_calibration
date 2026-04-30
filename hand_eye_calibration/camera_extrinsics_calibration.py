#!/usr/bin/env python3
"""
Collect poses and perform calibration
"""

import enum
import shutil
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

        self.declare_parameter("tracking_base_frame", "")
        self.declare_parameter("tracking_marker_frame", "")
        self.declare_parameter("robot_base_frame", "")
        self.declare_parameter("robot_effector_frame", "")
        self.declare_parameter("calibration_type", "eye-on-base")
        self.declare_parameter("camera_calibration_file", "")
        self.declare_parameter("calibration_transform_key", "")
        self.declare_parameter("calibration_parent_frame", "")
        self.declare_parameter("calibration_child_frame", "")

        self.tracking_base_frame = str(self.get_parameter("tracking_base_frame").value)
        self.tracking_marker_frame = str(self.get_parameter("tracking_marker_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.robot_effector_frame = str(self.get_parameter("robot_effector_frame").value)
        calibration_type = str(self.get_parameter("calibration_type").value)
        self.camera_calibration_file = str(self.get_parameter("camera_calibration_file").value)
        self.calibration_transform_key = str(self.get_parameter("calibration_transform_key").value)
        self.calibration_parent_frame = str(self.get_parameter("calibration_parent_frame").value)
        self.calibration_child_frame = str(self.get_parameter("calibration_child_frame").value)

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
        self.tracking_base_to_marker_samples = list()

        self.get_logger().info("Hand-Eye Calibration Node initialized — purely reading TFs, not publishing anything")

    def capture_point_service_callback(self, request, response):
        robot_base_to_effector, tracking_base_to_marker = self.retrieve_transforms()
        self.get_logger().info("robot: " + tf_to_string(robot_base_to_effector))
        self.get_logger().info("tracking: " + tf_to_string(tracking_base_to_marker))
        self.robot_base_to_effector_samples.append(robot_base_to_effector)
        self.tracking_base_to_marker_samples.append(tracking_base_to_marker)
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
            tracking_base_to_marker = self.tf_buffer.lookup_transform(
                self.tracking_base_frame, self.tracking_marker_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().error("Could not get transforms")
            self.get_logger().error(str(ex))
            exit(1)
        return robot_base_to_effector.transform, tracking_base_to_marker.transform

    def calibrate(self) -> Transform | None:
        if len(self.robot_base_to_effector_samples) < 4:
            return None
        self.get_logger().info("Estimating calibration...")
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            return CalibrationBackend.calibrate_eye_in_hand(
                self.robot_base_to_effector_samples, self.tracking_base_to_marker_samples
            )
        if self.calibration_type == CalibrationType.EYE_ON_BASE:
            return CalibrationBackend.calibrate_eye_on_base(
                self.robot_base_to_effector_samples, self.tracking_base_to_marker_samples
            )
        return None

    def resolve_transform_mapping(self) -> tuple[str, str, str] | None:
        key = self.calibration_transform_key.strip()
        parent = self.calibration_parent_frame.strip()
        child = self.calibration_child_frame.strip()

        if key and parent and child:
            return key, parent, child

        if self.tracking_base_frame == "camera_robot":
            return "camera_robot", "fr3_link8", "camera_robot"
        if self.tracking_base_frame == "camera_top":
            return "camera_fixed", "base", "camera_top"
        return None

    def resolve_calibration_file(self) -> Path:
        configured_path = self.camera_calibration_file.strip()
        if configured_path:
            return Path(configured_path).expanduser().resolve()

        if get_package_share_directory is not None:
            try:
                return (
                    Path(get_package_share_directory("sbtc_cv")) / "config" / "calibrated_camera_transforms.yaml"
                ).resolve()
            except Exception:
                pass

        fallback_source_path = Path("/workspace/ros2/src/sbtc-ros2-cv/sbtc_cv/config/calibrated_camera_transforms.yaml")
        if fallback_source_path.exists():
            return fallback_source_path.resolve()

        return (Path.cwd() / "calibrated_camera_transforms.yaml").resolve()

    def persist_camera_calibration(self, calibration_transform: Transform) -> str:
        mapping = self.resolve_transform_mapping()
        if mapping is None:
            return "Calibration result computed; no camera calibration mapping configured, skipping YAML update."

        transform_key, parent_frame, child_frame = mapping
        output_path = self.resolve_calibration_file().resolve()
        source_path = Path(
            "/workspace/ros2/src/sbtc-ros2-cv/sbtc_cv/config/calibrated_camera_transforms.yaml"
        ).resolve()

        output_paths: list[Path] = []
        for candidate_path in (output_path, source_path):
            if candidate_path not in output_paths:
                output_paths.append(candidate_path)

        for path in output_paths:
            path.parent.mkdir(parents=True, exist_ok=True)

        calibration_data = {}
        for path in output_paths:
            if path.exists():
                with path.open("r", encoding="utf-8") as calibration_file:
                    calibration_data = yaml.safe_load(calibration_file) or {}
                break

        static_transforms = calibration_data.setdefault("static_transforms", {})
        transform_entry = static_transforms.setdefault(transform_key, {})

        translation, rotation = tf_to_pos_quat(calibration_transform)
        transform_entry["translation"] = [f"{value:.4f}" for value in translation]
        transform_entry["rotation_xyzw"] = [f"{value:.4f}" for value in rotation]
        transform_entry["parent_frame"] = parent_frame
        transform_entry["child_frame"] = child_frame

        backup_paths = []
        for path in output_paths:
            if path.exists():
                backup_path = Path(str(path) + ".old")
                shutil.copy2(path, backup_path)
                backup_paths.append(str(backup_path))

            with path.open("w", encoding="utf-8") as calibration_file:
                yaml.safe_dump(calibration_data, calibration_file, sort_keys=False)

        updated_paths_str = ", ".join(str(path) for path in output_paths)
        if backup_paths:
            return (
                f"Updated {updated_paths_str} -> static_transforms.{transform_key} (backup: {', '.join(backup_paths)})"
            )

        return f"Updated {updated_paths_str} -> static_transforms.{transform_key}"

    def generate_response_message(self, cal: Transform | None, save_message: str = "") -> str:
        if cal is None:
            return (
                f"Measurement taken ({len(self.robot_base_to_effector_samples)}/4 minimum). Not enough samples yet..."
            )

        calibration_message = ""
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            calibration_message = (
                f"Eye-in-Hand result — {self.robot_effector_frame} -> {self.tracking_base_frame}:\n"
                f"{tf_to_string(cal)}\n"
            )
        elif self.calibration_type == CalibrationType.EYE_ON_BASE:
            calibration_message = (
                f"Eye-on-Base result — {self.robot_base_frame} -> {self.tracking_base_frame}:\n{tf_to_string(cal)}\n"
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
