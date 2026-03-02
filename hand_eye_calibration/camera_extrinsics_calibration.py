#!/usr/bin/env python3
"""
Collect poses and perform calibration
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Transform
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from typing import Tuple
import enum

from .calibration_backend import CalibrationBackend 
from .utilities import tf_to_string

class CalibrationType(enum.Enum):
    EYE_ON_BASE = enum.auto()
    EYE_IN_HAND = enum.auto()


class CameraExtrinsicsCalibration(Node):
    
    def __init__(self):
        mname = "hand_eye_calibration"
        super().__init__(mname)

        self.declare_parameter('tracking_base_frame', "")
        self.declare_parameter('tracking_marker_frame', "")
        self.declare_parameter('robot_base_frame', "")
        self.declare_parameter('robot_effector_frame', "")
        self.declare_parameter('calibration_type', "eye-on-base")

        self.tracking_base_frame = str(self.get_parameter('tracking_base_frame').value)
        self.tracking_marker_frame = str(self.get_parameter('tracking_marker_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.robot_effector_frame = str(self.get_parameter('robot_effector_frame').value)
        calibration_type = str(self.get_parameter('calibration_type').value)
        
        if calibration_type == "eye-in-hand":
            self.calibration_type = CalibrationType.EYE_IN_HAND
        elif calibration_type == "eye-on-base":
            self.calibration_type = CalibrationType.EYE_ON_BASE
        else:
            self.get_logger().error(f"Unknown calibration type: {calibration_type}")
            exit(1) 

        self.capture_point_service = self.create_service(
            Trigger, 
            mname + "/capture_point", 
            self.capture_point_service_callback)

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
        response.success = True
        response.message = self.generate_response_message(cal)
        self.get_logger().info(response.message)
        return response


    def retrieve_transforms(self) -> Tuple[Transform, Transform]:
        try:                
            robot_base_to_effector = self.tf_buffer.lookup_transform(
                self.robot_base_frame, 
                self.robot_effector_frame,
                Time())
            tracking_base_to_marker = self.tf_buffer.lookup_transform(
                self.tracking_base_frame,
                self.tracking_marker_frame,
                Time())
        except TransformException as ex:
            self.get_logger().error("Could not get transforms")
            self.get_logger().error(str(ex))
            exit(1)
        return robot_base_to_effector.transform, tracking_base_to_marker.transform


    def calibrate(self):
        if len(self.robot_base_to_effector_samples) < 4:
            return None
        self.get_logger().info("Estimating calibration...")
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            return CalibrationBackend.calibrate_eye_in_hand(
                self.robot_base_to_effector_samples, 
                self.tracking_base_to_marker_samples)
        elif self.calibration_type == CalibrationType.EYE_ON_BASE:
            return CalibrationBackend.calibrate_eye_on_base(
                self.robot_base_to_effector_samples, 
                self.tracking_base_to_marker_samples)


    def generate_response_message(self, cal: Transform):
        if cal is None:
            return f"Measurement taken ({len(self.robot_base_to_effector_samples)}/4 minimum). Not enough samples yet..."
        
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            return (
                f"Eye-in-Hand result — {self.robot_effector_frame} -> {self.tracking_base_frame}:\n"
                f"{tf_to_string(cal)}\n"
            )
        elif self.calibration_type == CalibrationType.EYE_ON_BASE:
            return (
                f"Eye-on-Base result — {self.robot_base_frame} -> {self.tracking_base_frame}:\n"
                f"{tf_to_string(cal)}\n"
            )


def main():
    rclpy.init()
    node = CameraExtrinsicsCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()