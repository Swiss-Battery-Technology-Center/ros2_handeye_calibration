#!/usr/bin/env python3
"""
Collect poses and perform calibration
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Duration, Time
from geometry_msgs.msg import TransformStamped, Transform
from std_srvs.srv import Trigger
from tf2_ros import TransformException, StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from typing import Tuple
import enum

from .calibration_backend import CalibrationBackend 
from .utilities import tf_to_string, inverse_tf

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
        self.declare_parameter('calibration_type', "eye-on-base")  # options: "eye-in-hand" or "eye-on-base"

        self.tracking_base_frame = str(self.get_parameter('tracking_base_frame').value)
        self.tracking_marker_frame = str(self.get_parameter('tracking_marker_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.robot_effector_frame = str(self.get_parameter('robot_effector_frame').value)
        self.calibration_type = str(self.get_parameter('calibration_type').value)
        
        if self.calibration_type == "eye-in-hand":
            self.calibration_type = CalibrationType.EYE_IN_HAND
        elif self.calibration_type == "eye-on-base":
            self.calibration_type = CalibrationType.EYE_ON_BASE
        else:
            self.get_logger().error(f"Unknown calibration type: {self.calibration_type}")
            exit(1) 

        self.capture_point_service_name = mname + "/capture_point"
        self.capture_point_service = self.create_service(
            Trigger, 
            self.capture_point_service_name, 
            self.capture_point_service_callback)

        self.tf_buffer = Buffer()
        self._listener = TransformListener(self.tf_buffer, self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.publish_identity_tracking_base()

        self.robot_base_to_effector_samples = list()
        self.tracking_base_to_marker_samples = list()

        self.get_logger().info("Camera Extrinsics Calibration Node initialized")

    def capture_point_service_callback(self, request, response):
        robot_base_to_effector, tracking_base_to_marker = self.retrieve_transforms()
        self.get_logger().info("robot: " + tf_to_string(robot_base_to_effector))
        self.get_logger().info("tracking: " + tf_to_string(tracking_base_to_marker))
        self.robot_base_to_effector_samples.append(robot_base_to_effector)
        self.tracking_base_to_marker_samples.append(tracking_base_to_marker)
        cal = self.calibrate()
        response.success = True
        msg = self.generate_response_message(cal)
        response.message = msg
        self.get_logger().info(response.message)
        if cal is not None:
            self.publish_static_transform(cal)
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
            self.get_logger().info("Not enough samples yet...")
            return None
        else:
            self.get_logger().info("Estimating calibration...")
            if self.calibration_type == CalibrationType.EYE_IN_HAND:
                cal = CalibrationBackend.calibrate_eye_in_hand(
                    self.robot_base_to_effector_samples, 
                    self.tracking_base_to_marker_samples)
            elif self.calibration_type == CalibrationType.EYE_ON_BASE:
                cal = CalibrationBackend.calibrate_eye_on_base(
                    self.robot_base_to_effector_samples, 
                    self.tracking_base_to_marker_samples)
            return cal


    def generate_response_message(self, cal: Transform):
        if cal is None:
            return "Measurement taken. Not enough samples yet..."
        
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            msg = (
                "Eye-in-Hand Calibration:\n"
                f"Estimated transformation from robot effector ({self.robot_effector_frame}) "
                f"to tracking base ({self.tracking_base_frame}): {tf_to_string(cal)}\n"
            )
        elif self.calibration_type == CalibrationType.EYE_ON_BASE:
            msg = (
                "Eye-to-Hand Calibration:\n"
                f"Estimated transformation from robot base ({self.robot_base_frame}) "
                f"to tracking base ({self.tracking_base_frame}): {tf_to_string(cal)}\n"
            )
        return msg
    
    
    def publish_static_transform(self, transform: Transform):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        if self.calibration_type == CalibrationType.EYE_IN_HAND:
            static_transform.header.frame_id = self.robot_effector_frame
            static_transform.child_frame_id = self.tracking_base_frame
        elif self.calibration_type == CalibrationType.EYE_ON_BASE:
            static_transform.header.frame_id = self.robot_base_frame
            static_transform.child_frame_id = self.tracking_base_frame
        static_transform.transform = transform
        self._static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Static transform published")



    def publish_identity_tracking_base(self):
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()

        # For eye-on-base, tracking base should be anchored to robot base (initially identity)
        if self.calibration_type == CalibrationType.EYE_ON_BASE:
            st.header.frame_id = self.robot_base_frame
            st.child_frame_id = self.tracking_base_frame
        else:
            # For eye-in-hand, tracking base is anchored to effector (initially identity)
            st.header.frame_id = self.robot_effector_frame
            st.child_frame_id = self.tracking_base_frame

        st.transform.translation.x = 0.0
        st.transform.translation.y = 0.0
        st.transform.translation.z = 0.0
        st.transform.rotation.x = 0.0
        st.transform.rotation.y = 0.0
        st.transform.rotation.z = 0.0
        st.transform.rotation.w = 1.0

        self._static_tf_broadcaster.sendTransform(st)
        self.get_logger().info(
            f"Published identity static TF: {st.header.frame_id} -> {st.child_frame_id}"
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
