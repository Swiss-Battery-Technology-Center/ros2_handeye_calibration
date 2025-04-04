import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from typing import List, Tuple
from geometry_msgs.msg import Transform

from .utilities import tf_to_pos_quat, pos_quat_to_tf, invert_rot_pos

"""
Check the OpenCV documentation for the Hand-Eye calibration:
https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
"""


class CalibrationBackend:
    MIN_SAMPLES = 4

    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }


    @staticmethod
    def prepare_samples(
        robot_base_to_effector_samples: List[Transform], 
        camera_to_marker_samples: List[Transform]
    ) -> Tuple[List[np.array], List[np.array], List[np.array], List[np.array]]:

        if len(robot_base_to_effector_samples) < CalibrationBackend.MIN_SAMPLES:
            raise ValueError(f"Not enough samples. Minimum required is {CalibrationBackend.MIN_SAMPLES}.")
        if len(robot_base_to_effector_samples) != len(camera_to_marker_samples):
            raise ValueError("Robot and tracking samples must have the same length.")

        # Prepare data
        base_to_effector_rot = []
        base_to_effector_pos = []
        camera_to_marker_rot = []
        camera_to_marker_pos = []
        
        for robot_base_to_effector in robot_base_to_effector_samples:
            pos, quat = tf_to_pos_quat(robot_base_to_effector)
            base_to_effector_pos.append(np.array(pos))
            base_to_effector_rot.append(Rot.from_quat(quat).as_matrix())
            
        for camera_to_marker in camera_to_marker_samples:
            pos, quat = tf_to_pos_quat(camera_to_marker)
            camera_to_marker_pos.append(np.array(pos))
            camera_to_marker_rot.append(Rot.from_quat(quat).as_matrix())
            
        return base_to_effector_rot, base_to_effector_pos, camera_to_marker_rot, camera_to_marker_pos


    @staticmethod
    def calibrate_eye_on_base(
        robot_base_to_effector_samples: List[Transform],
        camera_to_marker_samples: List[Transform], 
        algorithm: str='Tsai-Lenz'
    ) -> Transform:

        base_to_effector_rot, base_to_effector_pos, camera_to_marker_rot, camera_to_marker_pos = \
            CalibrationBackend.prepare_samples(
                robot_base_to_effector_samples, 
                camera_to_marker_samples
            )
            
        # Inverse for eye-on-base
        effector_to_base_rot, effector_to_base_pos = invert_rot_pos(base_to_effector_rot, base_to_effector_pos)
        
        # Calibrate
        robot_base_to_camera_rot, robot_base_to_camera_tr = cv2.calibrateHandEye(
            effector_to_base_rot, 
            effector_to_base_pos, 
            camera_to_marker_rot,
            camera_to_marker_pos,
            method=CalibrationBackend.AVAILABLE_ALGORITHMS[algorithm]
        )
        
        robot_base_to_camera_rot = np.array(robot_base_to_camera_rot).reshape(3, 3)
        robot_base_to_camera_pos = np.array(robot_base_to_camera_tr).reshape(3)
        robot_base_to_camera_quat = Rot.from_matrix(robot_base_to_camera_rot).as_quat()
        robot_base_to_camera = pos_quat_to_tf(robot_base_to_camera_pos, robot_base_to_camera_quat)

        return robot_base_to_camera     

    
    @staticmethod
    def calibrate_eye_in_hand(
        robot_base_to_effector_samples: List[Transform],
        camera_to_marker_samples: List[Transform], 
        algorithm: str='Tsai-Lenz'
    ) -> Transform:
    
        base_to_effector_rot, base_to_effector_pos, camera_to_marker_rot, camera_to_marker_pos = \
            CalibrationBackend.prepare_samples(
                robot_base_to_effector_samples, 
                camera_to_marker_samples
            )
                    
        effector_to_camera_rot, effector_to_camera_tr= cv2.calibrateHandEye(
            base_to_effector_rot,
            base_to_effector_pos, 
            camera_to_marker_rot,
            camera_to_marker_pos,
            method=CalibrationBackend.AVAILABLE_ALGORITHMS[algorithm]
        )

        effector_to_camera_rot = np.array(effector_to_camera_rot).reshape(3, 3)
        effector_to_camera_pos = np.array(effector_to_camera_tr).reshape(3)
        effector_to_camera_quat = Rot.from_matrix(effector_to_camera_rot).as_quat()
        effector_to_camera = pos_quat_to_tf(effector_to_camera_pos, effector_to_camera_quat)

        return effector_to_camera
  
        