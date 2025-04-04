import rclpy
from geometry_msgs.msg import Transform
from scipy.spatial.transform import Rotation as Rot
import numpy as np
from typing import List, Tuple, Sequence


def tf_to_pos_quat(tf_message: Transform) -> List[float]:
    tr = tf_message.translation
    qt = tf_message.rotation
    pos = [tr.x, tr.y, tr.z]
    quat = [qt.x, qt.y, qt.z, qt.w]
    return pos, quat


def pos_quat_to_tf(pos: Sequence[float], quat: Sequence[float]) -> Transform:
    """Converts position and quaternion into a geometry_msgs/Transform message.
    
    `pos` should be [tx, ty, tz], and `quat` should be [qx, qy, qz, qw].
    Frame IDs are not set in this function.
    """
    if isinstance(pos, np.ndarray):
        pos = pos.tolist()
    if isinstance(quat, np.ndarray):
        quat = quat.tolist()

    tf = Transform()
    tf.translation.x = pos[0]
    tf.translation.y = pos[1]
    tf.translation.z = pos[2]
    tf.rotation.x = quat[0]
    tf.rotation.y = quat[1]
    tf.rotation.z = quat[2]
    tf.rotation.w = quat[3]

    return tf


def tf_to_string(tf_message: Transform, child_frame_id: str = "", frame_id: str = "") -> str:
    pos, quat = tf_to_pos_quat(tf_message)
    tf_string = "tx, ty, tz, qx, qy, qz, qw: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]" % (
        pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]
    )
    out = f"{child_frame_id} -> {frame_id}:\n\t{tf_string}"
    return out


def inverse_tf(transform: Transform) -> Transform:
    pos, quat = tf_to_pos_quat(transform)
    translation = np.array(pos)
    quaternion = np.array(quat)
    rotation = Rot.from_quat(quaternion)  # Quaternion in (x, y, z, w)
    inv_rotation = rotation.inv()
    inv_translation = -inv_rotation.apply(translation)
    inv_quaternion = inv_rotation.as_quat()  # Returns in (x, y, z, w)
    inv_transform = np.concatenate([inv_translation, inv_quaternion])
    tf = pos_quat_to_tf(inv_transform)
    return tf


def transform_to_R_t(pos: List[float], quat: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    """
    transform = [tx, ty, tz, qx, qy, qz, qw]
    """
    tr = np.array(pos)
    rot = Rot.from_quat(quat).as_matrix()
    return rot, tr


def invert_rot_pos(R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Given a rotation matrix R and a translation vector t,
    return the inverse transformation (R.T, -R.T @ t).
    """
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv
