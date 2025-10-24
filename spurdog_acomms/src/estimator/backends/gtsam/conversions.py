"""
Conversion utilities between internal types and GTSAM types.

This module provides functions to convert between our custom types (Key, Pose2D, Pose3D, etc.)
and GTSAM's native types (symbol, Pose2, Pose3, etc.).
"""
import numpy as np
import rospy  # type: ignore

from gtsam.gtsam import Pose2, Pose3, Rot3, symbol

from typing import Union

from ...types.key import Key
from ...types.measurements import OdometryMeasurement, OdometryMeasurement2D, OdometryMeasurement3D
from ...utils.validation import _check_transformation_matrix
from ...utils.transformations import (
    get_theta_from_transformation_matrix,
    get_translation_from_transformation_matrix,
)


def get_gtsam_symbol_from_key(key: Key) -> int:
    """
    Convert a Key to a GTSAM symbol.
    
    Args:
        key: The Key to convert.
        
    Returns:
        The GTSAM symbol as an integer.
    """
    assert isinstance(key, Key), "Key must be of type Key"
    return symbol(key.char, key.index)


def get_pose2_from_matrix(pose_matrix: np.ndarray) -> Pose2:
    """
    Convert a 2D transformation matrix to a GTSAM Pose2.
    
    Args:
        pose_matrix: 3x3 homogeneous transformation matrix.
        
    Returns:
        GTSAM Pose2 object.
    """
    _check_transformation_matrix(pose_matrix, dim=2)
    theta = get_theta_from_transformation_matrix(pose_matrix)
    trans = get_translation_from_transformation_matrix(pose_matrix)
    return Pose2(trans[0], trans[1], theta)


def get_pose3_from_matrix(pose_matrix: np.ndarray) -> Pose3:
    """
    Convert a 3D transformation matrix to a GTSAM Pose3.
    
    Args:
        pose_matrix: 4x4 homogeneous transformation matrix.
        
    Returns:
        GTSAM Pose3 object.
    """
    _check_transformation_matrix(pose_matrix, dim=3)
    rot_matrix = pose_matrix[:3, :3]
    tx, ty, tz = pose_matrix[:3, 3]
    return Pose3(Rot3(rot_matrix), np.array([tx, ty, tz]))  # type: ignore


def get_relative_pose_from_odom_measurement(
    odom_measurement: OdometryMeasurement,
) -> Union[Pose2, Pose3]:
    """
    Extract the relative pose from an odometry measurement.
    
    Args:
        odom_measurement: The odometry measurement.
        
    Returns:
        GTSAM Pose2 or Pose3 depending on the measurement dimension.
        
    Raises:
        ValueError: If the odometry measurement type is unknown.
    """
    if isinstance(odom_measurement, OdometryMeasurement2D):
        return Pose2(odom_measurement.x, odom_measurement.y, odom_measurement.psi)
    elif isinstance(odom_measurement, OdometryMeasurement3D):
        return Pose3(
            Rot3(odom_measurement.rotation_matrix), odom_measurement.translation_vector
        )
    else:
        err = f"Unknown odometry measurement type: {type(odom_measurement)}"
        rospy.logerr(err)
        raise ValueError(err)
