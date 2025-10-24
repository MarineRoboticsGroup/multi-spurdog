"""
ROS message conversion utilities.

Converts between ROS messages and internal estimator types.
"""

import numpy as np
from typing import Tuple, Optional, Union

from ..types.key import Key, KeyPair
from ..types.measurements import OdometryMeasurement3D, OdometryMeasurement2D, RangeMeasurement
from ..types.variables import Pose3D
from ..types.covariance import RelPoseCovar6, get_diag_relpose_covar
from ..utils.conversions import convert_odom3_to_odom2

from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped

import rospy
import math


def extract_covariance_from_pose_msg(
    msg: PoseFactorStamped,
) -> Tuple[np.ndarray, RelPoseCovar6]:
    """
    Extract covariance matrix and RelPoseCovar6 from a PoseFactorStamped message.
    
    Args:
        msg: PoseFactorStamped message containing covariance data
        
    Returns:
        Tuple of (6x6 covariance matrix, RelPoseCovar6 object)
        
    Raises:
        ValueError: If covariance extraction fails
    """
    # ROS PoseWithCovariance uses a 6x6 covariance stored in row-major order
    cov_list = list(msg.pose.covariance)
    try:
        cov_mat = np.array(cov_list).reshape((6, 6))
    except Exception:
        cov_mat = np.zeros((6, 6))

    # Extract standard deviations and correlations for RelPoseCovar6
    sx = float(np.sqrt(max(0.0, cov_mat[0, 0])))
    sy = float(np.sqrt(max(0.0, cov_mat[1, 1])))
    sz = float(np.sqrt(max(0.0, cov_mat[2, 2])))
    spsi = float(np.sqrt(max(0.0, cov_mat[3, 3])))

    # Guard against zero sigmas
    if sx == 0.0:
        sx = 1e-6
    if sy == 0.0:
        sy = 1e-6
    if spsi == 0.0:
        spsi = 1e-6

    rho_xy = float(cov_mat[0, 1] / (sx * sy)) if (sx * sy) != 0 else 0.0
    rho_xpsi = float(cov_mat[0, 3] / (sx * spsi)) if (sx * spsi) != 0 else 0.0
    rho_ypsi = float(cov_mat[1, 3] / (sy * spsi)) if (sy * spsi) != 0 else 0.0

    try:
        covar = RelPoseCovar6(
            relative_pose_sigma_x=sx,
            relative_pose_sigma_y=sy,
            relative_pose_sigma_z=sz,
            relative_pose_sigma_psi=spsi,
            relative_pose_rho_xy=rho_xy,
            relative_pose_rho_xpsi=rho_xpsi,
            relative_pose_rho_ypsi=rho_ypsi,
        )
    except Exception as e:
        # Fallback to default diagonal covariance for first pose
        if msg.key1 == "A0" and msg.key2 == "A1":
            covar = get_diag_relpose_covar(np.array([0.1] * 3 + [0.05] * 3))
            assert isinstance(covar, RelPoseCovar6)
            rospy.logwarn(
                f"Using default diagonal RelPoseCovar6 for odometry measurement between {msg.key1} and {msg.key2} due to error: {e}"
            )
        else:
            rospy.logerr(
                f"Failed to create RelPoseCovar6 for odometry measurement between {msg.key1} and {msg.key2}: {e}"
            )
            raise e

    return cov_mat, covar


def pose_factor_to_pose_prior(msg: PoseFactorStamped) -> Pose3D:
    """
    Convert a PoseFactorStamped message to a Pose3D prior.
    
    This is used when key1 == key2, indicating a pose prior factor.
    
    Args:
        msg: PoseFactorStamped message with identical keys
        
    Returns:
        Pose3D object representing the prior
        
    Raises:
        ValueError: If keys are not identical or pose data is invalid
    """
    if msg.key1 != msg.key2:
        raise ValueError(
            f"Pose prior must have identical keys, got {msg.key1} and {msg.key2}"
        )

    # Extract position and orientation
    position = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    )
    orientation = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    )

    # Check for zero/default values and warn
    position_zeros = np.allclose(position, 0.0)
    rotation_default = np.allclose(orientation, (0.0, 0.0, 0.0, 1.0))
    if position_zeros and rotation_default:
        rospy.logwarn(
            f"Pose prior for key {msg.key1} has zero position and default orientation, skipping prior addition."
        )
        raise ValueError("Invalid pose prior: all zeros")
    elif position_zeros:
        rospy.logwarn(
            f"Pose prior for key {msg.key1} has zero position. Allowing prior addition, but this may be unintended."
        )
    elif rotation_default:
        rospy.logwarn(
            f"Pose prior for key {msg.key1} has default orientation. Allowing prior addition, but this may be unintended."
        )

    # Extract covariance
    assert (
        len(msg.pose.covariance) == 36
    ), f"Pose covariance must have 36 elements, got {len(msg.pose.covariance)}"
    
    pose_covariance = np.array(msg.pose.covariance).reshape((6, 6))
    
    # Use default covariance if all zeros
    if np.allclose(pose_covariance, 0.0):
        pose_covariance[:3, :3] = np.eye(3) * 1e-2
        pose_covariance[3:, 3:] = np.eye(3) * 1e-3

    new_pose = Pose3D(
        key=msg.key1,
        position=position,
        orientation=orientation,
        marginal_covariance=pose_covariance,
    )
    
    return new_pose


def pose_factor_to_odometry(
    msg: PoseFactorStamped, dimension: int = 3
# ) -> Optional[OdometryMeasurement3D | OdometryMeasurement2D]:
) -> Optional[Union[OdometryMeasurement3D, OdometryMeasurement2D]]:
    """
    Convert a PoseFactorStamped message to an OdometryMeasurement.
    
    This is used when key1 != key2, indicating a relative pose measurement.
    
    Args:
        msg: PoseFactorStamped message with different keys
        dimension: State space dimension (2 or 3)
        
    Returns:
        OdometryMeasurement3D or OdometryMeasurement2D depending on dimension
        None if conversion fails
        
    Raises:
        ValueError: If keys are identical or invalid
    """
    if msg.key1 == msg.key2:
        raise ValueError(
            f"Odometry measurement must have different keys, got {msg.key1} and {msg.key2}"
        )

    key1, key2 = Key(msg.key1), Key(msg.key2)
    agent1, agent2 = key1.char, key2.char
    
    assert (
        agent1 == agent2
    ), f"Odometry measurement: agents should be the same, got {agent1} and {agent2}"
    assert not (
        key1.is_landmark or key2.is_landmark
    ), f"Odometry measurement keys cannot be landmarks, got {key1} and {key2}"

    idx1, idx2 = key1.index, key2.index
    assert (
        idx2 == idx1 + 1
    ), f"Odometry measurement keys must be consecutive, got {key1} and {key2}"

    # Extract translation
    tx = msg.pose.pose.position.x
    ty = msg.pose.pose.position.y
    tz = msg.pose.pose.position.z
    
    # Extract rotation (quaternion)
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    # Extract covariance
    _, covar = extract_covariance_from_pose_msg(msg)

    odom = OdometryMeasurement3D(
        key_pair=KeyPair(key1, key2),
        relative_translation=(tx, ty, tz),
        relative_rotation=(qx, qy, qz, qw),
        covariance=covar,
    )

    if dimension == 2:
        try:
            return convert_odom3_to_odom2(odom)
        except Exception as e:
            rospy.logerr(f"Failed to convert 3D odometry to 2D: {e}")
            return None

    return odom


def range_factor_to_range_measurement(msg: RangeFactorStamped) -> RangeMeasurement:
    """
    Convert a RangeFactorStamped message to a RangeMeasurement.
    
    Args:
        msg: RangeFactorStamped message
        
    Returns:
        RangeMeasurement object
        
    Raises:
        ValueError: If keys are identical
    """
    if msg.key1 == msg.key2:
        raise ValueError(
            f"Range measurement must have different keys, got {msg.key1} and {msg.key2}"
        )

    kp = KeyPair(Key(msg.key1), Key(msg.key2))
    
    # Treat range_sigma as standard deviation
    try:
        variance = float(msg.range_sigma) ** 2
    except Exception:
        # Fallback: use 1% of range as variance
        variance = (
            float(msg.meas_range) * 0.01 if msg.meas_range != 0.0 else 1.0
        )

    depth1 = None
    depth2 = None
    
    # If depths are finite and not zero, use them
    if (
        math.isfinite(msg.depth1)
        and math.isfinite(msg.depth2)
        and (msg.depth1 != 0.0 or msg.depth2 != 0.0)
    ):
        depth1 = float(msg.depth1)
        depth2 = float(msg.depth2)

    range_meas = RangeMeasurement(
        key_pair=kp,
        distance=float(msg.meas_range),
        variance=variance,
        depth1=depth1,
        depth2=depth2,
    )

    return range_meas
