"""
Conversion utilities for transforming between measurement types.
"""
import numpy as np
import scipy.spatial.transform
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..types.measurements import OdometryMeasurement3D, OdometryMeasurement2D, RangeMeasurement


def convert_odom3_to_odom2(odom3: "OdometryMeasurement3D") -> "OdometryMeasurement2D":
    """
    Converts a 3D odometry measurement to a 2D odometry measurement by projecting 
    the translation onto the XY plane and extracting the yaw (psi) from the quaternion.

    Args:
        odom3: The 3D odometry measurement to convert.

    Returns:
        The converted 2D odometry measurement.
    """
    from ..types.measurements import OdometryMeasurement2D
    from ..types.covariance import RelPoseCovar3
    
    x, y, _ = odom3.relative_translation
    quat = odom3.relative_rotation
    
    # Convert quaternion to Euler angles (roll, pitch, yaw)
    euler = scipy.spatial.transform.Rotation.from_quat(quat).as_euler("xyz")
    psi = euler[2]  # Yaw angle

    # Create a new RelPoseCovar3 from the RelPoseCovar6
    covar6 = odom3.covariance
    covar3 = RelPoseCovar3(
        relative_pose_sigma_x=covar6.relative_pose_sigma_x,
        relative_pose_sigma_y=covar6.relative_pose_sigma_y,
        relative_pose_sigma_psi=covar6.relative_pose_sigma_psi,
        relative_pose_rho_xy=covar6.relative_pose_rho_xy,
        relative_pose_rho_xpsi=covar6.relative_pose_rho_xpsi,
        relative_pose_rho_ypsi=covar6.relative_pose_rho_ypsi,
    )

    return OdometryMeasurement2D(
        key_pair=odom3.key_pair,
        relative_translation=(x, y),
        relative_rotation=psi,
        covariance=covar3,
    )


def project_range_to_2d(range_measurement: "RangeMeasurement") -> "RangeMeasurement":
    """
    Projects a range measurement to 2D by calculating the horizontal distance 
    in the xy plane, accounting for the vertical distance between depth measurements.

    Args:
        range_measurement: The range measurement with depth information.

    Returns:
        A new range measurement with the horizontal distance and no depth info.
        
    Raises:
        ValueError: If depths are not provided.
    """
    from ..types.measurements import RangeMeasurement
    
    if range_measurement.depth1 is None or range_measurement.depth2 is None:
        raise ValueError(
            "Both depths must be provided to project the range measurement to 2D."
        )

    vertical_distance = abs(range_measurement.depth1 - range_measurement.depth2)
    horizontal_distance = (range_measurement.distance**2 - vertical_distance**2) ** 0.5

    assert (
        horizontal_distance > 0
    ), f"Horizontal distance should be positive, got {horizontal_distance} for range {range_measurement}"

    return RangeMeasurement(
        key_pair=range_measurement.key_pair,
        distance=horizontal_distance,
        variance=range_measurement.variance,
        depth1=None,  # Depths are not needed in 2D projection
        depth2=None,
    )
