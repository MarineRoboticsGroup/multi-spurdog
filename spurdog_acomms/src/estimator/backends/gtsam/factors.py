"""
Factor creation utilities for GTSAM.

This module provides functions to create various GTSAM factors from our internal
measurement types.
"""
import numpy as np
import rospy  # type: ignore

from gtsam.gtsam import (
    BetweenFactorPose2,
    BetweenFactorPose3,
    Rot2,
    Rot3,
    noiseModel,
)

from typing import Union

from ...custom_factors.SESyncFactor2d import RelativePose2dFactor
from ...custom_factors.SESyncFactor3d import RelativePose3dFactor
from ...types.measurements import OdometryMeasurement, OdometryMeasurement2D, OdometryMeasurement3D

from .conversions import get_relative_pose_from_odom_measurement

# Valid factor model types
VALID_BETWEEN_FACTOR_MODELS = ["SESync", "between"]


def _get_between_factor(
    odom_measurement: OdometryMeasurement, i_sym: int, j_sym: int
) -> Union[BetweenFactorPose2, BetweenFactorPose3]:
    """
    Create a standard GTSAM BetweenFactor from an odometry measurement.
    
    Args:
        odom_measurement: The odometry measurement.
        i_sym: The symbol for the first pose.
        j_sym: The symbol for the second pose.
        
    Returns:
        BetweenFactorPose2 or BetweenFactorPose3.
        
    Raises:
        ValueError: If the measurement type is unknown.
    """
    odom_covar_diags = np.diag(odom_measurement.covariance.covariance_matrix)
    odom_noise = noiseModel.Diagonal.Variances(
        odom_covar_diags  # type: ignore
    )
    rospy.logwarn(f"Odom noise variances for {i_sym}->{j_sym}: {odom_covar_diags}")

    rel_pose = get_relative_pose_from_odom_measurement(odom_measurement)
    
    if isinstance(odom_measurement, OdometryMeasurement2D):
        odom_factor = BetweenFactorPose2(i_sym, j_sym, rel_pose, odom_noise)  # type: ignore
    elif isinstance(odom_measurement, OdometryMeasurement3D):
        odom_factor = BetweenFactorPose3(i_sym, j_sym, rel_pose, odom_noise)  # type: ignore
    else:
        raise ValueError(f"Unknown measurement type: {type(odom_measurement)}")
    
    return odom_factor


def _get_between_se_sync_factor(
    odom_measurement: OdometryMeasurement, i_sym: int, j_sym: int
) -> Union[RelativePose2dFactor, RelativePose3dFactor]:
    """
    Create a SESync-style RelativePoseFactor from an odometry measurement.
    
    Args:
        odom_measurement: The odometry measurement.
        i_sym: The symbol for the first pose.
        j_sym: The symbol for the second pose.
        
    Returns:
        RelativePose2dFactor or RelativePose3dFactor.
        
    Raises:
        ValueError: If the measurement type is unknown.
    """
    if isinstance(odom_measurement, OdometryMeasurement2D):
        rot2_measure = Rot2(odom_measurement.relative_rotation)
        odom_factor = RelativePose2dFactor(
            i_sym,
            j_sym,
            rot2_measure,
            odom_measurement.translation_vector,
            odom_measurement.rotation_precision,
            odom_measurement.translation_precision,
        )
    elif isinstance(odom_measurement, OdometryMeasurement3D):
        rot3_measure = Rot3(odom_measurement.rotation_matrix)
        odom_factor = RelativePose3dFactor(
            i_sym,
            j_sym,
            rot3_measure,
            odom_measurement.translation_vector,
            odom_measurement.rotation_precision,
            odom_measurement.translation_precision,
        )
    else:
        raise ValueError(f"Unknown measurement type: {type(odom_measurement)}")
    
    return odom_factor


def get_pose_to_pose_factor(
    odom_measurement: OdometryMeasurement,
    i_symbol: int,
    j_symbol: int,
    factor_model: str = "between",
) -> Union[
    BetweenFactorPose2, BetweenFactorPose3, RelativePose2dFactor, RelativePose3dFactor
]:
    """
    Create a pose-to-pose factor from an odometry measurement.
    
    This is the main entry point for creating odometry factors. It supports
    both standard GTSAM BetweenFactors and SESync-style RelativePoseFactors.
    
    Args:
        odom_measurement: The odometry measurement.
        i_symbol: The symbol for the first pose.
        j_symbol: The symbol for the second pose.
        factor_model: The factor model to use ("between" or "SESync").
        
    Returns:
        The appropriate factor type based on the factor_model.
        
    Raises:
        AssertionError: If factor_model is not valid.
        ValueError: If factor_model is not recognized.
    """
    assert (
        factor_model in VALID_BETWEEN_FACTOR_MODELS
    ), f"Invalid factor model: {factor_model}. Valid models are: {VALID_BETWEEN_FACTOR_MODELS}"
    
    if factor_model == "SESync":
        odom_factor = _get_between_se_sync_factor(odom_measurement, i_symbol, j_symbol)
    elif factor_model == "between":
        odom_factor = _get_between_factor(odom_measurement, i_symbol, j_symbol)
    else:
        raise ValueError(f"Unknown factor model: {factor_model}")
    
    return odom_factor
