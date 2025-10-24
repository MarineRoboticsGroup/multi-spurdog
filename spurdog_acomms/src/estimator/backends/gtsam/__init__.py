"""
GTSAM backend for the estimator.

This module provides GTSAM-specific implementations for factor graph optimization.
"""

from .estimator import GtsamEstimator
from .conversions import (
    get_gtsam_symbol_from_key,
    get_pose2_from_matrix,
    get_pose3_from_matrix,
    get_relative_pose_from_odom_measurement,
)
from .factors import get_pose_to_pose_factor
from .solvers import (
    solve_with_isam2,
    solve_with_levenberg_marquardt,
    diagnose_optimization_problem,
    try_gauss_newton_probe,
)

__all__ = [
    "GtsamEstimator",
    "get_gtsam_symbol_from_key",
    "get_pose2_from_matrix",
    "get_pose3_from_matrix",
    "get_relative_pose_from_odom_measurement",
    "get_pose_to_pose_factor",
    "solve_with_isam2",
    "solve_with_levenberg_marquardt",
    "diagnose_optimization_problem",
    "try_gauss_newton_probe",
]
