"""
ROS integration layer for the estimator package.

This module provides ROS-specific functionality including:
- Message conversion between ROS messages and internal types
- ROS message handlers for pose and range factors
- ROS node management and wrapper classes
- Visualization utilities for RViz
- Publisher management for pose estimates

This layer isolates ROS dependencies from the core estimation logic,
making the estimator backend-agnostic and testable without ROS.
"""

# ROS Manager - main entry point for ROS integration
from .manager import ROSEstimatorManager

# Message conversion utilities
from .message_converters import (
    pose_factor_to_pose_prior,
    pose_factor_to_odometry,
    range_factor_to_range_measurement,
    extract_covariance_from_pose_msg,
)

# Message handlers
from .handlers import (
    handle_pose_factor,
    handle_range_factor,
)

# Visualization utilities
from .visualization import (
    make_marker,
    create_node_and_label_markers,
    create_edge_marker,
    create_range_edge_marker,
)

# Publisher utilities
from .publishers import (
    publish_pose_msgs,
)

__all__ = [
    "ROSEstimatorManager",
    "pose_factor_to_pose_prior",
    "pose_factor_to_odometry",
    "range_factor_to_range_measurement",
    "extract_covariance_from_pose_msg",
    "handle_pose_factor",
    "handle_range_factor",
    "make_marker",
    "create_node_and_label_markers",
    "create_edge_marker",
    "create_range_edge_marker",
    "publish_pose_msgs",
]
