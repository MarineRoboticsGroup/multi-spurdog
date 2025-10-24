"""
ROS message handlers for estimator.

These handlers process incoming ROS messages and update the estimator accordingly.
They contain the business logic for handling pose priors, odometry, and range measurements.
"""

import rospy
from typing import Dict

from ..estimator import Estimator
from ..types.key import Key
from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped

from .message_converters import (
    pose_factor_to_pose_prior,
    pose_factor_to_odometry,
    range_factor_to_range_measurement,
)


def handle_pose_prior(
    estimator: Estimator,
    msg: PoseFactorStamped,
) -> None:
    """
    Handle a pose prior factor message.
    
    Args:
        estimator: The estimator to update
        msg: PoseFactorStamped message with identical keys (prior)
    """
    try:
        new_pose = pose_factor_to_pose_prior(msg)
    except ValueError as e:
        rospy.logwarn(f"Skipping invalid pose prior: {e}")
        return

    # Initialize and add prior
    try:
        rospy.logdebug(f"Initializing pose for key: {msg.key1}")
        estimator.initialize_pose(new_pose)
    except Exception:
        # Initialization may be a no-op for some estimator implementations
        rospy.logdebug(f"initialize_pose failed for {msg.key1}")
    
    try:
        rospy.logdebug(f"Adding pose prior for key: {msg.key1}")
        estimator.add_pose_prior(new_pose)
    except Exception as e:
        rospy.logwarn(f"add_pose_prior failed: {e}")


def handle_odometry_measurement(
    estimator: Estimator,
    msg: PoseFactorStamped,
    most_recent_pose_keys: Dict[str, Key],
    dimension: int = 3,
) -> None:
    """
    Handle an odometry measurement (relative pose between consecutive poses).
    
    Args:
        estimator: The estimator to update
        msg: PoseFactorStamped message with different keys (odometry)
        most_recent_pose_keys: Dict mapping agent names to their most recent pose keys
        dimension: State space dimension (2 or 3)
    """
    try:
        odom = pose_factor_to_odometry(msg, dimension=dimension)
    except (ValueError, AssertionError) as e:
        rospy.logerr(f"Invalid odometry measurement: {e}")
        return
    
    if odom is None:
        # Conversion failed (e.g., 3D to 2D conversion error)
        return

    # Update the most recent pose key for the agent
    key2 = Key(msg.key2)
    agent = key2.char
    most_recent_pose_keys[agent] = key2

    try:
        rospy.logdebug(f"Adding odometry measurement: {odom}")
        estimator.add_odometry(odom)
    except Exception as e:
        rospy.logerr(f"Failed to add odometry measurement | odom: {odom} | error: {e}")


def handle_pose_factor(
    estimator: Estimator,
    msg: PoseFactorStamped,
    most_recent_pose_keys: Dict[str, Key],
    dimension: int = 3,
) -> None:
    """
    Process a pose factor message (either prior or odometry).
    
    PoseFactorStamped:
        Header header
        string key1
        string key2
        geometry_msgs/PoseWithCovariance pose
    
    If key1 == key2: This is a pose prior
    If key1 != key2: This is an odometry measurement
    
    Args:
        estimator: The estimator to update
        msg: PoseFactorStamped message
        most_recent_pose_keys: Dict mapping agent names to their most recent pose keys
        dimension: State space dimension (2 or 3)
    """
    try:
        if msg.key1 == msg.key2:
            rospy.logwarn(f"[handle_pose_factor] Adding pose prior for key: {msg.key1}")
            handle_pose_prior(estimator, msg)
        else:
            handle_odometry_measurement(estimator, msg, most_recent_pose_keys, dimension)
    except Exception as e:
        rospy.logerr(f"Error in handle_pose_factor: {e}")


def handle_range_factor(
    estimator: Estimator,
    msg: RangeFactorStamped,
) -> None:
    """
    Process a range factor message.
    
    RangeFactorStamped:
        Header header
        string key1
        string key2
        float32 meas_range
        float32 range_sigma
        float32 depth1
        float32 depth2
    
    Args:
        estimator: The estimator to update
        msg: RangeFactorStamped message
    """
    try:
        range_meas = range_factor_to_range_measurement(msg)
    except ValueError as e:
        rospy.logerr(f"Invalid range measurement: {e}")
        return

    try:
        estimator.add_range(range_meas)
    except Exception as e:
        rospy.logwarn(f"Failed to add range measurement: {e}")
