"""
Example: Cleaned-up version of key sections showing best practices.
This demonstrates what the refactored code should look like.
"""

# ============================================================================
# EXAMPLE 1: Simplified estimator_manager.py (ROS-free)
# ============================================================================

from typing import Dict, Set
from .estimator import Estimator
from .backends.gtsam.estimator import GtsamEstimator
from .backends.cora.estimator import CoraEstimator
from .types.key import Key
from .types.enums import EstimatorMode


class EstimatorManager:
    """Core estimation manager without ROS dependencies."""
    
    def __init__(
        self,
        agent_name: str,
        mode: EstimatorMode = EstimatorMode.GTSAM_LM,
        dimension: int = 3,
    ):
        self.dimension = dimension
        self.agent_name = agent_name
        self.most_recent_pose_keys: Dict[str, Key] = {}

        # Construct the appropriate estimator
        if mode == EstimatorMode.GTSAM_LM:
            self.estimator = GtsamEstimator(
                mode=mode,
                dimension=dimension,
                odom_factor_type="between",
            )
        elif mode == EstimatorMode.CORA:
            self.estimator = CoraEstimator(mode=mode, dimension=dimension)
        else:
            raise ValueError(f"Unsupported estimator mode: {mode}")

    def get_all_estimated_variables(self):
        """Return all estimated variables (poses and points)."""
        return self.estimator.current_estimate.all_variable_map

    @property
    def agents_seen(self) -> Set[str]:
        """Return set of agent names that have been seen."""
        return set(self.most_recent_pose_keys.keys())

    def update(self):
        """Update the estimator with current measurements."""
        self.estimator.update()


# ============================================================================
# EXAMPLE 2: ROS wrapper (ros/ros_manager.py)
# ============================================================================

import rospy
import threading
from typing import Dict
from ..estimator_manager import EstimatorManager
from ..types.key import Key
from ..types.enums import EstimatorMode
from .ros_interface import (
    parse_pose_prior,
    parse_odom_measurement,
    parse_range_measurement,
)
from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped


class ROSEstimatorManager:
    """ROS-aware wrapper around EstimatorManager."""
    
    def __init__(
        self,
        agent_name: str,
        mode: EstimatorMode = EstimatorMode.GTSAM_LM,
        dimension: int = 3,
    ):
        self.manager = EstimatorManager(agent_name, mode, dimension)
        self._lock = threading.RLock()
        self.has_received_data = False

        # Set up ROS subscribers
        self.pose_subscriber = rospy.Subscriber(
            f"{agent_name}/pose_factor",
            PoseFactorStamped,
            self._handle_pose_factor,
        )
        self.range_subscriber = rospy.Subscriber(
            f"{agent_name}/range_factor",
            RangeFactorStamped,
            self._handle_range_factor,
        )

    def _handle_pose_factor(self, msg: PoseFactorStamped):
        """Handle incoming pose factor message."""
        try:
            if msg.key1 == msg.key2:
                # Pose prior
                pose = parse_pose_prior(msg, self.manager.dimension)
                with self._lock:
                    self.manager.estimator.initialize_pose(pose)
                    self.manager.estimator.add_pose_prior(pose)
            else:
                # Odometry measurement
                odom = parse_odom_measurement(msg, self.manager.dimension)
                key1, key2 = odom.key1, odom.key2
                self.manager.most_recent_pose_keys[key2.char] = key2
                with self._lock:
                    self.manager.estimator.add_odometry(odom)
            
            self.has_received_data = True
        except Exception as e:
            rospy.logerr(f"Error handling pose factor: {e}")

    def _handle_range_factor(self, msg: RangeFactorStamped):
        """Handle incoming range factor message."""
        try:
            range_meas = parse_range_measurement(msg)
            with self._lock:
                self.manager.estimator.add_range(range_meas)
            self.has_received_data = True
        except Exception as e:
            rospy.logerr(f"Error handling range factor: {e}")

    def update(self):
        """Update the estimator if new data received."""
        with self._lock:
            if self.has_received_data:
                self.manager.update()
                self.has_received_data = False


# ============================================================================
# EXAMPLE 3: ROS message parsing (ros/ros_interface.py)
# ============================================================================

import numpy as np
from ..types.key import Key, KeyPair
from ..types.measurements import (
    RangeMeasurement,
    OdometryMeasurement3D,
    OdometryMeasurement2D,
)
from ..types.variables import Pose3D, Pose2D
from ..types.covariance import RelPoseCovar6, get_diag_relpose_covar
from ..utils.conversions import convert_odom3_to_odom2
from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped


def parse_pose_prior(msg: PoseFactorStamped, dimension: int):
    """Parse a pose prior from a ROS message."""
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
    
    covariance = np.array(msg.pose.covariance).reshape((6, 6))
    if np.allclose(covariance, 0.0):
        covariance[:3, :3] = np.eye(3) * 1e-2
        covariance[3:, 3:] = np.eye(3) * 1e-3
    
    if dimension == 3:
        return Pose3D(
            key=Key(msg.key1),
            position=position,
            orientation=orientation,
            marginal_covariance=covariance,
        )
    else:
        # Convert to 2D
        theta = np.arctan2(2*(orientation[3]*orientation[2] + orientation[0]*orientation[1]),
                          1 - 2*(orientation[1]**2 + orientation[2]**2))
        return Pose2D(
            key=Key(msg.key1),
            position=(position[0], position[1]),
            orientation=theta,
            marginal_covariance=covariance[:3, :3],
        )


def parse_odom_measurement(msg: PoseFactorStamped, dimension: int):
    """Parse an odometry measurement from a ROS message."""
    key_pair = KeyPair(Key(msg.key1), Key(msg.key2))
    
    translation = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    )
    rotation = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    )
    
    cov_mat = np.array(msg.pose.covariance).reshape((6, 6))
    
    # Extract standard deviations with guards
    sx = max(1e-6, float(np.sqrt(max(0.0, cov_mat[0, 0]))))
    sy = max(1e-6, float(np.sqrt(max(0.0, cov_mat[1, 1]))))
    sz = float(np.sqrt(max(0.0, cov_mat[2, 2])))
    spsi = max(1e-6, float(np.sqrt(max(0.0, cov_mat[3, 3]))))
    
    # Correlations
    rho_xy = float(cov_mat[0, 1] / (sx * sy)) if (sx * sy) != 0 else 0.0
    rho_xpsi = float(cov_mat[0, 3] / (sx * spsi)) if (sx * spsi) != 0 else 0.0
    rho_ypsi = float(cov_mat[1, 3] / (sy * spsi)) if (sy * spsi) != 0 else 0.0
    
    covar = RelPoseCovar6(
        relative_pose_sigma_x=sx,
        relative_pose_sigma_y=sy,
        relative_pose_sigma_z=sz,
        relative_pose_sigma_psi=spsi,
        relative_pose_rho_xy=rho_xy,
        relative_pose_rho_xpsi=rho_xpsi,
        relative_pose_rho_ypsi=rho_ypsi,
    )
    
    odom3d = OdometryMeasurement3D(
        key_pair=key_pair,
        relative_translation=translation,
        relative_rotation=rotation,
        covariance=covar,
    )
    
    if dimension == 2:
        return convert_odom3_to_odom2(odom3d)
    return odom3d


def parse_range_measurement(msg: RangeFactorStamped):
    """Parse a range measurement from a ROS message."""
    key_pair = KeyPair(Key(msg.key1), Key(msg.key2))
    
    variance = float(msg.range_sigma) ** 2 if msg.range_sigma > 0 else 1.0
    
    depth1 = float(msg.depth1) if msg.depth1 != 0.0 else None
    depth2 = float(msg.depth2) if msg.depth2 != 0.0 else None
    
    return RangeMeasurement(
        key_pair=key_pair,
        distance=float(msg.meas_range),
        variance=variance,
        depth1=depth1,
        depth2=depth2,
    )


# ============================================================================
# EXAMPLE 4: Cleaned-up estimator.py add_odometry (no nested try-except)
# ============================================================================

def add_odometry(self, odom_measurement):
    """Add an odometry measurement to the estimator."""
    key1, key2 = odom_measurement.key1, odom_measurement.key2

    # Initialize first pose at origin if needed
    if key1 not in self.current_estimate.pose_map and key1.index == 0:
        if self.dimension == 2:
            init_pose = Pose2D(key=key1, position=(0.0, 0.0), orientation=0.0)
        else:
            init_pose = Pose3D(
                key=key1,
                position=(0.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
            )
        self.initialize_pose(init_pose)

    # Get current pose and compute expected next pose
    pose1 = self.current_estimate.get_variable(key1)
    transform1 = pose1.transformation_matrix
    rel_transform = odom_measurement.transformation_matrix
    expected_transform2 = transform1 @ rel_transform

    # Extract rotation and translation
    expected_rot = expected_transform2[0:self.dimension, 0:self.dimension]
    expected_trans = expected_transform2[0:self.dimension, self.dimension]

    # Build and initialize expected pose
    if self.dimension == 2:
        theta = get_theta_from_rotation_matrix(expected_rot)
        expected_pose2 = Pose2D(
            key=key2,
            position=tuple(expected_trans),
            orientation=theta,
        )
    else:
        quat = tuple(get_quat_from_rotation_matrix(expected_rot))
        expected_pose2 = Pose3D(
            key=key2,
            position=tuple(expected_trans),
            orientation=quat,
        )
    
    self.initialize_pose(expected_pose2)
    self._specific_add_odometry(odom_measurement)


# ============================================================================
# KEY IMPROVEMENTS DEMONSTRATED:
# ============================================================================
#
# 1. Separation of Concerns
#    - EstimatorManager: core logic only
#    - ROSEstimatorManager: ROS integration only
#    - ros_interface: pure parsing functions
#
# 2. Clean Error Handling
#    - Single try-except at appropriate level
#    - Log once, handle appropriately
#    - No nested exception pyramids
#
# 3. Minimal Logging
#    - Only ERROR level for actual problems
#    - No debug noise in production code
#    - Let users increase verbosity if needed
#
# 4. Clear Responsibilities
#    - Each function does one thing
#    - Easy to test in isolation
#    - Easy to understand and maintain
#
# 5. Type Hints
#    - Clear parameter and return types
#    - Better IDE support
#    - Self-documenting code
#
# 6. No Dead Code
#    - No commented sections
#    - No TEST CODE markers
#    - No temporary workarounds
#
