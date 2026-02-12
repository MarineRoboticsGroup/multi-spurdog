"""
ROS wrapper for EstimatorManager.

Provides ROS integration by handling subscriber setup and message routing.
"""

import rospy
import numpy as np
from typing import Dict, Optional
from scipy.spatial.transform import Rotation as R

from ..estimator_manager import EstimatorManager
from ..types.enums import EstimatorMode
from ..types.key import Key, KeyPair
from ..types.variables import Pose2D, Pose3D
from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from .handlers import handle_pose_factor, handle_range_factor


class ROSEstimatorManager:
    """
    ROS-aware wrapper around EstimatorManager.
    
    Handles ROS subscriber setup, message routing, and provides
    the same interface as EstimatorManager while managing ROS integration.
    """
    
    def __init__(
        self,
        agent_name: str,
        mode: EstimatorMode = EstimatorMode.GTSAM_LM,
        dimension: int = 3,
        enable_diagnostics: bool = False,
        try_gauss_newton_fallback: bool = False,
    ):
        """
        Initialize the ROS estimator manager.
        
        Args:
            agent_name: The name of the agent using this estimator
            mode: The mode of the estimator (GTSAM_LM, GTSAM_DOG, or CORA)
            dimension: The dimension of the state space (2 or 3)
            enable_diagnostics: If True, run GTSAM optimization diagnostics (default: False)
            try_gauss_newton_fallback: If True, try Gauss-Newton if LM fails (default: False)
        """
        self.agent_name = agent_name
        self.dimension = dimension
        
        # LOCAL FRAME TRANSFORMATION - DISABLED per Morrison's thesis
        # Morrison's actual CORA implementation uses RANDOM INITIALIZATION in WORLD FRAME
        # No coordinate transformations needed - let both estimators work directly in world frame
        self.world_frame_reference = None  # Dict with 'position' and 'rotation' (scipy Rotation object)
        # DISABLED: Morrison's thesis shows CORA uses random init, not local frame transforms
        from estimator.types import EstimatorMode
        self.use_local_frame = False  # Disabled for both GTSAM and CORA - Morrison uses world frame
        
        # Track integrated_state for pose initialization
        self.integrated_state_count = 0
        self.first_integrated_state: Optional[PoseWithCovarianceStamped] = None
        self.prev_integrated_state: Optional[PoseWithCovarianceStamped] = None
        self.surface_timestamp = None  # Could be set to filter post-surface data
        
        # Create the core estimator manager (ROS-agnostic)
        self.manager = EstimatorManager(
            agent_name=agent_name,
            mode=mode,
            dimension=dimension,
            enable_diagnostics=enable_diagnostics,
            try_gauss_newton_fallback=try_gauss_newton_fallback,
        )
        
        # Set up ROS subscribers
        self.pose_subscriber = rospy.Subscriber(
            f"{self.agent_name}/pose_factor",
            PoseFactorStamped,
            self._handle_pose_factor_callback,
        )
        self.range_subscriber = rospy.Subscriber(
            f"{self.agent_name}/range_factor",
            RangeFactorStamped,
            self._handle_range_factor_callback,
        )
        
        # Subscribe to integrated_state for pose initialization (Morrison Fix #5)
        self.integrated_state_subscriber = rospy.Subscriber(
            f"{self.agent_name}/integrated_state",
            PoseWithCovarianceStamped,
            self._handle_integrated_state_callback,
            queue_size=500,
        )
        
        rospy.loginfo(f"ROSEstimatorManager initialized with integrated_state subscriber for {agent_name}")
    
    def _handle_pose_factor_callback(self, msg: PoseFactorStamped) -> None:
        """
        ROS callback for pose factor messages.
        
        Routes messages to handler and marks data as received.
        """
        handle_pose_factor(
            self.manager.estimator,
            msg,
            self.manager.most_recent_pose_keys,
            self.dimension,
        )
        self.manager.mark_data_received()
    
    def _handle_range_factor_callback(self, msg: RangeFactorStamped) -> None:
        """
        ROS callback for range factor messages.
        
        Routes messages to handler and marks data as received.
        """
        handle_range_factor(self.manager.estimator, msg)
        self.manager.mark_data_received()
    
    def _handle_integrated_state_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        ROS callback for integrated_state messages (absolute world-frame pose).
        
        Morrison Fix #5: Initialize poses from integrated_state to handle disconnected odometry chains.
        Morrison Fix #6: Create between-factors from consecutive integrated_state messages.
        Morrison LOCAL FRAME: Transform world coordinates to local frame for optimization.
        
        Pipeline: World Frame Input → Local Frame Optimization → World Frame Output
        """
        # Filter out post-surface data if timestamp provided
        if self.surface_timestamp and msg.header.stamp.to_sec() >= self.surface_timestamp:
            return
        
        # Assign key based on message count (A0, A1, A2, ...)
        key = Key(f"A{self.integrated_state_count}")
        self.integrated_state_count += 1
        
        # Extract pose from message (WORLD FRAME)
        position_world = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        
        orientation_world = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        
        # Store first integrated state as reference frame
        if self.first_integrated_state is None:
            self.first_integrated_state = msg
            # Store world frame reference for transformation
            self.world_frame_reference = {
                'position': position_world.copy(),
                'rotation': R.from_quat(orientation_world),
                'rotation_inv': R.from_quat(orientation_world).inv()
            }
            rospy.loginfo(f"[Morrison Fix #5] Stored first integrated_state for key {key}")
            rospy.loginfo(f"[Morrison LOCAL FRAME] World frame reference at position: {position_world}")
            rospy.loginfo(f"[Morrison LOCAL FRAME] World frame reference orientation (quat): {orientation_world}")
            rospy.loginfo(f"[Morrison LOCAL FRAME] Transformation enabled: {self.use_local_frame}")
        
        # Transform pose from WORLD FRAME to LOCAL FRAME for optimization
        # Morrison: "It generates relative poses (relative to the landmarks)"
        # Local frame: first pose at (0,0,0) with identity rotation
        if self.use_local_frame and self.world_frame_reference is not None:
            # Apply transformation: local = R0_inv @ (world - p0)
            position_local = self.world_frame_reference['rotation_inv'].apply(
                position_world - self.world_frame_reference['position']
            )
            orientation_local_R = self.world_frame_reference['rotation_inv'] * R.from_quat(orientation_world)
            orientation_local = orientation_local_R.as_quat()
            
            # Log first pose transformation for verification
            if key.index == 0:
                rospy.loginfo(f"[Morrison LOCAL FRAME] First pose transformed:")
                rospy.loginfo(f"  World: pos={position_world}, quat={orientation_world}")
                rospy.loginfo(f"  Local: pos={position_local}, quat={orientation_local}")
        else:
            # No transformation - use world frame directly (for rollback/comparison)
            position_local = position_world
            orientation_local = orientation_world
            orientation_local_R = R.from_quat(orientation_world)
        
        # Create pose in LOCAL optimization frame
        if self.dimension == 3:
            pose = Pose3D(
                key=key,
                position=tuple(position_local),
                orientation=tuple(orientation_local),
                marginal_covariance=np.eye(6) * 1e-6,  # Tight prior for initialization
            )
        else:
            # Convert to 2D
            euler = orientation_local_R.as_euler('xyz')
            theta = euler[2]  # Yaw angle
            pose = Pose2D(
                key=key,
                position=(position_local[0], position_local[1]),
                orientation=theta,
                marginal_covariance=np.eye(3) * 1e-6,
            )
        
        # Initialize or UPDATE the pose in the estimator
        # Morrison Fix #5: integrated_state provides authoritative world coordinates
        # Update is needed because odometry may have temporarily initialized pose 0 at origin
        with self.manager._lock:
            if not self.manager.estimator.current_estimate.key_exists(key):
                self.manager.estimator.initialize_pose(pose)
                self.manager.most_recent_pose_keys[key.char] = key
                rospy.logdebug(f"Initialized pose {key} from integrated_state")
            else:
                # Update existing pose with authoritative integrated_state data
                self.manager.estimator.current_estimate.update_variable(key, pose)
                rospy.logdebug(f"Updated pose {key} from integrated_state (overwriting temporary initialization)")
        
            # Morrison Fix #6: Create between-factor from consecutive integrated_state messages
            # Only create if BOTH poses exist (prevents race condition)
            if self.prev_integrated_state is not None:
                prev_key = Key(f"A{self.integrated_state_count - 2}")
                # Check that BOTH poses exist before creating between-factor
                if (self.manager.estimator.current_estimate.key_exists(prev_key) and
                    self.manager.estimator.current_estimate.key_exists(key)):
                    self._create_between_factor_from_integrated_state(
                        self.prev_integrated_state,
                        msg,
                        prev_key,
                        key
                    )
                else:
                    rospy.logdebug(f"Skipping between-factor {prev_key}->{key}: one or both poses not initialized yet")
        
        # Store for next iteration
        self.prev_integrated_state = msg
    
    def _create_between_factor_from_integrated_state(
        self,
        msg1: PoseWithCovarianceStamped,
        msg2: PoseWithCovarianceStamped,
        key1: Key,
        key2: Key
    ) -> None:
        """
        Morrison Fix #6: Create between-factor from consecutive integrated_state messages.
        """
        from .message_converters import get_diag_relpose_covar
        from ..types.measurements import OdometryMeasurement3D
        
        # Extract positions
        pos1 = np.array([msg1.pose.pose.position.x, msg1.pose.pose.position.y, msg1.pose.pose.position.z])
        pos2 = np.array([msg2.pose.pose.position.x, msg2.pose.pose.position.y, msg2.pose.pose.position.z])
        
        # Extract orientations (in [x,y,z,w] order for scipy)
        quat1 = np.array([msg1.pose.pose.orientation.x, msg1.pose.pose.orientation.y, 
                         msg1.pose.pose.orientation.z, msg1.pose.pose.orientation.w])
        quat2 = np.array([msg2.pose.pose.orientation.x, msg2.pose.pose.orientation.y,
                         msg2.pose.pose.orientation.z, msg2.pose.pose.orientation.w])
        
        # Compute relative transformation
        R1 = R.from_quat(quat1)
        R2 = R.from_quat(quat2)
        R_rel = R1.inv() * R2
        
        # Relative translation in body frame of pose 1
        delta_pos_world = pos2 - pos1
        delta_pos_body = R1.inv().apply(delta_pos_world)
        
        # Create covariance (relaxed for integrated_state between-factors)
        # Position: 0.3m std, Orientation: 3 degrees std
        covar = get_diag_relpose_covar(np.array([0.3, 0.3, 0.3, 0.05, 0.05, 0.05]))
        
        # Create odometry measurement (use 3D version explicitly)
        key_pair = KeyPair(key1=key1, key2=key2)
        odom = OdometryMeasurement3D(
            key_pair=key_pair,
            relative_translation=tuple(delta_pos_body),
            relative_rotation=tuple(R_rel.as_quat()),
            covariance=covar
        )
        
        # Add to estimator
        with self.manager._lock:
            self.manager.estimator.add_odometry(odom)
            rospy.logdebug(f"[Morrison Fix #6] Added between-factor {key1}->{key2} from integrated_state")
    
    # Delegate properties and methods to the underlying manager
    
    @property
    def estimator(self):
        """Access the underlying estimator."""
        return self.manager.estimator
    
    @property
    def most_recent_pose_keys(self) -> Dict[str, Key]:
        """Access the most recent pose keys."""
        return self.manager.most_recent_pose_keys
    
    @property
    def agents_seen(self):
        """Access the set of agents seen."""
        return self.manager.agents_seen
    
    def get_all_estimated_variables(self):
        """Get all estimated variables from the manager."""
        return self.manager.get_all_estimated_variables()
    
    def update(self, force: bool = False) -> bool:
        """
        Update the estimator with current measurements.
        
        Args:
            force: If True, run optimization even if no new data received
        
        Returns:
            True if update was successful, False otherwise
        """
        success = self.manager.update(force=force)
        if not success:
            rospy.logdebug("No new data received, skipping update.")
        return success
    
    def get_world_frame_reference(self):
        """
        Get the world frame reference for local→world transformation.
        
        Returns:
            Dict with 'position', 'rotation', 'rotation_inv' or None if not initialized
        """
        return self.world_frame_reference
    
    def is_using_local_frame(self) -> bool:
        """Check if local frame transformation is enabled."""
        return self.use_local_frame
