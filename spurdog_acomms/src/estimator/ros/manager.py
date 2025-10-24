"""
ROS wrapper for EstimatorManager.

Provides ROS integration by handling subscriber setup and message routing.
"""

import rospy
from typing import Dict

from ..estimator_manager import EstimatorManager
from ..types.enums import EstimatorMode
from ..types.key import Key
from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped

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
    
    def update(self) -> bool:
        """
        Update the estimator with current measurements.
        
        Returns:
            True if update was successful, False otherwise
        """
        success = self.manager.update()
        if not success:
            rospy.logdebug("No new data received, skipping update.")
        return success
