#!/usr/bin/env python3
"""
ROS-agnostic estimator manager.

Coordinates backend estimators and manages state estimation logic
without any ROS dependencies. ROS integration is handled by ros/manager.py.
"""

from .estimator import Estimator
from .backends.gtsam import GtsamEstimator
from .types.key import Key
from .types.variables import Pose3D, Pose2D, Point2D, Point3D
from .types.enums import EstimatorMode

import threading
from typing import Union, Set, Dict



class EstimatorManager:
    """
    ROS-agnostic manager for backend estimators.
    
    Coordinates state estimation without any ROS dependencies.
    Tracks most recent pose keys for each agent and provides access
    to estimated variables.
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
        Initializes the EstimatorManager with the specified mode and dimension.
        
        Args:
            agent_name: The name of the agent using this estimator
            mode: The mode of the estimator (GTSAM_LM, GTSAM_DOG, or CORA)
            dimension: The dimension of the state space (2 or 3)
            enable_diagnostics: If True, run GTSAM optimization diagnostics (default: False)
            try_gauss_newton_fallback: If True, try Gauss-Newton if LM fails (default: False)
        """
        self.dimension = dimension
        self.agent_name = agent_name
        self.most_recent_pose_keys: Dict[str, Key] = {}
        
        # Re-entrant lock to guard estimator access (updates and other calls)
        self._lock = threading.RLock()

        # Construct the estimator based on the mode
        if mode == EstimatorMode.GTSAM_LM:
            self.estimator = GtsamEstimator(
                mode=EstimatorMode.GTSAM_LM,
                dimension=dimension,
                odom_factor_type="between",
                enable_diagnostics=enable_diagnostics,
                try_gauss_newton_fallback=try_gauss_newton_fallback,
            )
        elif mode == EstimatorMode.CORA:
            from .cora_estimator import CoraEstimator

            self.estimator = CoraEstimator(mode=EstimatorMode.CORA, dimension=dimension)
        else:
            raise ValueError(f"Unsupported estimator mode: {mode}")

        self.has_received_data = False

    def get_all_estimated_variables(self) -> Dict[Key, Union[Pose2D, Pose3D, Point2D, Point3D]]:
        """
        Return a dictionary of all estimated variables (poses and points) for this agent.
        
        Returns:
            Dict mapping Keys to estimated variables (Pose2D, Pose3D, Point2D, or Point3D)
        """
        with self._lock:
            return self.estimator.current_estimate.all_variable_map

    @property
    def agents_seen(self) -> Set[str]:
        """
        Return the set of agent names that have been observed.
        
        Returns:
            Set of agent names (single characters)
        """
        return set(self.most_recent_pose_keys.keys())

    def update(self) -> bool:
        """
        Update the estimator with the current measurements.
        
        Returns:
            True if update was successful, False if no new data or update failed
        """
        with self._lock:
            if not self.has_received_data:
                return False

            try:
                self.estimator.update()
                self.has_received_data = False
                return True
            except Exception as e:
                # Log error but don't raise (let caller handle)
                print(f"Estimator update failed: {e}")
                return False
    
    def mark_data_received(self) -> None:
        """
        Mark that new data has been received and update is needed.
        
        This should be called by the ROS wrapper after adding measurements.
        """
        with self._lock:
            self.has_received_data = True


