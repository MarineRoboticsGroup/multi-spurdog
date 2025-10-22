"""
Abstract base class and interface for estimator implementations.
Defines the Estimator API for range, odometry, pose, and point management.
"""

from enum import Enum
from abc import ABC, abstractmethod
from typing import Optional, Tuple, Union, Dict, Set
import numpy as np
from numpy import ndarray
from estimator.estimator_helpers import (
    Key,
    RangeMeasurement,
    OdometryMeasurement,
    DepthMeasurement,
    EstimatorMode,
    RelPoseCovar6,
    Pose3D,
    Pose2D,
    Point2D,
    Point3D,
    get_theta_from_rotation_matrix,
    get_quat_from_rotation_matrix,
    KeyPair,
)
from estimator.values import EstimatorValues
import rospy


class Estimator(ABC):
    """
    Abstract base class for estimators.
    Defines the interface for adding measurements, initializing variables, and updating state.
    Subclasses must implement the core methods for their specific backend.
    """

    def __init__(self, mode: EstimatorMode, dimension: int):
        """
        Initialize the estimator with mode and dimension.
        Args:
            mode: The estimator mode (e.g., GTSAM, CORA).
            dimension: State space dimension (2 or 3).
        """
        self.mode: EstimatorMode = mode
        self.dimension: int = dimension
        self.current_estimate: EstimatorValues = EstimatorValues()
        self.delayed_range_inits: Dict[Key, RangeMeasurement] = {}
        self.range_measurement_pairs: Set[Tuple[Key, Key]] = set()
        if dimension not in [2, 3]:
            raise ValueError(
                f"Dimension must be 2 or 3, got {dimension}. This estimator is designed for 2D or 3D state estimation."
            )

    def add_range(self, range_measurement: RangeMeasurement):
        """
        Add a range measurement to the estimator. If the pose variable for key1 is not initialized,
        the measurement is delayed until initialization. If key2 is a landmark and not initialized,
        it is initialized at the appropriate distance from key1.
        Args:
            range_measurement: The range measurement to be added.
        """
        key1, key2 = range_measurement.key1, range_measurement.key2
        key1_exists = key1 in self.current_estimate.pose_map
        if not key1_exists:
            rospy.logdebug(
                f"Pose for key {key1} not initialized yet, delaying range measurement for landmark {key2}."
            )
            self.delayed_range_inits[key1] = range_measurement
            return

        # If key2 is a landmark and not initialized, initialize it at the correct distance from key1
        need_init_landmark = (
            key2.is_landmark and key2 not in self.current_estimate.point_map
        )
        if need_init_landmark:
            pose1 = self.current_estimate.get_variable(key1)
            assert isinstance(
                pose1, (Pose2D, Pose3D)
            ), f"Expected pose for key {key1} to be Pose2D or Pose3D, got {type(pose1)}"
            position1 = pose1.position
            rand_unit_vec = np.random.randn(self.dimension)
            rand_unit_vec /= np.linalg.norm(rand_unit_vec)
            position2 = np.array(position1) + rand_unit_vec * range_measurement.distance
            if self.dimension == 2:
                init_point = Point2D(key=key2, position=tuple(position2))
            elif self.dimension == 3:
                init_point = Point3D(key=key2, position=tuple(position2))
            else:
                raise ValueError(f"Unknown dimension: {self.dimension}")
            self.initialize_point(init_point)
        self._specific_add_range(range_measurement)
        self.range_measurement_pairs.add((key1, key2))

    @abstractmethod
    def _specific_add_range(self, range_measurement: RangeMeasurement) -> None:
        """
        Backend-specific implementation for adding a range measurement.
        Args:
            range_measurement: The range measurement to be added.
        """
        raise NotImplementedError(
            "[_specific_add_range] This method should be implemented by subclasses."
        )

    def add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        """
        Add an odometry measurement to the estimator. If key1 is not initialized and is a start pose,
        it is initialized at the origin. Computes the expected pose for key2 and initializes it if needed.
        Args:
            odometry_measurement: The odometry measurement to be added.
        """
        key1, key2 = odom_measurement.key1, odom_measurement.key2

        def _handle_first_pose_initialization():
            not_initialized = key1 not in self.current_estimate.pose_map
            is_start_pose = key1.index == 0
            if not_initialized and is_start_pose:
                if self.dimension == 2:
                    init_pose = Pose2D(key=key1, position=(0.0, 0.0), orientation=0.0)
                elif self.dimension == 3:
                    init_pose = Pose3D(
                        key=key1,
                        position=(0.0, 0.0, 0.0),
                        orientation=(0.0, 0.0, 0.0, 1.0),
                    )
                else:
                    raise ValueError(f"Unknown dimension: {self.dimension}")
                self.initialize_pose(init_pose)

        _handle_first_pose_initialization()
        pose1 = self.current_estimate.get_variable(key1)
        assert isinstance(
            pose1, (Pose2D, Pose3D)
        ), f"Expected pose for key {key1} to be Pose2D or Pose3D, got {type(pose1)}"
        transform1 = pose1.transformation_matrix
        rel_transform = odom_measurement.transformation_matrix
        expected_transform2 = transform1 @ rel_transform
        expected_rot = expected_transform2[0 : self.dimension, 0 : self.dimension]
        expected_trans = expected_transform2[0 : self.dimension, self.dimension]
        if self.dimension == 2:
            expected_pose2 = Pose2D(
                key=key2,
                position=tuple(expected_trans),
                orientation=get_theta_from_rotation_matrix(expected_rot),
            )
            self.initialize_pose(expected_pose2)
        elif self.dimension == 3:
            expected_pose2 = Pose3D(
                key=key2,
                position=tuple(expected_trans),
                orientation=tuple(get_quat_from_rotation_matrix(expected_rot)),
            )
            self.initialize_pose(expected_pose2)
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")
        self._specific_add_odometry(odom_measurement)

    @abstractmethod
    def _specific_add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        """
        Backend-specific implementation for adding an odometry measurement.
        Args:
            odometry_measurement: The odometry measurement to be added.
        """
        pass

    def initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Initialize the pose of a key in the estimator and process any delayed range measurements.
        Args:
            pose: The initial pose to set for the key.
        """
        assert isinstance(
            pose, (Pose2D, Pose3D)
        ), "Pose must be of type Pose2D or Pose3D"
        self.current_estimate.update_variable(pose)
        self._specific_initialize_pose(pose)
        rospy.logdebug(f"[estimator] Initialized pose for key {pose.key}")
        if pose.key in self.delayed_range_inits:
            range_measurement = self.delayed_range_inits.pop(pose.key)
            rospy.logdebug(
                f"Processing delayed initialization for landmark {pose.key} after pose initialization."
            )
            self.add_range(range_measurement)

    @abstractmethod
    def _specific_initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Backend-specific implementation for initializing a pose.
        Args:
            pose: The initial pose to set for the key.
        """
        raise NotImplementedError(
            "[_specific_initialize_pose] This method should be implemented by subclasses."
        )

    def initialize_point(self, point: Union[Point2D, Point3D]) -> None:
        """
        Initialize the point of a key in the estimator.
        Args:
            point: The initial point to set for the key.
        """
        assert isinstance(
            point, (Point2D, Point3D)
        ), "Point must be of type Point2D or Point3D"
        self.current_estimate.update_variable(point)
        self._specific_initialize_point(point)
        rospy.logdebug(f"[estimator] Initialized point for key {point.key}")

    @abstractmethod
    def _specific_initialize_point(self, point: Union[Point2D, Point3D]) -> None:
        """
        Backend-specific implementation for initializing a point.
        Args:
            point: The initial point to set for the key.
        """
        raise NotImplementedError(
            "[_specific_initialize_point] This method should be implemented by subclasses."
        )

    @abstractmethod
    def add_pose_prior(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Add a pose prior to the estimator.
        Args:
            pose: The pose to be added as a prior. Must have key and marginal_covariance set.
        """
        assert pose.key is not None, "Pose key must be set before adding a prior."
        assert (
            pose.marginal_covariance is not None
        ), "Pose marginal covariance must be set before adding a prior."
        raise NotImplementedError(
            "[add_pose_prior] This method should be implemented by subclasses."
        )

    @abstractmethod
    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        """
        Add a depth measurement to the estimator.
        Args:
            depth_measurement: The depth measurement to be added.
        """
        pass

    @abstractmethod
    def get_pose_from_estimator(self, key: Key) -> Union[Pose2D, Pose3D]:
        """
        Get the current pose estimate for a given key.
        Args:
            key: The key for which to retrieve the pose.
        Returns:
            The current pose estimate.
        """
        pass

    @abstractmethod
    def get_point_from_estimator(self, key: Key) -> Union[Point2D, Point3D]:
        """
        Get the current point estimate for a given key.
        Args:
            key: The key for which to retrieve the point estimate.
        Returns:
            The point estimate corresponding to the given key.
        """
        pass

    @abstractmethod
    def update(self):
        """
        Update the estimator with the latest measurements.
        This method should be called after adding all measurements.
        """
        pass


# No main routine: this module is intended as an abstract interface only.
