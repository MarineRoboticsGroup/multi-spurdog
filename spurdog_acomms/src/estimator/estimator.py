from enum import Enum
from abc import ABC, abstractmethod
from attrs import define, field, validators
from typing import Optional, Tuple, Union
import numpy as np
from numpy import ndarray
from estimator_helpers import (
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
)

class Estimator(ABC):
    """
    An abstract base class to define the interface for an estimator.
    """

    def __init__(self, mode: EstimatorMode, dimension: int):
        self.mode: EstimatorMode = mode
        self.dimension: int = dimension

        if dimension not in [2, 3]:
            raise ValueError(
                f"Dimension must be 2 or 3, got {dimension}. This estimator is designed for 2D or 3D state estimation."
            )

    @abstractmethod
    def add_range(self, range_measurement: RangeMeasurement):
        """
        Add a range measurement to the estimator.

        Args:
            range_measurement: The range measurement to be added.
        """
        pass

    @abstractmethod
    def add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        """
        Add an odometry measurement to the estimator.

        Args:
            odometry_measurement: The odometry measurement to be added.
        """
        pass

    @abstractmethod
    def initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Initialize the pose of a key in the estimator.

        Args:
            key: The key for which to initialize the pose.
            pose: The initial pose to set for the key.
        """
        raise NotImplementedError("This method should be implemented by subclasses.")

    @abstractmethod
    def initialize_point(self, point: Union[Point2D, Point3D]) -> None:
        """
        Initialize the point of a key in the estimator.

        Args:
            key: The key for which to initialize the point.
            point: The initial point to set for the key.
        """
        raise NotImplementedError("This method should be implemented by subclasses.")

    @abstractmethod
    def add_pose_prior(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Add a pose prior to the estimator.

        Args:
            pose: The pose to be added as a prior.
            key: The key associated with the pose.
        """
        assert pose.key is not None, "Pose key must be set before adding a prior."
        assert pose.marginal_covariance is not None, "Pose marginal covariance must be set before adding a prior."
        raise NotImplementedError("This method should be implemented by subclasses.")


    @abstractmethod
    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        """
        Add a depth measurement to the estimator.

        Args:
            depth_measurement: The depth measurement to be added.
        """
        pass

    @abstractmethod
    def get_pose(self, key: Key) -> Union[Pose2D, Pose3D]:
        """
        Get the current state of the estimator.

        Returns:
            The current state of the estimator.
        """
        pass

    @abstractmethod
    def get_point(self, key: Key) -> Union[Point2D, Point3D]:
        """
        Get the current point estimate from the estimator.

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


if __name__ == "__main__":
    # Example usage
    key1 = Key("X1")
    key2 = Key("X1")
