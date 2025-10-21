from enum import Enum
from abc import ABC, abstractmethod
from typing import Optional, Tuple, Union, Dict
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
)
from estimator.values import EstimatorValues
import rospy


class Estimator(ABC):
    """
    An abstract base class to define the interface for an estimator.
    """

    def __init__(self, mode: EstimatorMode, dimension: int):
        self.mode: EstimatorMode = mode
        self.dimension: int = dimension
        self.current_estimate: EstimatorValues = EstimatorValues()

        self.delayed_range_inits: Dict[Key, RangeMeasurement] = {}

        if dimension not in [2, 3]:
            raise ValueError(
                f"Dimension must be 2 or 3, got {dimension}. This estimator is designed for 2D or 3D state estimation."
            )

    def add_range(self, range_measurement: RangeMeasurement):
        """
        Add a range measurement to the estimator. If the pose variable for key1 is not initialized, then
        we will add the range measurement to a delayed initialization list to be processed later.

        Args:
            range_measurement: The range measurement to be added.
        """
        key1, key2 = range_measurement.key1, range_measurement.key2
        key1_exists = key1 in self.current_estimate.pose_map
        if not key1_exists:
            rospy.logwarn(f"Pose for key {key1} not initialized yet, delaying range measurement for landmark {key2}.")
            self.delayed_range_inits[key1] = range_measurement
            return

        # if key2 is a landmark and not initialized, initialize it at some
        # default position the proper distance from key1
        need_init_landmark = key2.is_landmark and key2 not in self.current_estimate.point_map
        if need_init_landmark:
            pose1 = self.current_estimate.get_variable(key1)
            assert isinstance(pose1, (Pose2D, Pose3D)), f"Expected pose for key {key1} to be Pose2D or Pose3D, got {type(pose1)}"
            position1 = pose1.position
            rand_unit_vec = np.random.randn(self.dimension)
            rand_unit_vec /= np.linalg.norm(rand_unit_vec)
            position2 = np.array(position1) + rand_unit_vec * range_measurement.distance

            # initialize to origin
            if self.dimension == 2:
                init_point = Point2D(
                    key=key2,
                    position=tuple(position2),
                )
            elif self.dimension == 3:
                init_point = Point3D(
                    key=key2,
                    position=tuple(position2),
                )
            else:
                raise ValueError(f"Unknown dimension: {self.dimension}")

            self.initialize_point(init_point)


        self._specific_add_range(range_measurement)

    @abstractmethod
    def _specific_add_range(self, range_measurement: RangeMeasurement) -> None:
        """
        Add a range measurement to the estimator.

        Args:
            range_measurement: The range measurement to be added.
        """
        raise NotImplementedError("[_specific_add_range] This method should be implemented by subclasses.")

    def add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        """
        Add an odometry measurement to the estimator.

        Args:
            odometry_measurement: The odometry measurement to be added.
        """
        key1, key2 = odom_measurement.key1, odom_measurement.key2

        # handle the edge case, where key1 has not been initialized yet (should only happen for first instance of an agent's pose)
        def _handle_first_pose_initialization():
            not_initialized = key1 not in self.current_estimate.pose_map
            is_start_pose = key1.index == 0
            if not_initialized and is_start_pose:
                # initialize to origin
                if self.dimension == 2:
                    init_pose = Pose2D(
                        key=key1,
                        position=(0.0, 0.0),
                        orientation=0.0,
                    )
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
        assert isinstance(pose1, (Pose2D, Pose3D)), f"Expected pose for key {key1} to be Pose2D or Pose3D, got {type(pose1)}"

        transform1 = pose1.transformation_matrix
        rel_transform = odom_measurement.transformation_matrix
        expected_transform2 = transform1 @ rel_transform
        expected_rot = expected_transform2[0: self.dimension, 0: self.dimension]
        expected_trans = expected_transform2[0: self.dimension, self.dimension]
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
        Add an odometry measurement to the estimator.

        Args:
            odometry_measurement: The odometry measurement to be added.
        """
        pass

    def initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Initialize the pose of a key in the estimator.

        Args:
            key: The key for which to initialize the pose.
            pose: The initial pose to set for the key.
        """
        assert isinstance(pose, (Pose2D, Pose3D)), "Pose must be of type Pose2D or Pose3D"
        self.current_estimate.update_variable(pose)
        self._specific_initialize_pose(pose)
        rospy.loginfo(f"[estimator] Initialized pose for key {pose.key}")

        if pose.key in self.delayed_range_inits:
            range_measurement = self.delayed_range_inits.pop(pose.key)
            rospy.loginfo(f"Processing delayed initialization for landmark {pose.key} after pose initialization.")
            self.add_range(range_measurement)


    @abstractmethod
    def _specific_initialize_pose(
        self, pose: Union[Pose2D, Pose3D]
    ) -> None:
        """
        Estimator-specific implementation for initializing a pose.

        Args:
            pose: The initial pose to set for the key.
        """
        raise NotImplementedError("[_specific_initialize_pose] This method should be implemented by subclasses.")

    def initialize_point(self, point: Union[Point2D, Point3D]) -> None:
        """
        Initialize the point of a key in the estimator.

        Args:
            key: The key for which to initialize the point.
            point: The initial point to set for the key.
        """
        assert isinstance(point, (Point2D, Point3D)), "Point must be of type Point2D or Point3D"
        self.current_estimate.update_variable(point)
        self._specific_initialize_point(point)
        rospy.loginfo(f"[estimator] Initialized point for key {point.key}")

    @abstractmethod
    def _specific_initialize_point(
        self, point: Union[Point2D, Point3D]
    ) -> None:
        """
        Estimator-specific implementation for initializing a point.

        Args:
            point: The initial point to set for the key.
        """
        raise NotImplementedError("[_specific_initialize_point] This method should be implemented by subclasses.")

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
        raise NotImplementedError("[add_pose_prior] This method should be implemented by subclasses.")


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
        Get the current state of the estimator.

        Returns:
            The current state of the estimator.
        """
        pass

    @abstractmethod
    def get_point_from_estimator(self, key: Key) -> Union[Point2D, Point3D]:
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
