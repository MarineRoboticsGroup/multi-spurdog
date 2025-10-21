from .estimator_helpers import (
    Key,
    KeyPair,
    RangeMeasurement,
    OdometryMeasurement,
    OdometryMeasurement2D,
    OdometryMeasurement3D,
    convert_odom3_to_odom2,
    DepthMeasurement,
    EstimatorMode,
    RelPoseCovar6,
    Pose3D,
    Pose2D,
    Point2D,
    Point3D,
    _check_transformation_matrix,
    _check_valid_key,
    get_theta_from_transformation_matrix,
    get_quat_from_rotation_matrix,
    get_theta_from_rotation_matrix,
    get_translation_from_transformation_matrix,
    get_measurement_precisions_from_covariance_matrix,
    get_diag_relpose_covar,
)
from attrs import define, field
import numpy as np
from typing import Dict, List, Optional, Union

@define
class EstimatorValues:
    """
    A class to hold the current estimated values from the estimator. Assumes that
    points are variables that start with "L" and poses are variables that start
    with any other letter.
    """

    pose_map: Dict[Key, Union[Pose2D, Pose3D]] = field(factory=dict)
    """
    A mapping from pose keys to their estimated Pose2D or Pose3D values.
    """

    point_map: Dict[Key, Union[Point2D, Point3D]] = field(factory=dict)
    """
    A mapping from point keys to their estimated Point2D or Point3D values.
    """

    def key_exists(self, key: Key) -> bool:
        """
        Check if a given key exists in the current estimates.

        Args:
            key: The key to check.
        Returns:
            True if the key exists, False otherwise.
        """
        assert isinstance(key, Key), f"Expected key to be of type Key, got {type(key)}"
        return key in self.pose_map or key in self.point_map

    def get_variable(self, key: Key) -> Optional[Union[Pose2D, Pose3D, Point2D, Point3D]]:
        """
        Get the estimated variable (pose or point) for a given key.

        Args:
            key: The key of the variable to retrieve.

        Returns:
            The estimated Pose2D, Pose3D, Point2D, or Point3D if it exists, otherwise None.
        """
        assert isinstance(key, Key), f"Expected key to be of type Key, got {type(key)}"
        try:
            if key.is_landmark:
                return self.point_map[key]
            else:
                return self.pose_map[key]
        except KeyError as e:
            print(f"Key {key} not found in current estimates: {e}")
            raise e

    def update_variable(
        self,
        variable: Union[Pose2D, Pose3D, Point2D, Point3D],
    ) -> None:
        """
        Update the estimated variable (pose or point) in the appropriate map.

        Args:
            variable: The variable to update (Pose2D, Pose3D, Point2D, or Point3D).
        """
        key = variable.key
        if key.is_landmark:
            assert isinstance(variable, (Point2D, Point3D)), "Expected a Point2D or Point3D for landmark key."
            self.point_map[key] = variable
        else:
            assert isinstance(variable, (Pose2D, Pose3D)), "Expected a Pose2D or Pose3D for pose key."
            self.pose_map[key] = variable

    @property
    def all_variable_map(self) -> Dict[Key, Union[Pose2D, Pose3D, Point2D, Point3D]]:
        """
        Get a list of all estimated variables (poses and points).

        Returns:
            A list of all estimated Pose2D, Pose3D, Point2D, and Point3D.
        """
        return {**self.pose_map, **self.point_map}