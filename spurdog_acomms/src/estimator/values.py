from .types.key import Key
from .types.variables import Pose2D, Pose3D, Point2D, Point3D
from .utils.validation import _check_valid_key
from attrs import define, field
import numpy as np
from typing import Dict, List, Optional, Union, Set

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
            val_before = self.point_map.get(key, None)
            self.point_map[key] = variable
        else:
            assert isinstance(variable, (Pose2D, Pose3D)), "Expected a Pose2D or Pose3D for pose key."
            val_before = self.pose_map.get(key, None)
            self.pose_map[key] = variable

        # print(f"Updated variable for key {key}. Previous value: {val_before}, New value: {variable}")

    @property
    def all_variable_map(self) -> Dict[Key, Union[Pose2D, Pose3D, Point2D, Point3D]]:
        """
        Get a list of all estimated variables (poses and points).

        Returns:
            A list of all estimated Pose2D, Pose3D, Point2D, and Point3D.
        """
        return {**self.pose_map, **self.point_map}

    @property
    def all_keys(self) -> Set[Key]:
        """
        Get a set of all keys in the current estimates.

        Returns:
            A set of all keys.
        """
        return set(self.pose_map.keys()).union(set(self.point_map.keys()))