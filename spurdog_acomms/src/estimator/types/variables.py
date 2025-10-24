"""
Variable types for the estimator (Poses and Points).
"""
from attrs import define, field, validators
from typing import Optional, Tuple
import numpy as np
from numpy import ndarray

from .key import Key
from ..utils.validation import tuple_length_validator, quaternion_validator, bound_validator, _check_symmetric
from ..utils.transformations import get_rotation_matrix_from_theta, get_rotation_matrix_from_quat


@define
class Pose2D:
    """
    2D pose with position (x, y) and orientation (theta).
    """

    key: Key = field(
        validator=validators.instance_of(Key),
        metadata={"description": "The key identifying the pose variable"},
    )
    position: Tuple[float, float] = field(
        validator=tuple_length_validator(2),
        metadata={"description": "The position (x, y) of the pose"},
    )
    orientation: float = field(
        validator=bound_validator(-np.pi, np.pi),
        metadata={"description": "The orientation (psi) of the pose in radians"},
    )
    marginal_covariance: Optional[ndarray] = field(
        default=None,
        validator=validators.optional(validators.instance_of(ndarray)),
        metadata={"description": "Marginal covariance of the pose variable (optional)"},
    )

    @property
    def transformation_matrix(self) -> ndarray:
        """Returns the 3x3 homogeneous transformation matrix."""
        theta = self.orientation
        T = np.eye(3)
        T[:2, :2] = get_rotation_matrix_from_theta(theta)
        T[:2, 2] = np.array(self.position)
        return T


@define
class Pose3D:
    """
    3D pose with position (x, y, z) and orientation (quaternion).
    """

    key: Key = field(
        validator=validators.instance_of(Key),
        metadata={"description": "The key identifying the pose variable"},
    )
    position: Tuple[float, float, float] = field(
        validator=tuple_length_validator(3),
        metadata={"description": "The position (x, y, z) of the pose"},
    )
    orientation: Tuple[float, float, float, float] = field(
        validator=quaternion_validator(),
        metadata={"description": "The orientation quaternion (x, y, z, w) of the pose"},
    )
    marginal_covariance: Optional[ndarray] = field(
        default=None,
        validator=validators.optional(validators.instance_of(ndarray)),
        metadata={"description": "Marginal covariance of the pose variable (optional)"},
    )

    @property
    def transformation_matrix(self) -> ndarray:
        """Returns the 4x4 homogeneous transformation matrix."""
        R = get_rotation_matrix_from_quat(np.array(self.orientation))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array(self.position)
        return T

    def __attrs_post_init__(self):
        """Validate marginal covariance if provided."""
        if self.marginal_covariance is not None:
            if self.marginal_covariance.shape != (6, 6):
                raise ValueError(
                    "Marginal covariance must be a 6x6 matrix if provided."
                )
            _check_symmetric(self.marginal_covariance)
            if not np.all(np.linalg.eigvals(self.marginal_covariance) > 0):
                raise ValueError("Marginal covariance must be positive definite.")


@define
class Point2D:
    """
    2D point with position (x, y).
    """

    key: Key = field(
        validator=validators.instance_of(Key),
        metadata={"description": "The key identifying the point variable"},
    )
    position: Tuple[float, float] = field(
        validator=tuple_length_validator(2),
        metadata={"description": "The position (x, y) of the point"},
    )
    marginal_covariance: Optional[ndarray] = field(
        default=None,
        validator=validators.optional(validators.instance_of(ndarray)),
        metadata={
            "description": "Marginal covariance of the point variable (optional)"
        },
    )


@define
class Point3D:
    """
    3D point with position (x, y, z).
    """

    key: Key = field(
        validator=validators.instance_of(Key),
        metadata={"description": "The key identifying the point variable"},
    )
    position: Tuple[float, float, float] = field(
        validator=tuple_length_validator(3),
        metadata={"description": "The position (x, y, z) of the point"},
    )
    marginal_covariance: Optional[ndarray] = field(
        default=None,
        validator=validators.optional(validators.instance_of(ndarray)),
        metadata={
            "description": "Marginal covariance of the point variable (optional)"
        },
    )
