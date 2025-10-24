"""
Measurement types for the estimator.
"""
from abc import ABC
from attrs import define, field, validators
from typing import Optional, Tuple, Union
import numpy as np
from numpy import ndarray

from .key import Key, KeyPair
from .covariance import RelPoseCovar3, RelPoseCovar6
from ..utils.validation import tuple_length_validator, quaternion_validator, bound_validator
from ..utils.transformations import get_rotation_matrix_from_theta, get_rotation_matrix_from_quat


@define
class PairMeasurement(ABC):
    """
    Base class for measurements between two variables.
    """

    key_pair: KeyPair = field(
        validator=validators.instance_of(KeyPair),
        metadata={"description": "The keys identifying the two points"},
    )

    @property
    def key1(self) -> Key:
        """Returns the first key from the key pair."""
        return self.key_pair.key1

    @property
    def key2(self) -> Key:
        """Returns the second key from the key pair."""
        return self.key_pair.key2


@define
class RangeMeasurement(PairMeasurement):
    """
    Range measurement between two variables.
    """

    distance: float = field(
        validator=validators.gt(0.0),
        metadata={"description": "The range measurement value"},
    )
    variance: float = field(
        validator=validators.gt(0.0),
        metadata={"description": "Variance of the range measurement"},
    )
    depth1: Optional[float] = field(
        default=None,
        metadata={"description": "Depth of the first point (optional)"},
    )
    depth2: Optional[float] = field(
        default=None,
        metadata={"description": "Depth of the second point (optional)"},
    )

    def __attrs_post_init__(self):
        # if one depth is provided, both should be provided
        if (self.depth1 is None) != (self.depth2 is None):
            raise ValueError(
                "Both depths must be provided or both must be None. "
                f"Got depth1={self.depth1}, depth2={self.depth2}."
            )

        # first key should be a vehicle, second key can be vehicle or landmark
        if self.key1.is_landmark:
            raise ValueError(f"First key in range measurement cannot be a landmark: {self.key1}")

    def __repr__(self) -> str:
        return f"Range({self.key_pair}, distance={self.distance})"

    @property
    def is_2d(self) -> bool:
        """Returns True if the range measurement is 2D (depths not provided)."""
        return self.depth1 is None and self.depth2 is None

    @property
    def precision(self) -> float:
        """Returns the precision (inverse of the variance)."""
        return 1.0 / self.variance


@define
class OdometryMeasurement3D(PairMeasurement):
    """
    3D odometry measurement between two pose variables.
    """

    relative_translation: Tuple[float, float, float] = field(
        validator=tuple_length_validator(3),
        metadata={"description": "Translation vector (x, y, z)"},
    )
    relative_rotation: Tuple[float, float, float, float] = field(
        validator=quaternion_validator(),
        metadata={"description": "Rotation quaternion (x, y, z, w)"},
    )
    covariance: RelPoseCovar6 = field(
        metadata={"description": "Covariance of the odometry measurement"}
    )

    def __attrs_post_init__(self):
        if self.key1.is_landmark or self.key2.is_landmark:
            raise ValueError(f"OdometryMeasurement3D cannot have landmark keys: {self.key_pair}")

    def __repr__(self) -> str:
        return f"Odom3D({self.key_pair})"

    @property
    def rotation_matrix(self) -> ndarray:
        """Returns the rotation matrix from the quaternion."""
        return get_rotation_matrix_from_quat(np.array(self.relative_rotation))

    @property
    def translation_vector(self) -> ndarray:
        """Returns the translation vector as a numpy array."""
        return np.array(self.relative_translation)

    @property
    def transformation_matrix(self) -> ndarray:
        """Returns the 4x4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.rotation_matrix
        T[:3, 3] = np.array(self.relative_translation)
        return T

    @property
    def rotation_precision(self) -> float:
        """Returns the rotation precision."""
        from ..utils.precision import get_measurement_precisions_from_covariance_matrix
        _, rot_precision = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=6
        )
        return rot_precision

    @property
    def translation_precision(self) -> float:
        """Returns the translation precision."""
        from ..utils.precision import get_measurement_precisions_from_covariance_matrix
        trans_precision, _ = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=6
        )
        return trans_precision


@define
class OdometryMeasurement2D(PairMeasurement):
    """
    2D odometry measurement between two pose variables.
    """

    relative_translation: Tuple[float, float] = field(
        validator=tuple_length_validator(2),
        metadata={"description": "Translation vector (x, y)"},
    )
    relative_rotation: float = field(
        validator=bound_validator(-np.pi, np.pi),
        metadata={"description": "Rotation angle (psi) in radians"},
    )
    covariance: RelPoseCovar3 = field(
        metadata={"description": "Covariance of the odometry measurement"}
    )

    def __attrs_post_init__(self):
        if self.key1.is_landmark or self.key2.is_landmark:
            raise ValueError(f"OdometryMeasurement2D cannot have landmark keys: {self.key_pair}")

    def __repr__(self) -> str:
        return f"Odom2D({self.key_pair})"

    @property
    def rotation_matrix(self) -> ndarray:
        """Returns the 2x2 rotation matrix."""
        return get_rotation_matrix_from_theta(self.relative_rotation)

    @property
    def translation_vector(self) -> ndarray:
        """Returns the translation vector as a numpy array."""
        return np.array(self.relative_translation)

    @property
    def transformation_matrix(self) -> ndarray:
        """Returns the 3x3 homogeneous transformation matrix."""
        T = np.eye(3)
        T[:2, :2] = self.rotation_matrix
        T[:2, 2] = np.array(self.relative_translation)
        return T

    @property
    def rotation_precision(self) -> float:
        """Returns the rotation precision."""
        from ..utils.precision import get_measurement_precisions_from_covariance_matrix
        _, rot_precision = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=3
        )
        return rot_precision

    @property
    def translation_precision(self) -> float:
        """Returns the translation precision."""
        from ..utils.precision import get_measurement_precisions_from_covariance_matrix
        trans_precision, _ = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=3
        )
        return trans_precision

    @property
    def x(self) -> float:
        """Returns the x component of the translation."""
        return self.relative_translation[0]

    @property
    def y(self) -> float:
        """Returns the y component of the translation."""
        return self.relative_translation[1]

    @property
    def psi(self) -> float:
        """Returns the rotation angle in radians."""
        return self.relative_rotation


# Type alias for any odometry measurement
OdometryMeasurement = Union[OdometryMeasurement2D, OdometryMeasurement3D]


@define
class DepthMeasurement:
    """
    Depth measurement for a variable.
    """

    key: Key = field(
        validator=validators.instance_of(Key),
        metadata={"description": "The key identifying the variable"},
    )
    depth: float = field(
        validator=validators.gt(0.0),
        metadata={"description": "The depth measurement value"},
    )
    variance: float = field(
        validator=validators.gt(0.0),
        metadata={"description": "Variance of the depth measurement"},
    )
