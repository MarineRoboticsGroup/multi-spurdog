from enum import Enum
from abc import ABC, abstractmethod
from attrs import define, field, validators
from typing import Optional, Tuple, Union
import numpy as np
from numpy import ndarray
import scipy.spatial.transform

Key = str  # Assuming Key is a string for simplicity, can be replaced with a more complex type if needed

#### Validators ####


def bound_validator(a: float, b: float):
    """
    Returns a validator that checks if a value is within the bounds [a, b].
    """
    return validators.and_(validators.ge(a), validators.le(b))


def tuple_length_validator(length: int):
    """
    Returns a validator that checks if a value is a tuple of a specific length.
    """

    def _validator(instance, attribute, value):
        if not isinstance(value, tuple):
            raise TypeError(f"{attribute.name} must be a tuple.")
        if len(value) != length:
            raise ValueError(f"{attribute.name} must have {length} elements.")

    return _validator


def quaternion_validator():
    """
    Validates if a quaternion is normalized (i.e., its norm is close to 1).
    """

    def _validator(instance, attribute, quat):
        if not isinstance(quat, tuple):
            raise TypeError(f"{attribute.name} must be a tuple.")

        if len(quat) != 4:
            raise ValueError(f"{attribute.name} must have 4 elements.")
        norm = sum(x**2 for x in quat)
        if not np.isclose(norm, 1.0):
            raise ValueError(f"{attribute.name} is not normalized. Norm: {norm}")

    return _validator


def _check_square(mat: np.ndarray) -> None:
    """Checks that a matrix is square"""
    assert mat.shape[0] == mat.shape[1], "matrix must be square"


def _check_symmetric(mat) -> None:
    """Checks that a matrix is symmetric"""
    assert np.allclose(mat, mat.T)


def _check_transformation_matrix(
    T: np.ndarray, assert_test: bool = True, dim: Optional[int] = None
) -> None:
    """Checks that the matrix passed in is a homogeneous transformation matrix.
    If assert_test is True, then this is in the form of assertions, otherwise we
    just print out error messages but continue

    Args:
        T (np.ndarray): the homogeneous transformation matrix to test
        assert_test (bool, optional): Whether this is a 'hard' test and is
        asserted or just a 'soft' test and only prints message if test fails. Defaults to True.
        dim (Optional[int] = None): dimension of the homogeneous transformation matrix
    """
    _check_square(T)
    matrix_dim = T.shape[0]
    if dim is not None:
        assert (
            matrix_dim == dim + 1
        ), f"matrix dimension {matrix_dim} != dim + 1 {dim + 1}"

    assert matrix_dim in [
        3,
        4,
    ], f"Was {T.shape} but must be 3x3 or 4x4 for a transformation matrix"

    # check that is rotation matrix in upper left block
    R = T[:-1, :-1]
    _check_rotation_matrix(R, assert_test=assert_test)

    # check that the bottom row is [0, 0, 1]
    bottom = T[-1, :]
    bottom_expected = np.array([0] * (matrix_dim - 1) + [1])
    assert np.allclose(
        bottom.flatten(), bottom_expected
    ), f"Transformation matrix bottom row is {bottom} but should be {bottom_expected}"


def _check_rotation_matrix(R: np.ndarray, assert_test: bool = False) -> None:
    """
    Checks that R is a rotation matrix.

    Args:
        R (np.ndarray): the candidate rotation matrix
        assert_test (bool, optional): if false just print if not rotation matrix, otherwise raise error

    Raises:
        ValueError: the candidate rotation matrix is not orthogonal
        ValueError: the candidate rotation matrix determinant is incorrect
    """
    d = R.shape[0]
    is_orthogonal = np.allclose(R @ R.T, np.eye(d), rtol=1e-3, atol=1e-3)
    if not is_orthogonal:
        # print(f"R not orthogonal: {R @ R.T}")
        if assert_test:
            raise ValueError(f"R is not orthogonal {R @ R.T}")

    has_correct_det = abs(np.linalg.det(R) - 1) < 1e-3
    if not has_correct_det:
        # print(f"R det != 1: {np.linalg.det(R)}")
        if assert_test:
            raise ValueError(f"R det incorrect {np.linalg.det(R)}")


#### Data Classes ###


@define
class RelPoseCovar6:
    """
    A class to represent a 6D covariance matrix, likely for relative pose measurements
    """

    relative_pose_1_sigma_x: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in x direction"},
    )
    relative_pose_1_sigma_y: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in y direction"},
    )
    relative_pose_1_sigma_z: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in z direction"},
    )
    relative_pose_1_sigma_psi: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in psi (yaw) direction"},
    )
    relative_pose_1_rho_xy: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and y"},
    )
    relative_pose_1_rho_xpsi: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and psi"},
    )
    relative_pose_1_rho_ypsi: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between y and psi"},
    )

    def __attrs_post_init__(self):
        matrix = self.covariance_matrix
        if not np.allclose(matrix, matrix.T):
            raise ValueError("Covariance matrix must be symmetric.")
        if not np.all(np.linalg.eigvals(matrix) > 0):
            raise ValueError(
                f"Covariance matrix must be positive definite. Eigenvalues: {np.linalg.eigvals(matrix)}"
            )

    def __str__(self):
        return (
            f"RelPoseCovar6(sigma_x={self.relative_pose_1_sigma_x}, "
            f"sigma_y={self.relative_pose_1_sigma_y}, "
            f"sigma_z={self.relative_pose_1_sigma_z}, "
            f"sigma_psi={self.relative_pose_1_sigma_psi}, "
            f"rho_xy={self.relative_pose_1_rho_xy}, "
            f"rho_xpsi={self.relative_pose_1_rho_xpsi}, "
            f"rho_ypsi={self.relative_pose_1_rho_ypsi})"
        )

    @property
    def covariance_matrix(self) -> ndarray:
        """
        Returns the covariance matrix as a 6x6 numpy array.
        """
        covar = np.zeros((6, 6))
        covar[0, 0] = self.relative_pose_1_sigma_x**2
        covar[1, 1] = self.relative_pose_1_sigma_y**2
        covar[2, 2] = self.relative_pose_1_sigma_z**2
        covar[3, 3] = self.relative_pose_1_sigma_psi**2

        covar[0, 1] = covar[1, 0] = self.relative_pose_1_rho_xy * (
            self.relative_pose_1_sigma_x * self.relative_pose_1_sigma_y
        )
        covar[0, 3] = covar[3, 0] = self.relative_pose_1_rho_xpsi * (
            self.relative_pose_1_sigma_x * self.relative_pose_1_sigma_psi
        )
        covar[1, 3] = covar[3, 1] = self.relative_pose_1_rho_ypsi * (
            self.relative_pose_1_sigma_y * self.relative_pose_1_sigma_psi
        )

        return covar


@define
class RelPoseCovar3:
    """
    A class to represent a 3D covariance matrix, likely for relative pose measurements
    """

    relative_pose_1_sigma_x: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in x direction"},
    )
    relative_pose_1_sigma_y: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in y direction"},
    )
    relative_pose_1_sigma_psi: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in psi (yaw) direction"},
    )
    relative_pose_1_rho_xy: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and y"},
    )
    relative_pose_1_rho_xpsi: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and psi"},
    )
    relative_pose_1_rho_ypsi: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between y and psi"},
    )

    def __attrs_post_init__(self):
        matrix = self.covariance_matrix
        if not np.allclose(matrix, matrix.T):
            raise ValueError("Covariance matrix must be symmetric.")
        if not np.all(np.linalg.eigvals(matrix) > 0):
            raise ValueError(
                f"Covariance matrix must be positive definite. Eigenvalues: {np.linalg.eigvals(matrix)}"
            )

    def __str__(self):
        return (
            f"RelPoseCovar3(sigma_x={self.relative_pose_1_sigma_x}, "
            f"sigma_y={self.relative_pose_1_sigma_y}, "
            f"sigma_psi={self.relative_pose_1_sigma_psi}, "
            f"rho_xy={self.relative_pose_1_rho_xy}, "
            f"rho_xpsi={self.relative_pose_1_rho_xpsi}, "
            f"rho_ypsi={self.relative_pose_1_rho_ypsi})"
        )

    @property
    def covariance_matrix(self) -> ndarray:
        """
        Returns the covariance matrix as a 3x3 numpy array.
        """
        covar = np.zeros((3, 3))
        covar[0, 0] = self.relative_pose_1_sigma_x**2
        covar[1, 1] = self.relative_pose_1_sigma_y**2
        covar[2, 2] = self.relative_pose_1_sigma_psi**2

        covar[0, 1] = covar[1, 0] = self.relative_pose_1_rho_xy * (
            self.relative_pose_1_sigma_x * self.relative_pose_1_sigma_y
        )
        covar[0, 2] = covar[2, 0] = self.relative_pose_1_rho_xpsi * (
            self.relative_pose_1_sigma_x * self.relative_pose_1_sigma_psi
        )
        covar[1, 2] = covar[2, 1] = self.relative_pose_1_rho_ypsi * (
            self.relative_pose_1_sigma_y * self.relative_pose_1_sigma_psi
        )

        return covar


class EstimatorMode(Enum):
    GTSAM_LM = 1
    GTSAM_DOG = 2
    CORA = 3


@define
class KeyPair:
    """
    A class to represent a pair of keys.
    It is used to identify two variables (typically connected by a measurement)
    """

    key1: Key = field(metadata={"description": "The first key"})
    key2: Key = field(metadata={"description": "The second key"})

    def __str__(self):
        return f"KeyPair({self.key1}, {self.key2})"


##### Measurement Classes #####


@define
class PairMeasurement(ABC):
    """
    A class to represent a pair measurement.
    It includes the keys identifying the two points and the value of the measurement.
    """

    key_pair: KeyPair = field(
        validator=validators.instance_of(KeyPair),
        metadata={"description": "The keys identifying the two points"},
    )

    @property
    def key1(self) -> Key:
        """
        Returns the first key from the key pair.
        """
        return self.key_pair.key1

    @property
    def key2(self) -> Key:
        """
        Returns the second key from the key pair.
        """
        return self.key_pair.key2


@define
class RangeMeasurement(PairMeasurement):
    """
    A class to represent a range measurement.
    It includes the range value and optionally the depths of the two points between which the range is measured.
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
        validator=validators.optional(validators.ge(0.0)),
        metadata={"description": "Depth of the first point (optional)"},
    )
    depth2: Optional[float] = field(
        default=None,
        validator=validators.optional(validators.ge(0.0)),
        metadata={"description": "Depth of the second point (optional)"},
    )

    def __attrs_post_init__(self):
        # if one depth is provided, both should be provided
        if (self.depth1 is None) != (self.depth2 is None):
            raise ValueError(
                "Both depths must be provided or both must be None. "
                f"Got depth1={self.depth1}, depth2={self.depth2}."
            )

    @property
    def is_2d(self) -> bool:
        """
        Returns True if the range measurement is 2D (i.e., depth1 and depth2 are None).
        """
        return self.depth1 is None and self.depth2 is None


@define
class OdometryMeasurement3D(PairMeasurement):
    """
    A class to represent an odometry measurement.
    It includes the translation and rotation values.
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
        metadata={"description": "Covariance of the odometry measurement (optional)"}
    )

    @property
    def rotation_matrix(self) -> ndarray:
        return get_rotation_matrix_from_quat(np.array(self.relative_rotation))

    @property
    def translation_vector(self) -> ndarray:
        """
        Returns the translation vector as a numpy array.
        """
        return np.array(self.relative_translation)

    @property
    def rotation_precision(self) -> float:
        _, rot_precision = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=6
        )
        return rot_precision

    @property
    def translation_precision(self) -> float:
        """
        Returns the translation precision (inverse of the variance) for the relative translations.
        """
        trans_precision, _ = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=6
        )
        return trans_precision


@define
class OdometryMeasurement2D(PairMeasurement):
    """
    A class to represent an odometry measurement.
    It includes the translation and rotation values.
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
        metadata={"description": "Covariance of the odometry measurement (optional)"}
    )

    @property
    def rotation_matrix(self) -> ndarray:
        """
        Returns the rotation matrix corresponding to the relative rotation.
        """
        theta = self.relative_rotation
        return np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )

    @property
    def translation_vector(self) -> ndarray:
        """
        Returns the translation vector as a numpy array.
        """
        return np.array(self.relative_translation)

    @property
    def rotation_precision(self) -> float:
        _, rot_precision = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=3
        )
        return rot_precision

    @property
    def translation_precision(self) -> float:
        """
        Returns the translation precision (inverse of the variance) for the relative translations.
        """
        trans_precision, _ = get_measurement_precisions_from_covariance_matrix(
            self.covariance.covariance_matrix, matrix_dim=3
        )
        return trans_precision

    @property
    def x(self) -> float:
        """
        Returns the x component of the translation vector.
        """
        return self.relative_translation[0]

    @property
    def y(self) -> float:
        """
        Returns the y component of the translation vector.
        """
        return self.relative_translation[1]

    @property
    def psi(self) -> float:
        """
        Returns the rotation angle (psi) in radians.
        """
        return self.relative_rotation

OdometryMeasurement = Union[
    OdometryMeasurement2D,
    OdometryMeasurement3D,
]


@define
class DepthMeasurement:
    """
    A class to represent a depth measurement.
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


#### Variable Classes ####


@define
class Pose2D:
    """
    A class to represent a pose in 2D space.
    It includes the position (x, y) and orientation (psi).
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


@define
class Pose3D:
    """
    A class to represent a pose in 3D space.
    It includes the position (x, y, z) and orientation (quaternion).
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


@define
class Point2D:
    """
    A class to represent a point in 2D space.
    It includes the position (x, y).
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
    A class to represent a point in 3D space.
    It includes the position (x, y, z).
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


##### Helper Functions ####


def project_range_to_2d(range_measurement: RangeMeasurement) -> RangeMeasurement:
    """
    Considering that a range measurement is taken between two points at different depths (z values),
    this function projects the range measurement to a 2D plane by calculating the horizontal distance (in the xy plane)

    Args:
        range_measurement (RangeMeasurement): The range measurement object containing the range value and optional depths.
    """
    if range_measurement.depth1 is None or range_measurement.depth2 is None:
        raise ValueError(
            "Both depths must be provided to project the range measurement to 2D."
        )

    vertical_distance = abs(range_measurement.depth1 - range_measurement.depth2)
    horizontal_distance = (range_measurement.distance**2 - vertical_distance**2) ** 0.5

    assert (
        horizontal_distance > 0
    ), f"Horizontal distance should be positive, got {horizontal_distance} for range {range_measurement}"

    return RangeMeasurement(
        key_pair=range_measurement.key_pair,
        distance=horizontal_distance,
        variance=range_measurement.variance,
        depth1=None,  # Depths are not needed in 2D projection
        depth2=None,
    )


def get_theta_from_transformation_matrix(T: np.ndarray) -> float:
    """Returns the angle theta from a transformation matrix

    Args:
        T (np.ndarray): the transformation matrix

    Returns:
        float: the angle theta
    """
    assert T.shape == (3, 3), "transformation matrix must be 3x3"
    return get_theta_from_rotation_matrix(
        get_rotation_matrix_from_transformation_matrix(T)
    )


def get_theta_from_rotation_matrix(mat: np.ndarray) -> float:
    """Returns theta from a matrix M

    Args:
        mat (np.ndarray): the candidate rotation matrix

    Returns:
        float: theta
    """
    _check_square(mat)
    assert mat.shape == (2, 2)
    return float(np.arctan2(mat[1, 0], mat[0, 0]))


def get_rotation_matrix_from_transformation_matrix(T: np.ndarray) -> np.ndarray:
    """Returns the rotation matrix from the transformation matrix

    Args:
        T (np.ndarray): the transformation matrix

    Returns:
        np.ndarray: the rotation matrix
    """
    _check_square(T)
    dim = T.shape[0] - 1
    return T[:dim, :dim]


def get_translation_from_transformation_matrix(T: np.ndarray) -> np.ndarray:
    """Returns the translation from a transformation matrix

    Args:
        T (np.ndarray): the transformation matrix

    Returns:
        np.ndarray: the translation
    """
    _check_square(T)
    dim = T.shape[0] - 1
    return T[:dim, dim]


def get_rotation_matrix_from_theta(theta: float) -> np.ndarray:
    """Returns the rotation matrix from theta

    Args:
        theta (float): the angle of rotation

    Returns:
        np.ndarray: the rotation matrix
    """
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def get_rotation_matrix_from_quat(quat: np.ndarray) -> np.ndarray:
    """Returns the rotation matrix from a quaternion in scalar-last (x, y, z, w)
    format.

    Args:
        quat (np.ndarray): the quaternion

    Returns:
        np.ndarray: the rotation matrix
    """
    assert quat.shape == (4,)
    rot = scipy.spatial.transform.Rotation.from_quat(quat)
    assert isinstance(rot, scipy.spatial.transform.Rotation)

    # get as a matrix
    rot_mat = rot.as_matrix()
    assert isinstance(rot_mat, np.ndarray)
    assert rot_mat.shape == (3, 3)

    _check_rotation_matrix(rot_mat, assert_test=True)
    return rot_mat


def get_quat_from_rotation_matrix(mat: np.ndarray) -> np.ndarray:
    """Returns the quaternion from a rotation matrix in scalar-last (x, y, z, w).
    The function ensures w is positive by convention, given R(-q) = R(q)

    Args:
        mat (np.ndarray): the rotation matrix

    Returns:
        np.ndarray: the quaternion
    """
    _check_rotation_matrix(mat)
    mat_dim = mat.shape[0]

    if mat_dim == 2:
        rot_matrix = np.eye(3)
        rot_matrix[:2, :2] = mat
    else:
        rot_matrix = mat

    rot = scipy.spatial.transform.Rotation.from_matrix(rot_matrix)
    assert isinstance(rot, scipy.spatial.transform.Rotation)
    quat = rot.as_quat()
    assert isinstance(quat, np.ndarray)

    if quat[-1] < 0:
        quat = np.negative(quat)

    return quat


def get_measurement_precisions_from_info_matrix(
    info_mat: np.ndarray, matrix_dim: Optional[int] = None
) -> Tuple[float, float]:
    """Computes the optimal measurement precisions (assuming isotropic noise) from the information matrix

    Based on SE-Sync:SESync_utils.cpp:113-124

    Args:
        info_mat (np.ndarray): the information matrix
        matrix_dim (Optional[int] = None): the information matrix dimension

    Returns:
        Tuple[float, float]: (translation precision, rotation precision)
    """
    _check_square(info_mat)
    dim = info_mat.shape[0]
    if matrix_dim is not None:
        assert (
            matrix_dim == dim
        ), f"matrix_dim {matrix_dim} must match info_mat dim {dim}"

    assert dim in [3, 6], "information matrix must be 3x3 or 6x6"
    covar_mat = la.inv(info_mat)
    trans_precision, rot_precision = get_measurement_precisions_from_covariance_matrix(
        covar_mat, matrix_dim
    )
    return (trans_precision, rot_precision)


def get_measurement_precisions_from_covariance_matrix(
    covar_mat: np.ndarray, matrix_dim: Optional[int] = None
) -> Tuple[float, float]:
    """Computes the optimal measurement precisions (assuming isotropic noise) from the covariance matrix

    Based on SE-Sync:SESync_utils.cpp:113-124

    Args:
        covar_mat (np.ndarray): the covariance matrix
        matrix_dim (Optional[int] = None): the covariance matrix dimension

    Returns:
        Tuple[float, float]: (translation precision, rotation precision)

    Raises:
        ValueError: the dimension of the covariance matrix is invalid
    """
    _check_square(covar_mat)
    dim = covar_mat.shape[0]
    if matrix_dim is not None:
        assert (
            matrix_dim == dim
        ), f"matrix_dim {matrix_dim} must match info_mat dim {dim}"

    assert dim in [3, 6], "information matrix must be 3x3 or 6x6"

    def _get_trans_precision() -> float:
        if dim == 3:
            trans_covar = covar_mat[:2, :2]
            trans_precision = 2 / (np.trace(trans_covar))
        elif dim == 6:
            trans_covar = covar_mat[:3, :3]
            trans_precision = 3 / (np.trace(trans_covar))
        else:
            raise ValueError(f"Invalid dimension: {dim}")
        return trans_precision

    def _get_rot_precision() -> float:
        if dim == 3:
            rot_precision = 1 / covar_mat[2, 2]
        elif dim == 6:
            rot_cov = covar_mat[3:, 3:]
            rot_precision = 3 / (2 * np.trace(rot_cov))
        else:
            raise ValueError(f"Invalid dimension: {dim}")
        return rot_precision

    trans_precision = _get_trans_precision()
    rot_precision = _get_rot_precision()
    return trans_precision, rot_precision


def get_measurement_precisions_from_covariances(
    trans_cov: float, rot_cov: float, mat_dim: int
) -> Tuple[float, float]:
    """Converts the translation covariance and rotation covariance to measurement
    precisions (assuming isotropic noise)

    Args:
        trans_cov (float): translation measurement covariance
        rot_cov (float): rotation measurement covariance

    Returns:
        Tuple[float, float]: (translation precision, rotation precision)
    """
    assert mat_dim in [3, 6], f"Only support 3x3 or 6x6 info matrices"
    if mat_dim == 3:
        covars = [trans_cov] * 2 + [rot_cov]
    elif mat_dim == 6:
        covars = [trans_cov] * 3 + [rot_cov] * 3
    else:
        raise ValueError(f"Invalid dimension: {mat_dim}")
    covar_mat = np.diag(covars)
    return get_measurement_precisions_from_covariance_matrix(covar_mat, mat_dim)


def get_info_matrix_from_measurement_precisions(
    trans_precision: float, rot_precision: float, mat_dim: int
) -> np.ndarray:
    """Computes the information matrix from measurement precisions (assuming isotropic noise)

    Args:
        trans_precision (float): the translation precision
        rot_precision (float): the rotation precision
        mat_dim (int): the matrix dimension (3 for 2D, 6 for 3D)

    Returns:
        np.ndarray: the information matrix
    """
    assert mat_dim in [3, 6], f"Only support 3x3 or 6x6 info matrices"
    if mat_dim == 3:
        trans_info = [trans_precision] * 2
        rot_info = [rot_precision]
    if mat_dim == 6:
        trans_info = [trans_precision] * 3
        rot_info = [2 * rot_precision] * 3
    info_mat = np.diag(trans_info + rot_info)
    return info_mat


def get_covariance_matrix_from_measurement_precisions(
    trans_precision: float, rot_precision: float, mat_dim: int
) -> np.ndarray:
    """Computes the covariance matrix from measurement precisions (assuming isotropic noise)

    Args:
        trans_precision (float): the translation precision
        rot_precision (float): the rotation precision
        mat_dim (int): the matrix dimension (3 for 2D, 6 for 3D)

    Returns:
        np.ndarray: the covariance matrix
    """
    info_mat = get_info_matrix_from_measurement_precisions(
        trans_precision, rot_precision, mat_dim
    )
    cov_mat = la.inv(info_mat)
    return cov_mat
