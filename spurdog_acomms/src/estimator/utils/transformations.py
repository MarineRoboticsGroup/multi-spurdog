"""
Transformation matrix utilities for pose and rotation conversions.
"""
import numpy as np
import scipy.spatial.transform
from .validation import _check_square, _check_rotation_matrix


def get_theta_from_transformation_matrix(T: np.ndarray) -> float:
    """Returns the angle theta from a 2D transformation matrix.

    Args:
        T: the 3x3 transformation matrix

    Returns:
        the angle theta in radians
    """
    assert T.shape == (3, 3), "transformation matrix must be 3x3"
    return get_theta_from_rotation_matrix(
        get_rotation_matrix_from_transformation_matrix(T)
    )


def get_theta_from_rotation_matrix(mat: np.ndarray) -> float:
    """Returns theta from a 2D rotation matrix.

    Args:
        mat: the 2x2 rotation matrix

    Returns:
        theta in radians
    """
    _check_square(mat)
    assert mat.shape == (2, 2)
    return float(np.arctan2(mat[1, 0], mat[0, 0]))


def get_rotation_matrix_from_transformation_matrix(T: np.ndarray) -> np.ndarray:
    """Returns the rotation matrix from the transformation matrix.

    Args:
        T: the transformation matrix

    Returns:
        the rotation matrix
    """
    _check_square(T)
    dim = T.shape[0] - 1
    return T[:dim, :dim]


def get_translation_from_transformation_matrix(T: np.ndarray) -> np.ndarray:
    """Returns the translation from a transformation matrix.

    Args:
        T: the transformation matrix

    Returns:
        the translation vector
    """
    _check_square(T)
    dim = T.shape[0] - 1
    return T[:dim, dim]


def get_rotation_matrix_from_theta(theta: float) -> np.ndarray:
    """Returns the 2D rotation matrix from theta.

    Args:
        theta: the angle of rotation in radians

    Returns:
        2x2 rotation matrix
    """
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def get_rotation_matrix_from_quat(quat: np.ndarray) -> np.ndarray:
    """Returns the rotation matrix from a quaternion in scalar-last (x, y, z, w) format.

    Args:
        quat: the quaternion as (x, y, z, w)

    Returns:
        3x3 rotation matrix
    """
    assert quat.shape == (4,)
    rot = scipy.spatial.transform.Rotation.from_quat(quat)
    assert isinstance(rot, scipy.spatial.transform.Rotation)

    rot_mat = rot.as_matrix()
    assert isinstance(rot_mat, np.ndarray)
    assert rot_mat.shape == (3, 3)

    _check_rotation_matrix(rot_mat, assert_test=True)
    return rot_mat


def get_quat_from_rotation_matrix(mat: np.ndarray) -> np.ndarray:
    """Returns the quaternion from a rotation matrix in scalar-last (x, y, z, w) format.
    Ensures w is positive by convention, given R(-q) = R(q).

    Args:
        mat: the rotation matrix (2x2 or 3x3)

    Returns:
        quaternion as (x, y, z, w)
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

    # Ensure positive w by convention
    if quat[-1] < 0:
        quat = np.negative(quat)

    return quat


def get_quat_from_theta(theta: float) -> np.ndarray:
    """Returns the quaternion from theta in scalar-last (x, y, z, w) format.

    Args:
        theta: the angle of rotation in radians

    Returns:
        quaternion as (x, y, z, w)
    """
    R = np.eye(3)
    R[:2, :2] = get_rotation_matrix_from_theta(theta)
    return get_quat_from_rotation_matrix(R)
