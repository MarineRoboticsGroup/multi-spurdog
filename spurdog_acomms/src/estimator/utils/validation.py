"""
Validation utilities for estimator types.
"""
from typing import Optional
from attrs import validators
import numpy as np


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


def _check_symmetric(mat: np.ndarray) -> None:
    """Checks that a matrix is symmetric"""
    assert np.allclose(mat, mat.T)


def _check_transformation_matrix(
    T: np.ndarray, assert_test: bool = True, dim: Optional[int] = None
) -> None:
    """Checks that the matrix passed in is a homogeneous transformation matrix.
    
    Args:
        T: the homogeneous transformation matrix to test
        assert_test: Whether this is a 'hard' test with assertions or 'soft' test
        dim: dimension of the homogeneous transformation matrix
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

    # check that the bottom row is [0, 0, ..., 1]
    bottom = T[-1, :]
    bottom_expected = np.array([0] * (matrix_dim - 1) + [1])
    assert np.allclose(
        bottom.flatten(), bottom_expected
    ), f"Transformation matrix bottom row is {bottom} but should be {bottom_expected}"


def _check_rotation_matrix(R: np.ndarray, assert_test: bool = False) -> None:
    """
    Checks that R is a rotation matrix.

    Args:
        R: the candidate rotation matrix
        assert_test: if false just print if not rotation matrix, otherwise raise error

    Raises:
        ValueError: the candidate rotation matrix is not orthogonal
        ValueError: the candidate rotation matrix determinant is incorrect
    """
    d = R.shape[0]
    is_orthogonal = np.allclose(R @ R.T, np.eye(d), rtol=1e-3, atol=1e-3)
    if not is_orthogonal:
        if assert_test:
            raise ValueError(f"R is not orthogonal {R @ R.T}")

    has_correct_det = abs(np.linalg.det(R) - 1) < 1e-3
    if not has_correct_det:
        if assert_test:
            raise ValueError(f"R det incorrect {np.linalg.det(R)}")


def _check_valid_key(instance, attribute, key: str) -> None:
    """
    Checks that a key is valid (non-empty string starting with a capital letter followed by numbers).

    Args:
        key: the key to check
    Raises:
        ValueError: if the key is not valid
    """
    first_is_cap_letter = key[0].isupper()
    rest_are_numbers = key[1:].isdigit()
    if not (first_is_cap_letter and rest_are_numbers):
        raise ValueError(f"Invalid key: {key}")
