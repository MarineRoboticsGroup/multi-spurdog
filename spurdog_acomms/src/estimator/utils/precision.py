"""
Precision and information matrix utilities for measurements.
"""
import numpy as np
import numpy.linalg as la
from typing import Tuple, Optional
from numpy import ndarray

from .validation import _check_square


def get_measurement_precisions_from_info_matrix(
    info_mat: ndarray, matrix_dim: Optional[int] = None
) -> Tuple[float, float]:
    """
    Computes the optimal measurement precisions (assuming isotropic noise) 
    from the information matrix.

    Based on SE-Sync:SESync_utils.cpp:113-124

    Args:
        info_mat: the information matrix
        matrix_dim: the information matrix dimension

    Returns:
        (translation precision, rotation precision)
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
    covar_mat: ndarray, matrix_dim: Optional[int] = None
) -> Tuple[float, float]:
    """
    Computes the optimal measurement precisions (assuming isotropic noise) 
    from the covariance matrix.

    Based on SE-Sync:SESync_utils.cpp:113-124

    Args:
        covar_mat: the covariance matrix
        matrix_dim: the covariance matrix dimension

    Returns:
        (translation precision, rotation precision)

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
    """
    Converts the translation covariance and rotation covariance to measurement
    precisions (assuming isotropic noise).

    Args:
        trans_cov: translation measurement covariance
        rot_cov: rotation measurement covariance
        mat_dim: matrix dimension (3 for 2D, 6 for 3D)

    Returns:
        (translation precision, rotation precision)
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
) -> ndarray:
    """
    Computes the information matrix from measurement precisions (assuming isotropic noise).

    Args:
        trans_precision: the translation precision
        rot_precision: the rotation precision
        mat_dim: the matrix dimension (3 for 2D, 6 for 3D)

    Returns:
        the information matrix
    """
    assert mat_dim in [3, 6], f"Only support 3x3 or 6x6 info matrices"
    if mat_dim == 3:
        trans_info = [trans_precision] * 2
        rot_info = [rot_precision]
    elif mat_dim == 6:
        trans_info = [trans_precision] * 3
        rot_info = [2 * rot_precision] * 3
    else:
        raise ValueError(f"Invalid dimension: {mat_dim}")
    info_mat = np.diag(trans_info + rot_info)
    return info_mat


def get_covariance_matrix_from_measurement_precisions(
    trans_precision: float, rot_precision: float, mat_dim: int
) -> ndarray:
    """
    Computes the covariance matrix from measurement precisions (assuming isotropic noise).

    Args:
        trans_precision: the translation precision
        rot_precision: the rotation precision
        mat_dim: the matrix dimension (3 for 2D, 6 for 3D)

    Returns:
        the covariance matrix
    """
    info_mat = get_info_matrix_from_measurement_precisions(
        trans_precision, rot_precision, mat_dim
    )
    cov_mat = la.inv(info_mat)
    return cov_mat
