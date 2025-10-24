"""
Covariance types for measurements and variables.
"""
from attrs import define, field, validators
import numpy as np
from numpy import ndarray
from ..utils.validation import bound_validator
from typing import Union


@define
class RelPoseCovar6:
    """
    A class to represent a 6D covariance matrix for relative pose measurements (3D).
    """

    relative_pose_sigma_x: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in x direction"},
    )
    relative_pose_sigma_y: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in y direction"},
    )
    relative_pose_sigma_z: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in z direction"},
    )
    relative_pose_sigma_psi: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in psi (yaw) direction"},
    )
    relative_pose_rho_xy: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and y"},
    )
    relative_pose_rho_xpsi: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and psi"},
    )
    relative_pose_rho_ypsi: float = field(
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

    def __str__(self) -> str:
        return (
            f"RelPoseCovar6(sigma_x={self.relative_pose_sigma_x}, "
            f"sigma_y={self.relative_pose_sigma_y}, "
            f"sigma_z={self.relative_pose_sigma_z}, "
            f"sigma_psi={self.relative_pose_sigma_psi}, "
            f"rho_xy={self.relative_pose_rho_xy}, "
            f"rho_xpsi={self.relative_pose_rho_xpsi}, "
            f"rho_ypsi={self.relative_pose_rho_ypsi})"
        )

    @property
    def covariance_matrix(self) -> ndarray:
        """
        Returns the covariance matrix as a 6x6 numpy array.
        """
        covar = np.zeros((6, 6))
        covar[0, 0] = self.relative_pose_sigma_x**2
        covar[1, 1] = self.relative_pose_sigma_y**2
        covar[2, 2] = self.relative_pose_sigma_z**2

        covar[3, 3] = covar[4, 4] = covar[5, 5] = self.relative_pose_sigma_psi**2

        covar[0, 1] = covar[1, 0] = self.relative_pose_rho_xy * (
            self.relative_pose_sigma_x * self.relative_pose_sigma_y
        )
        covar[0, 3] = covar[3, 0] = self.relative_pose_rho_xpsi * (
            self.relative_pose_sigma_x * self.relative_pose_sigma_psi
        )
        covar[1, 3] = covar[3, 1] = self.relative_pose_rho_ypsi * (
            self.relative_pose_sigma_y * self.relative_pose_sigma_psi
        )

        return covar


@define
class RelPoseCovar3:
    """
    A class to represent a 3D covariance matrix for relative pose measurements (2D).
    """

    relative_pose_sigma_x: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in x direction"},
    )
    relative_pose_sigma_y: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in y direction"},
    )
    relative_pose_sigma_psi: float = field(
        validator=validators.ge(0.0),
        metadata={"description": "Standard deviation in psi (yaw) direction"},
    )
    relative_pose_rho_xy: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and y"},
    )
    relative_pose_rho_xpsi: float = field(
        validator=bound_validator(-1.0, 1.0),
        metadata={"description": "Correlation between x and psi"},
    )
    relative_pose_rho_ypsi: float = field(
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

    def __str__(self) -> str:
        return (
            f"RelPoseCovar3(sigma_x={self.relative_pose_sigma_x}, "
            f"sigma_y={self.relative_pose_sigma_y}, "
            f"sigma_psi={self.relative_pose_sigma_psi}, "
            f"rho_xy={self.relative_pose_rho_xy}, "
            f"rho_xpsi={self.relative_pose_rho_xpsi}, "
            f"rho_ypsi={self.relative_pose_rho_ypsi})"
        )

    @property
    def covariance_matrix(self) -> ndarray:
        """
        Returns the covariance matrix as a 3x3 numpy array.
        """
        covar = np.zeros((3, 3))
        covar[0, 0] = self.relative_pose_sigma_x**2
        covar[1, 1] = self.relative_pose_sigma_y**2
        covar[2, 2] = self.relative_pose_sigma_psi**2

        covar[0, 1] = covar[1, 0] = self.relative_pose_rho_xy * (
            self.relative_pose_sigma_x * self.relative_pose_sigma_y
        )
        covar[0, 2] = covar[2, 0] = self.relative_pose_rho_xpsi * (
            self.relative_pose_sigma_x * self.relative_pose_sigma_psi
        )
        covar[1, 2] = covar[2, 1] = self.relative_pose_rho_ypsi * (
            self.relative_pose_sigma_y * self.relative_pose_sigma_psi
        )

        return covar


def get_diag_relpose_covar(diags: ndarray) -> Union[RelPoseCovar6, RelPoseCovar3]:
    """Creates a RelPoseCovar6 or RelPoseCovar3 object from the diagonal elements.

    Args:
        diags: the diagonal elements of the covariance matrix

    Returns:
        The corresponding covariance object
    """
    length = diags.shape[0]
    assert length in [3, 6], f"diags must be of length 3 or 6, got {length}"

    if length == 6:
        # check that the last 3 elements are the same
        assert np.allclose(
            diags[3:], diags[3]
        ), "Last 3 elements must be the same for RelPoseCovar6"
        return RelPoseCovar6(
            relative_pose_sigma_x=diags[0],
            relative_pose_sigma_y=diags[1],
            relative_pose_sigma_z=diags[2],
            relative_pose_sigma_psi=diags[3],
            relative_pose_rho_xy=0.0,
            relative_pose_rho_xpsi=0.0,
            relative_pose_rho_ypsi=0.0,
        )
    else:
        return RelPoseCovar3(
            relative_pose_sigma_x=diags[0],
            relative_pose_sigma_y=diags[1],
            relative_pose_sigma_psi=diags[2],
            relative_pose_rho_xy=0.0,
            relative_pose_rho_xpsi=0.0,
            relative_pose_rho_ypsi=0.0,
        )
