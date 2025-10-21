import gtsam
import numpy as np
from typing import List, Optional
from ..estimator_helpers import get_rotation_matrix_from_theta


def tangent_matrix(omega: float):
    Omega = np.zeros((2, 2))
    Omega[0, 1] = -omega
    Omega[1, 0] = omega
    return Omega


class RelativePose2dFactor(gtsam.CustomFactor):
    def __init__(
        self,
        key_i: int,
        key_j: int,
        relative_rotation: gtsam.Rot2,
        relative_translation: np.ndarray,
        rotation_precision: float,
        translation_precision: float,
    ) -> None:
        noise_model = gtsam.noiseModel.Diagonal.Precisions(
            2 * np.array([translation_precision] * 2 + [rotation_precision] * 4)
        )
        super().__init__(noise_model, [key_i, key_j], self.relative_pose2d_error_func)
        self._key_i = key_i
        self._key_j = key_j
        self._relative_rotation = get_rotation_matrix_from_theta(relative_rotation.theta())
        self._relative_translation = relative_translation
        self._rotation_precision = rotation_precision
        self._translation_precision = translation_precision

    def relative_pose2d_error_func(
        self,
        this: gtsam.CustomFactor,
        values: gtsam.Values,
        jacobians: Optional[List[np.ndarray]],
    ) -> float:
        """Error function for RelativePose2dFactor with Langevin noise on the
        rotation and Gaussian noise on the translation.

        Args:
            measurement (np.ndarray): The measurement, a 3x1 vector of [x, y, theta]
            this (gtsam.CustomFactor): The factor itself
            values (gtsam.Values): The current estimate of the variables
            jacobians (Optional[List[np.ndarray]]): The jacobians of the error function

        Returns:
            float: The unwhitened error
        """
        # Get the keys
        key1 = this.keys()[0]
        key2 = this.keys()[1]

        # Get the values
        Xi = values.atPose2(key1)
        Xj = values.atPose2(key2)

        # Get the measurement
        measured_translation = self._relative_translation
        measured_rotation = self._relative_rotation

        # Get the error
        residual = np.zeros(6)
        trans_error = (Xj.translation() - Xi.translation()) - (
            Xi.rotation().matrix() @ measured_translation
        )
        rot_error = Xj.rotation().matrix() - (
            Xi.rotation().matrix() @ measured_rotation
        )
        residual[0:2] = trans_error
        residual[2:6] = rot_error.reshape(4)

        # Get the jacobians

        # J = [ dt_err / dt,    dt_err / dR ;
        #       0,              dvec(R_err) / dR];
        if jacobians:
            # build the first jacobian
            H1 = np.zeros((6, 3))
            Ri = Xi.rotation().matrix()
            dRi_domega = tangent_matrix(1.0)

            # Set dt_err / dt_i
            H1[0:2, 0:2] = -Ri

            # Set dt_err / dR_i
            # H1->topRightCorner<2, 1>() = -Ri * dRi_domegai * tij;
            H1[0:2, 2] = -Ri @ dRi_domega @ measured_translation

            # dR_err / dt_i = 0

            # Set dR_err / dR_i
            # Eigen::Matrix2d dRerr_dRi = -Ri * dRi_domegai * Rij.matrix();
            dRerr_dRi = -Ri @ dRi_domega @ measured_rotation

            # H1->bottomRightCorner<4, 1>() =
            #     Eigen::Map<Eigen::Vector4d>(dRerr_dRi.data());
            H1[2:6, 2] = dRerr_dRi.reshape(4)

            jacobians[0] = H1

            # build the second jacobian
            H2 = np.zeros((6, 3))
            Rj = Xj.rotation().matrix()
            dRj_domega = tangent_matrix(1.0)

            # Set dt_err / dt_j
            # H2->topLeftCorner<2, 2>() = Rj;
            H2[0:2, 0:2] = Rj

            # dt_err / dR_j = 0
            # dR_err / dt_j = 0

            # dR_err / dR_j
            # Eigen::Matrix2d dRerr_dRj = Rj * tangent_matrix(1);
            dRerr_dRj = Rj @ dRj_domega

            # H2->bottomRightCorner<4, 1>() =
            #     Eigen::Map<Eigen::Vector4d>(dRerr_dRj.data());
            H2[2:6, 2] = dRerr_dRj.reshape(4)

            jacobians[1] = H2

        # Return the error
        return residual


def theta2matrix(theta: float) -> np.ndarray:
    """Converts a rotation angle to a rotation matrix.

    Args:
        theta (float): The rotation angle

    Returns:
        np.ndarray: The rotation matrix
    """
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


if __name__ == "__main__":
    measured_translation = np.array([1.0, 2.0])
    measured_rotation = theta2matrix(0.1)

    theta_1 = 0.0
    pose1 = gtsam.Pose2(0.0, 0.0, theta_1)

    for rot_noise in np.arange(0.0, 0.5, step=0.05):
        theta_2 = 0.1 + rot_noise
        pose2 = gtsam.Pose2(1.0, 2.0, theta_2)
        symbol1 = gtsam.symbol("x", 1)
        symbol2 = gtsam.symbol("x", 2)

        factor = RelativePose2dFactor(
            symbol1,
            symbol2,
            measured_rotation,
            measured_translation,
            translation_precision=1,
            rotation_precision=1,
        )

        values = gtsam.Values()
        values.insert(symbol1, pose1)
        values.insert(symbol2, pose2)

        error = factor.error(values)
        linearized_factor = factor.linearize(values)
        print(linearized_factor)

        # get the error and the jacobians
        jacobians = [np.zeros((6, 3)), np.zeros((6, 3))]
        error = factor.error(values)

        print(f"Error: {error}")

        # manually calculate the Langevin error
        # rot_error = rot2 - rot1 @ measured_rotation
        rot1 = pose1.rotation().matrix()
        rot2 = pose2.rotation().matrix()
        rot_error = rot2 - rot1 @ measured_rotation
        rot_error_norm = np.linalg.norm(rot_error, "fro") ** 2
        print(f"Rot error should be: {rot_error_norm}")
