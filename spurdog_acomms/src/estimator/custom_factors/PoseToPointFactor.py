import gtsam
import numpy as np
from typing import List, Optional

# Generators for the tangent space of SO(3)
G1 = np.array([[0, 0, 0], [0, 0, -1], [0, 1, 0]])

G2 = np.array([[0, 0, 1], [0, 0, 0], [-1, 0, 0]])

G3 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])


def tangent_matrix_3d(omega: np.ndarray) -> np.ndarray:
    assert len(omega) == 3
    Omega = omega[0] * G1 + omega[1] * G2 + omega[2] * G3
    return Omega


def tangent_matrix_2d(omega: float):
    Omega = np.zeros((2, 2))
    Omega[0, 1] = -omega
    Omega[1, 0] = omega
    return Omega


class PoseToPoint2dFactor(gtsam.CustomFactor):
    def __init__(
        self,
        key_i: int,
        key_j: int,
        relative_translation: np.ndarray,
        noise_model: gtsam.noiseModel.Base,
    ) -> None:
        super().__init__(noise_model, [key_i, key_j], self.relative_error_func)
        self._key_i = key_i
        self._key_j = key_j
        assert len(relative_translation) == 2
        assert translation_precision > 0
        self._relative_translation = relative_translation
        self._translation_precision = translation_precision

    def relative_error_func(
        self,
        this: gtsam.CustomFactor,
        values: gtsam.Values,
        jacobians: Optional[List[np.ndarray]],
    ) -> np.ndarray:
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
        Xj = values.atPoint2(key2)

        # Get the measurement
        measured_translation = self._relative_translation

        # Get the error
        residual = np.zeros(2)
        trans_error = (Xj - Xi.translation()) - (
            Xi.rotation().matrix() @ measured_translation
        )
        residual[0:2] = trans_error

        # Get the jacobians

        # J = [ dt_err / dt,    dt_err / dR ;
        #       0,              dvec(R_err) / dR];
        if jacobians:
            # build the first jacobian
            H1 = np.zeros((2, 3))
            Ri = Xi.rotation().matrix()
            dRi_domega = tangent_matrix_2d(1.0)

            # Set dt_err / dt_i
            H1[0:2, 0:2] = -Ri

            # Set dt_err / dR_i
            # H1->topRightCorner<2, 1>() = -Ri * dRi_domegai * tij;
            H1[0:2, 2] = -Ri @ dRi_domega @ measured_translation

            jacobians[0] = H1

            # build the second jacobian
            H2 = np.zeros((2, 2))

            # Set dt_err / dt_j
            # H2->topLeftCorner<2, 2>() = Rj;
            H2[0:2, 0:2] = np.eye(2)

            jacobians[1] = H2

        # Return the error
        return residual


class PoseToPoint3dFactor(gtsam.CustomFactor):
    def __init__(
        self,
        key_i: int,
        key_j: int,
        relative_translation: np.ndarray,
        noise_model: gtsam.noiseModel.Base,
    ) -> None:
        # noise_model = noiseModel
        super().__init__(noise_model, [key_i, key_j], self.relative_error_func)
        self._key_i = key_i
        self._key_j = key_j
        self._relative_translation = relative_translation.reshape((3, 1))

    def relative_error_func(
        self,
        this: gtsam.CustomFactor,
        values: gtsam.Values,
        jacobians: Optional[List[np.ndarray]],
    ) -> np.ndarray:
        """Error function for PoseToPoint3dFactor with
         Gaussian noise on the translation.
        Args:
            measurement (np.ndarray): The measurement, a 3x1 vector of [x, y, z]
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
        Xi = values.atPose3(key1)
        Xj = values.atPoint3(key2)

        # Get the measurement
        measured_translation = self._relative_translation

        # Get the error
        residual = np.zeros(3)
        trans_error = (Xj.translation() - Xi.translation()).reshape((3, 1)) - (
            Xi.rotation().matrix() @ measured_translation
        )
        residual[0:3] = trans_error[:, 0]

        # Get the jacobians

        #  J = [ dt_err / dR, dt_err / dt ;
        #   dvec(R_err) / dR, 0];
        if jacobians:
            # build the first jacobian
            H1 = np.zeros((3, 6))
            Ri = Xi.rotation().matrix()
            # dRi_domega = tangent_matrix_3d(1.0)

            # Set dt_err / dR_i
            H1[0:3, 0:3] = Ri @ tangent_matrix_3d(measured_translation[:, 0])

            # Set dt_err / dt
            H1[0:3, 3:6] = -Ri

            jacobians[0] = H1

            # build the second jacobian
            H2 = np.zeros((3, 3))

            # Set dt_err / dt_j
            H2[0:3, 0:3] = np.eye(3)

            jacobians[1] = H2

        # Return the error
        return residual


if __name__ == "__main__":

    def theta2matrix(theta: float) -> np.ndarray:
        """Converts a rotation angle to a rotation matrix.

        Args:
            theta (float): The rotation angle

        Returns:
            np.ndarray: The rotation matrix
        """
        return np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )

    theta_1 = 0.0
    pose1 = gtsam.Pose2(0.0, 0.0, theta_1)
    landmark2 = np.array([1.0, 2.0])

    noise = np.random.normal(0, 0.1, 2)
    # noise = np.zeros(2)
    measured_translation_in_world = landmark2 + noise
    measured_translation_in_pose1 = (
        theta2matrix(theta_1) @ measured_translation_in_world
    )

    symbol1 = gtsam.symbol("x", 1)
    symbol2 = gtsam.symbol("l", 2)

    values = gtsam.Values()
    values.insert(symbol1, pose1)
    values.insert(symbol2, landmark2)

    factor = PoseToPoint2dFactor(
        symbol1, symbol2, measured_translation_in_pose1, translation_precision=1
    )
    error = factor.error(values)
    linearized_factor = factor.linearize(values)
    print(f"Linearized factor:\n{linearized_factor}")

    # get the error and the jacobians
    jacobians = [np.zeros((6, 3)), np.zeros((6, 3))]
    error = factor.error(values)

    print(f"Error: {error}")
    print(f"Jacobians:\n{jacobians}")

    # manually calculate the Langevin error
    # rot_error = rot2 - rot1 @ measured_rotation
    rot1 = pose1.rotation().matrix()
    trans_error = landmark2 - pose1.translation() - rot1 @ measured_translation_in_pose1
    trans_error_norm = np.linalg.norm(trans_error)
    print(f"Trans error should be: {trans_error_norm**2}")
