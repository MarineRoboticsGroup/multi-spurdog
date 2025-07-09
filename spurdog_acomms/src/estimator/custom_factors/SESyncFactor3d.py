import gtsam
import numpy as np
from typing import List, Optional

# Generators for the tangent space of SO(3)
G1 = np.array([[0, 0, 0], [0, 0, -1], [0, 1, 0]])

G2 = np.array([[0, 0, 1], [0, 0, 0], [-1, 0, 0]])

G3 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])


def tangent_matrix(omega: np.array):
    assert len(omega) == 3
    Omega = omega[0] * G1 + omega[1] * G2 + omega[2] * G3
    return Omega


class RelativePose3dFactor(gtsam.CustomFactor):
    def __init__(
        self,
        key_i: int,
        key_j: int,
        relative_rotation: np.ndarray,
        relative_translation: np.ndarray,
        rotation_precision: float,
        translation_precision: float,
        # noiseModel: gtsam.noiseModel.Base,
    ) -> None:
        noise_model = gtsam.noiseModel.Diagonal.Precisions(
            2 * np.array([translation_precision] * 3 + [rotation_precision] * 9)
        )
        # noise_model = noiseModel
        super().__init__(noise_model, [key_i, key_j], self.relative_pose3d_error_func)
        self._key_i = key_i
        self._key_j = key_j
        self._relative_rotation = relative_rotation.reshape((3, 3))
        self._relative_translation = relative_translation.reshape((3, 1))

    def relative_pose3d_error_func(
        self,
        this: gtsam.CustomFactor,
        values: gtsam.Values,
        jacobians: Optional[List[np.ndarray]],
    ) -> float:
        """Error function for RelativePose3dFactor with Langevin noise on the
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
        Xi = values.atPose3(key1)
        Xj = values.atPose3(key2)

        # Get the measurement
        measured_translation = self._relative_translation
        measured_rotation = self._relative_rotation

        # Get the error
        residual = np.zeros(12)
        # print(Xi.rotation().matrix())
        # print(measured_translation)
        trans_error = (Xj.translation() - Xi.translation()).reshape((3, 1)) - (
            Xi.rotation().matrix() @ measured_translation
        )
        # print(type(measured_rotation))
        rot_error = Xj.rotation().matrix() - (
            Xi.rotation().matrix() @ measured_rotation
        )
        residual[0:3] = trans_error[:, 0]
        residual[3:12] = rot_error.reshape(9)

        # Get the jacobians

        #  J = [ dt_err / dR, dt_err / dt ;
        #   dvec(R_err) / dR, 0];
        if jacobians:
            # build the first jacobian
            H1 = np.zeros((12, 6))
            Ri = Xi.rotation().matrix()
            # dRi_domega = tangent_matrix(1.0)

            # Set dt_err / dR_i
            H1[0:3, 0:3] = Ri @ tangent_matrix(measured_translation[:, 0])

            # Set dt_err / dt
            H1[0:3, 3:6] = -Ri
            # H1->topRightCorner<2, 1>() = -Ri * dRi_domegai * tij;
            # H1[0:3, 3] = (-Ri @ G1 @ measured_translation)[:,0]
            # H1[0:3, 4] = (-Ri @ G2 @ measured_translation)[:,0]
            # H1[0:3, 5] = (-Ri @ G3 @ measured_translation)[:,0]

            # dR_err / dt_i = 0

            # Set dR_err / dR_i
            # Eigen::Matrix2d dRerr_dRi = -Ri * dRi_domegai * Rij.matrix();
            # dRerr_dRi = -Ri @ dRi_domega @ measured_rotation
            dRerr_dRiG1 = -Ri @ G1 @ measured_rotation
            dRerr_dRiG2 = -Ri @ G2 @ measured_rotation
            dRerr_dRiG3 = -Ri @ G3 @ measured_rotation

            # H1->bottomRightCorner<4, 1>() =
            #     Eigen::Map<Eigen::Vector4d>(dRerr_dRi.data());
            H1[3:12, 0] = dRerr_dRiG1.reshape(9)
            H1[3:12, 1] = dRerr_dRiG2.reshape(9)
            H1[3:12, 2] = dRerr_dRiG3.reshape(9)

            jacobians[0] = H1

            # build the second jacobian
            H2 = np.zeros((12, 6))
            Rj = Xj.rotation().matrix()
            # dRj_domega = tangent_matrix(1.0)

            # Set dt_err / dt_j
            # H2->topLeftCorner<2, 2>() = Rj;
            H2[0:3, 3:6] = Rj

            # dt_err / dR_j = 0
            # dR_err / dt_j = 0

            # dR_err / dR_j
            # Eigen::Matrix2d dRerr_dRj = Rj * tangent_matrix(1);
            # dRerr_dRj = Rj @ dRj_domega
            dRerr_dRjG1 = Rj @ G1  # @ measured_rotation
            dRerr_dRjG2 = Rj @ G2  # @ measured_rotation
            dRerr_dRjG3 = Rj @ G3  # @ measured_rotation

            # H2->bottomRightCorner<4, 1>() =
            #     Eigen::Map<Eigen::Vector4d>(dRerr_dRj.data());
            H2[3:12, 0] = dRerr_dRjG1.reshape(9)
            H2[3:12, 1] = dRerr_dRjG2.reshape(9)
            H2[3:12, 2] = dRerr_dRjG3.reshape(9)

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


def uniform_random_vec3D():
    # See J. Arvo "Fast Random Rotation Matrices" (doi=10.1.1.53.1357)
    # for uniformly random unit vector computation
    x1 = np.random.uniform()
    x2 = np.random.uniform()
    vhat = np.array(
        [
            np.cos(2.0 * np.pi * x1) * np.sqrt(x2),
            np.sin(2.0 * np.pi * x1) * np.sqrt(x2),
            np.sqrt(1.0 - x2),
        ]
    )
    return np.mat(vhat).T


def sample_uniform_rotation(d):
    """
    A uniform sampler for SO(d), d=2,3

    See J. Arvo "Fast Random Rotation Matrices" (doi=10.1.1.53.1357) for uniformly
    random unit vector computation

    """
    x1 = np.random.uniform()
    theta = 2.0 * np.pi * x1
    rot2D = theta2matrix(theta)
    if d == 2:
        return rot2D
    else:
        # d == 3
        R = np.eye(3)
        R[:2, :2] = rot2D
        v = uniform_random_vec3D()
        vdv = v.dot(v.T)
        H = np.eye(3) - 2.0 * v.dot(v.T)
        return -H.dot(R)


if __name__ == "__main__":
    measured_translation = np.array([1.0, 2.0, 3.0])
    measured_rotation = sample_uniform_rotation(3)

    theta_1 = 0.0
    pose1 = gtsam.Pose3(r=gtsam.Rot3(np.eye(3)), t=np.zeros(3))

    translation_precision = 1
    rotation_precision = 1

    noise_model = gtsam.noiseModel.Diagonal.Precisions(
        2 * np.array([translation_precision] * 3 + [rotation_precision] * 9)
    )

    robustModel = gtsam.noiseModel.Robust.Create(
        gtsam.noiseModel.mEstimator.Cauchy.Create(1), noise_model
    )

    for i in range(10):
        tran_2 = np.array([1.0, 2.0, 3.0])  # np.random.uniform(-1, 1, 3)
        rot_2 = sample_uniform_rotation(3)
        pose2 = gtsam.Pose3(r=gtsam.Rot3(rot_2), t=tran_2)
        symbol1 = gtsam.symbol("x", 1)
        symbol2 = gtsam.symbol("x", 2)

        rot_precision = 1000
        tran_precision = 100
        factor = RelativePose3dFactor(
            symbol1,
            symbol2,
            measured_rotation,
            measured_translation,
            rot_precision,
            tran_precision,
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
        print(f"Rot error should be (less than with cauchy): {rot_error_norm}")
