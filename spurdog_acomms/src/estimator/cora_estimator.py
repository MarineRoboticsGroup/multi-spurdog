from typing import Union, List, Optional, Set

from estimator import Estimator
from estimator_helpers import (
    Key,
    KeyPair,
    RangeMeasurement,
    OdometryMeasurement,
    OdometryMeasurement2D,
    OdometryMeasurement3D,
    DepthMeasurement,
    EstimatorMode,
    RelPoseCovar6,
    Pose3D,
    Pose2D,
    Point2D,
    Point3D,
    _check_transformation_matrix,
    get_theta_from_transformation_matrix,
    get_quat_from_rotation_matrix,
    get_rotation_matrix_from_quat,
    get_rotation_matrix_from_theta,
    get_theta_from_rotation_matrix,
    get_translation_from_transformation_matrix,
    get_measurement_precisions_from_covariance_matrix,
    get_diag_relpose_covar,
)

import cora
import numpy as np


def get_symbol_from_key(key: Key) -> cora.Symbol:
    char = key[0]
    idx = int(key[1:])
    return cora.Symbol(char, idx)

class CoraEstimator(Estimator):
    """
    A concrete implementation of the Estimator interface using CORA.
    This class should implement the CORA-specific logic for adding measurements and updating the state.
    """

    def __init__(self, mode: EstimatorMode, dimension: int):
        assert mode == EstimatorMode.CORA
        super().__init__(mode, dimension)
        self._problem = cora.Problem(dimension, dimension + 1)
        self._current_estimate = None
        self._solver_results = None

    def add_pose_variable(self, key: Key) -> None:
        self._problem.addPoseVariable(get_symbol_from_key(key))

    def add_landmark_variable(self, key: Key) -> None:
        self._problem.addLandmarkVariable(get_symbol_from_key(key))

    def add_range(self, range_measurement: RangeMeasurement) -> None:
        range_measure = cora.RangeMeasurement(
            get_symbol_from_key(range_measurement.key1),
            get_symbol_from_key(range_measurement.key2),
            range_measurement.distance,
            range_measurement.variance,
        )
        self._problem.addRangeMeasurement(range_measure)

    def add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        if self.dimension == 2:
            assert isinstance(
                odom_measurement, OdometryMeasurement2D
            ), "2D odometry measurement must be of type OdometryMeasurement2D"
            rotation_matrix = get_rotation_matrix_from_theta(
                odom_measurement.relative_rotation
            )
        elif self.dimension == 3:
            assert isinstance(
                odom_measurement, OdometryMeasurement3D
            ), "3D odometry measurement must be of type OdometryMeasurement3D"
            rotation_matrix = get_rotation_matrix_from_quat(
                np.array(odom_measurement.relative_rotation)
            )
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")
        translation_vector = np.array(odom_measurement.relative_translation)
        covar = odom_measurement.covariance.covariance_matrix
        rel_pose_measure = cora.RelativePoseMeasurement(
            get_symbol_from_key(odom_measurement.key1),
            get_symbol_from_key(odom_measurement.key2),
            rotation_matrix,
            translation_vector,
            covar,
        )
        self._problem.addRelativePoseMeasurement(rel_pose_measure)

    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        raise NotImplementedError("Cora depth measurement addition not implemented.")

    def get_pose(self, key: Key) -> Union[Pose2D, Pose3D]:
        self._check_current_estimate()
        assert isinstance(self._current_estimate, np.ndarray)
        (rot, trans) = cora.extractPose(
            self._problem, self._current_estimate, get_symbol_from_key(key)
        )
        if self.dimension == 2:
            theta = get_theta_from_rotation_matrix(rot)
            return Pose2D(key=key, position=tuple(trans), orientation=theta)
        elif self.dimension == 3:
            quat = get_quat_from_rotation_matrix(rot)
            return Pose3D(key=key, position=tuple(trans), orientation=tuple(quat))
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")

    def get_point(self, key: Key) -> Union[Point2D, Point3D]:
        self._check_current_estimate()
        assert isinstance(self._current_estimate, np.ndarray)
        point = cora.extractPoint(
            self._problem, self._current_estimate, get_symbol_from_key(key)
        )
        if self.dimension == 2:
            return Point2D(key=key, position=tuple(point))
        elif self.dimension == 3:
            return Point3D(key=key, position=tuple(point))
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")

    def update(self):
        self._check_current_estimate()
        assert isinstance(self._current_estimate, np.ndarray)
        (self._solver_results, self._current_estimate) = cora.solveCORA(
            problem=self._problem,
            x0=self._current_estimate,
            max_relaxation_rank=6,
            verbose=False,
            log_iterates=False,
            show_iterates=False,
        )

    def parse_pyfg(self, filepath: str) -> None:
        self._problem = cora.parsePyfgTextToProblem(filepath)


    def _check_current_estimate(self):
        if self._current_estimate is None:
            raise ValueError("Current estimate is None. Please set an initial estimate before querying poses or points.")
        if not isinstance(self._current_estimate, np.ndarray):
            raise TypeError(f"Current estimate must be a numpy array. Is type {type(self._current_estimate)}")