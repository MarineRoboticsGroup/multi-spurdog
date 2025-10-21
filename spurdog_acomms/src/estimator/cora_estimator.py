from typing import Union, List, Optional, Set

from estimator import Estimator
from .estimator_helpers import (
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

import rospy  # type: ignore


def get_cora_symbol_from_key(key: Key) -> cora.Symbol:
    return cora.Symbol(key.char, key.index)


class CoraEstimator(Estimator):
    """
    A concrete implementation of the Estimator interface using CORA.
    This class should implement the CORA-specific logic for adding measurements and updating the state.
    """

    def __init__(self, mode: EstimatorMode, dimension: int):
        assert (
            mode == EstimatorMode.CORA
        ), f"Tried constructing CORA estimator but mode is {mode}."
        rospy.logdebug("Constructing CORA estimator")
        super().__init__(mode, dimension)
        self._dimension = dimension
        self._problem = cora.Problem(dimension, dimension + 1)
        self._current_estimate_val = cora.Values()
        self._solver_results = None
        rospy.logdebug("CORA estimator constructed.")

    def _specific_initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        rospy.logdebug(f"Initializing CORA pose for key: {pose.key}")
        if self._dimension == 2:
            assert isinstance(pose, Pose2D), "2D pose must be of type Pose2D"
            rotation_matrix = get_rotation_matrix_from_theta(pose.orientation)
        elif self._dimension == 3:
            assert isinstance(pose, Pose3D), "3D pose must be of type Pose3D"
            rotation_matrix = get_rotation_matrix_from_quat(np.array(pose.orientation))
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")
        translation_vector = np.array(pose.position)
        assert (
            len(translation_vector) == self._dimension
        ), "Translation vector has incorrect dimension"

        self._current_estimate_val.set_pose(
            get_cora_symbol_from_key(pose.key), rotation_matrix, translation_vector
        )
        rospy.logdebug(f"CORA pose for key {pose.key} initialized.")

    def _specific_initialize_point(self, point: Union[Point2D, Point3D]) -> None:
        rospy.logdebug(f"Initializing CORA point for key: {point.key}")
        position_vector = np.array(point.position)
        assert (
            len(position_vector) == self._dimension
        ), f"Position vector has incorrect dimension {position_vector.shape} (should be {self._dimension})"

        self._current_estimate_val.set_landmark(
            get_cora_symbol_from_key(point.key), position_vector
        )
        rospy.logdebug(f"CORA point for key {point.key} initialized.")

    def _specific_add_range(self, range_measurement: RangeMeasurement) -> None:
        rospy.logdebug(f"Adding CORA range measurement: {range_measurement}")
        range_measure = cora.RangeMeasurement(
            get_cora_symbol_from_key(range_measurement.key1),
            get_cora_symbol_from_key(range_measurement.key2),
            range_measurement.distance,
            range_measurement.variance,
        )
        self._problem.addRangeMeasurement(range_measure)
        rospy.logdebug("CORA range measurement added.")

    def _specific_add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        rospy.logdebug(f"Adding CORA odometry measurement: {odom_measurement}")
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
            get_cora_symbol_from_key(odom_measurement.key1),
            get_cora_symbol_from_key(odom_measurement.key2),
            rotation_matrix,
            translation_vector,
            covar,
        )
        self._problem.addRelativePoseMeasurement(rel_pose_measure)
        rospy.logdebug("CORA odometry measurement added.")

    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        raise NotImplementedError("Cora depth measurement addition not implemented.")

    def get_pose_from_estimator(self, key: Key) -> Union[Pose2D, Pose3D]:
        rospy.logdebug(f"Getting CORA pose for key: {key}")
        self._check_current_estimate()
        assert isinstance(self._current_estimate, np.ndarray)
        (rot, trans) = cora.extractRelaxedPose(
            self._problem, self._current_estimate, get_cora_symbol_from_key(key)
        )
        if self.dimension == 2:
            theta = get_theta_from_rotation_matrix(rot)
            return Pose2D(key=key, position=tuple(trans), orientation=theta)
        elif self.dimension == 3:
            quat = get_quat_from_rotation_matrix(rot)
            return Pose3D(key=key, position=tuple(trans), orientation=tuple(quat))
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")

    def get_point_from_estimator(self, key: Key) -> Union[Point2D, Point3D]:
        rospy.logdebug(f"Getting CORA point for key: {key}")
        self._check_current_estimate()
        assert isinstance(self._current_estimate, np.ndarray)
        point = cora.extractRelaxedPoint(
            self._problem, self._current_estimate, get_cora_symbol_from_key(key)
        )
        assert isinstance(point, np.ndarray), "2D point must be a numpy array"
        if self.dimension == 2:
            assert len(point) == 2, "2D point must have length 2"
            return Point2D(key=key, position=tuple(point))
        elif self.dimension == 3:
            assert len(point) == 3, "3D point must have length 3"
            return Point3D(key=key, position=tuple(point))
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")

    def update(self):
        rospy.logdebug("Updating CORA current estimate")
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
        rospy.logdebug("CORA current estimate updated.")

    def parse_pyfg(self, filepath: str) -> None:
        rospy.logdebug(f"Parsing CORA problem from file: {filepath}")
        self._problem = cora.parsePyfgTextToProblem(filepath)

    def _check_current_estimate(self):
        rospy.logdebug("Checking CORA current estimate is valid")
        if self._current_estimate is None:
            raise ValueError(
                "Current estimate is None. Please set an initial estimate before querying poses or points."
            )
        if not isinstance(self._current_estimate, np.ndarray):
            raise TypeError(
                f"Current estimate must be a numpy array. Is type {type(self._current_estimate)}"
            )
