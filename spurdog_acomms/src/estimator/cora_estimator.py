from typing import Union, List, Optional, Set

from .estimator import Estimator
from .types.key import Key, KeyPair
from .types.measurements import (
    RangeMeasurement,
    OdometryMeasurement,
    OdometryMeasurement2D,
    OdometryMeasurement3D,
    DepthMeasurement,
)
from .types.enums import EstimatorMode
from .types.covariance import RelPoseCovar6
from .types.variables import Pose3D, Pose2D, Point2D, Point3D
from .utils.validation import _check_transformation_matrix
from .utils.transformations import (
    get_theta_from_transformation_matrix,
    get_quat_from_rotation_matrix,
    get_rotation_matrix_from_quat,
    get_rotation_matrix_from_theta,
    get_theta_from_rotation_matrix,
    get_translation_from_transformation_matrix,
)
from .utils.precision import get_measurement_precisions_from_covariance_matrix
from .types.covariance import get_diag_relpose_covar

import cora
import numpy as np

import rospy  # type: ignore


def get_cora_symbol_from_key(key: Key) -> cora.Symbol:
    assert isinstance(key, Key), f"Key must be of type Key, got {type(key)}"
    assert isinstance(key.char, str) and len(key.char) == 1, f"Key char must be a single character string, got {key.char}"
    assert isinstance(key.index, int), f"Key index must be an integer, got {type(key.index)}"
    sym = cora.Symbol(key.char, key.index)
    rospy.logdebug(f"Constructed symbol {sym} from key {key}")
    return sym


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
        self._problem = cora.Problem(dimension, dimension + 1)
        self._solver_results = None
        self.cora_estimate = cora.Values()
        rospy.logdebug("CORA estimator constructed.")

    @property
    def current_estimate_matrix(self) -> np.ndarray:
        """
        Returns the current estimate as a numpy array.
        """
        self._check_current_estimate()
        return cora.getVarMatrixFromValues(self._problem, self.cora_estimate)

    def _specific_initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        rospy.logdebug(f"Initializing CORA pose for key: {pose.key}")
        if self.dimension == 2:
            assert isinstance(pose, Pose2D), "2D pose must be of type Pose2D"
            rotation_matrix = get_rotation_matrix_from_theta(pose.orientation)
        elif self.dimension == 3:
            assert isinstance(pose, Pose3D), "3D pose must be of type Pose3D"
            rotation_matrix = get_rotation_matrix_from_quat(np.array(pose.orientation))
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")
        translation_vector = np.array(pose.position)
        assert (
            len(translation_vector) == self.dimension
        ), f"Translation vector has incorrect dimension {len(translation_vector)} (should be {self.dimension})"

        self._problem.addPoseVariable(get_cora_symbol_from_key(pose.key))
        self.cora_estimate.set_pose(
            get_cora_symbol_from_key(pose.key), rotation_matrix, translation_vector
        )
        rospy.logdebug(f"CORA pose for key {pose.key} initialized.")

    def _specific_initialize_point(self, point: Union[Point2D, Point3D]) -> None:
        rospy.logdebug(f"Initializing CORA point for key: {point.key}")
        position_vector = np.array(point.position)
        assert (
            len(position_vector) == self.dimension
        ), f"Position vector has incorrect dimension {position_vector.shape} (should be {self.dimension})"

        self._problem.addLandmarkVariable(get_cora_symbol_from_key(point.key))
        self.cora_estimate.set_landmark(
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
        self._problem.updateProblemData()
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
        sym1 = get_cora_symbol_from_key(odom_measurement.key1)
        sym2 = get_cora_symbol_from_key(odom_measurement.key2)
        translation_vector = np.array(odom_measurement.relative_translation)
        covar = odom_measurement.covariance.covariance_matrix

        rospy.logdebug(f"Adding odometry between symbols: {sym1}, {sym2}")
        rospy.logdebug(f"Constructed symbols: {sym1}, {sym2}")
        rospy.logdebug(f"Constructed translation vector: {translation_vector}")
        rospy.logdebug(f"Constructed rotation matrix:\n{rotation_matrix}")

        rel_pose_measure = cora.RelativePoseMeasurement(
            sym1,
            sym2,
            rotation_matrix,
            translation_vector,
            covar,
        )
        rospy.logdebug(f"Constructed relative pose measurement: {rel_pose_measure}")
        self._problem.addRelativePoseMeasurement(rel_pose_measure)
        rospy.logdebug("CORA odometry measurement added.")
        self._problem.updateProblemData()
        rospy.logdebug("CORA problem data updated after adding odometry.")

    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        raise NotImplementedError("Cora depth measurement addition not implemented.")

    def get_pose_from_estimator(self, key: Key) -> Union[Pose2D, Pose3D]:
        rospy.logdebug(f"\n\n\n\nGetting CORA pose for key: {key}")
        self._problem.updateProblemData()
        rospy.logdebug(f"Current problem {self._problem}")
        rospy.logdebug(f"Current estimate matrix {self.current_estimate_matrix}")
        sym = get_cora_symbol_from_key(key)
        rospy.logdebug(f"Getting pose for symbol: {sym} \n\n\n\n")
        self._check_current_estimate()
        (rot, trans) = cora.extractRelaxedPose(
            self._problem, self.current_estimate_matrix, get_cora_symbol_from_key(key)
        )
        rospy.logdebug(f"Extracted rotation: {rot}, translation: {trans}")
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
        self._problem.updateProblemData()
        self._check_current_estimate()
        point = cora.extractRelaxedPoint(
            self._problem, self.current_estimate_matrix, get_cora_symbol_from_key(key)
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

    def add_pose_prior(self, pose: Union[Pose2D, Pose3D]) -> None:
        assert (
            pose.marginal_covariance is not None
        ), "Pose prior must have marginal covariance"
        rospy.logdebug(f"Adding CORA pose prior for key: {pose.key}")

        self._problem.addPosePrior(
            cora.PosePrior(
                id=get_cora_symbol_from_key(pose.key),
                R=(
                    get_rotation_matrix_from_theta(pose.orientation)  # type: ignore
                    if self.dimension == 2
                    else get_rotation_matrix_from_quat(np.array(pose.orientation))
                ),
                t=np.array(pose.position),
                cov=pose.marginal_covariance,
            )
        )

    def update(self):
        rospy.logdebug("Updating CORA current estimate")
        self._check_current_estimate()
        (self._solver_results, iterate_history) = cora.solveCORA(
            problem=self._problem,
            x0=self.current_estimate_matrix,
            max_relaxation_rank=6,
            verbose=True,
            log_iterates=False,
            show_iterates=False,
        )
        rospy.logdebug("CORA current estimate updated.")
        assert isinstance(self._solver_results.x, np.ndarray), f"Estimate matrix must be a numpy array, got {type(self._solver_results.x)}"

        try:
            self.cora_estimate = cora.getValuesFromVarMatrix(
                self._problem, self._solver_results.x
            )
        except Exception as e:
            rospy.logerr(f"Error converting variable matrix to CORA values: {e}")
            raise e

        for key in self.current_estimate.pose_map.keys():
            rospy.logdebug(f"Updated pose for key {key}: {self.get_pose_from_estimator(key)}")
            try:
                self.current_estimate.update_variable(self.get_pose_from_estimator(key))
            except Exception as e:
                err_msg = f"Error updating pose for key {key}: {e}"
                rospy.logerr(err_msg)
                raise Exception(err_msg)
        for key in self.current_estimate.point_map.keys():
            rospy.logdebug(f"Updated point for key {key}: {self.get_point_from_estimator(key)}")
            try:
                self.current_estimate.update_variable(self.get_point_from_estimator(key))
            except Exception as e:
                err_msg = f"Error updating point for key {key}: {e}"
                rospy.logerr(err_msg)
                raise Exception(err_msg)

    def parse_pyfg(self, filepath: str) -> None:
        rospy.logdebug(f"Parsing CORA problem from file: {filepath}")
        self._problem = cora.parsePyfgTextToProblem(filepath)

    def _check_current_estimate(self):
        rospy.logdebug("Checking CORA current estimate is valid")
        if self.current_estimate is None:
            raise ValueError(
                "Current estimate is None. Please set an initial estimate before querying poses or points."
            )
        if not isinstance(self.cora_estimate, cora.Values):
            raise TypeError(
                f"Current estimate must be of type cora.Values. Is type {type(self.cora_estimate)}"
            )
