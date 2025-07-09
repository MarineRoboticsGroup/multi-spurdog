import numpy as np
import re

import logging, coloredlogs

logger = logging.getLogger(__name__)
field_styles = {
    "filename": {"color": "green"},
    "filename": {"color": "green"},
    "levelname": {"bold": True, "color": "black"},
    "name": {"color": "blue"},
}
coloredlogs.install(
    level="INFO",
    fmt="[%(filename)s:%(lineno)d] %(name)s %(levelname)s - %(message)s",
    field_styles=field_styles,
)

from gtsam.gtsam import (
    NonlinearFactorGraph,
    Values,
    RangeFactor2D,
    RangeFactor3D,
    RangeFactorPose2,
    RangeFactorPose3,
    noiseModel,
    BetweenFactorPose2,
    BetweenFactorPose3,
    Pose2,
    Pose3,
    Rot2,
    Rot3,
    PriorFactorPose2,
    PriorFactorPose3,
    PriorFactorPoint2,
    PriorFactorPoint3,
    symbol,
)

try:
    from gtsam import SESyncFactor2d as RelativePose2dFactor

    logger.debug("Found C++ SESyncFactor2d")
except ImportError:
    logger.warning("Using python SESyncFactor2d - will be much slower")
    from ra_slam.custom_factors.SESyncFactor2d import RelativePose2dFactor

try:
    from gtsam import SESyncFactor3d as RelativePose3dFactor

    logger.debug("Found C++ SESyncFactor3d")
except ImportError:
    logger.warning("Using python SESyncFactor3d - will be much slower")
    from ra_slam.custom_factors.SESyncFactor3d import RelativePose3dFactor

try:
    from gtsam_unstable import PoseToPointFactor2D as PoseToPoint2dFactor
    from gtsam_unstable import PoseToPointFactor3D as PoseToPoint3dFactor

    logger.info("Found C++ PoseToPointFactor for 2D and 3D")
except ImportError:
    logger.warning("Using python PoseToPointFactor - will be much slower")
    from ra_slam.custom_factors.PoseToPointFactor import (
        PoseToPoint2dFactor,
        PoseToPoint3dFactor,
    )

from gtsam_unstable.gtsam_unstable import PoseToPointFactor2D, PoseToPointFactor3D
from ra_slam.custom_factors.SESyncFactor2d import RelativePose2dFactor
from ra_slam.custom_factors.SESyncFactor3d import RelativePose3dFactor
from ra_slam.custom_factors.PoseToPointFactor import (
    PoseToPoint2dFactor,
    PoseToPoint3dFactor,
)

from estimator import Estimator
from estimator_helpers import (
    Key,
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
    get_theta_from_rotation_matrix,
    get_translation_from_transformation_matrix,
    get_measurement_precisions_from_covariance_matrix
)

from typing import Union

VALID_BETWEEN_FACTOR_MODELS = ["SESync", "between"]


def get_symbol_from_name(name: str) -> symbol:
    """
    Returns the symbol from a variable name
    """
    assert isinstance(name, str)
    capital_letters_pattern = re.compile(r"^[A-Z][0-9]+$")
    assert capital_letters_pattern.match(name) is not None, "Invalid name"

    return symbol(name[0], int(name[1:]))


def get_pose2_from_matrix(pose_matrix: np.ndarray) -> Pose2:
    """
    Returns the pose2 from a transformation matrix
    """
    _check_transformation_matrix(pose_matrix, dim=2)
    theta = get_theta_from_transformation_matrix(pose_matrix)
    trans = get_translation_from_transformation_matrix(pose_matrix)
    return Pose2(trans[0], trans[1], theta)


def get_pose3_from_matrix(pose_matrix: np.ndarray) -> Pose3:
    """_summary_

    Args:
        pose_matrix (np.ndarray): _description_

    Returns:
        Pose3: _description_
    """
    _check_transformation_matrix(pose_matrix, dim=3)
    rot_matrix = pose_matrix[:3, :3]
    tx, ty, tz = pose_matrix[:3, 3]
    return Pose3(Rot3(rot_matrix), np.array([tx, ty, tz]))


def get_relative_pose_from_odom_measurement(odom_measurement: OdometryMeasurement) -> Union[Pose2, Pose3]:
    """Get the relative pose from the odometry measurement.

    Args:
        odom_measurement (POSE_MEASUREMENT_TYPES): the odometry measurement

    Returns:
        Pose2: the relative pose
    """
    if isinstance(odom_measurement, OdometryMeasurement2D):
        return Pose2(odom_measurement.x, odom_measurement.y, odom_measurement.psi)
    elif isinstance(odom_measurement, OdometryMeasurement3D):
        return Pose3(
            Rot3(odom_measurement.rotation_matrix), odom_measurement.translation_vector
        )
    else:
        err = f"Unknown odometry measurement type: {type(odom_measurement)}"
        logger.error(err)
        raise ValueError(err)

def _get_between_factor(
    odom_measurement: OdometryMeasurement, i_sym: int, j_sym: int
) -> Union[BetweenFactorPose2, BetweenFactorPose3]:
    odom_noise = noiseModel.Diagonal.Sigmas(np.diag(odom_measurement.covariance.covariance_matrix))
    rel_pose = get_relative_pose_from_odom_measurement(odom_measurement)
    if isinstance(odom_measurement, OdometryMeasurement2D):
        odom_factor = BetweenFactorPose2(i_sym, j_sym, rel_pose, odom_noise)
    elif isinstance(odom_measurement, OdometryMeasurement3D):
        odom_factor = BetweenFactorPose3(i_sym, j_sym, rel_pose, odom_noise)
    else:
        raise ValueError(f"Unknown measurement type: {type(odom_measurement)}")
    return odom_factor


def _get_between_se_sync_factor(
    odom_measurement: OdometryMeasurement, i_sym: int, j_sym: int
) -> Union[RelativePose2dFactor, RelativePose3dFactor]:
    if isinstance(odom_measurement, OdometryMeasurement2D):
        rot2_measure = Rot2(odom_measurement.relative_rotation)
        odom_factor = RelativePose2dFactor(
            i_sym,
            j_sym,
            rot2_measure,
            odom_measurement.translation_vector,
            odom_measurement.rotation_precision,
            odom_measurement.translation_precision,
        )
    elif isinstance(odom_measurement, OdometryMeasurement3D):
        rot3_measure = Rot3(odom_measurement.rotation_matrix)
        odom_factor = RelativePose3dFactor(
            i_sym,
            j_sym,
            rot3_measure,
            odom_measurement.translation_vector,
            odom_measurement.rotation_precision,
            odom_measurement.translation_precision,
        )
    else:
        raise ValueError(f"Unknown measurement type: {type(odom_measurement)}")
    return odom_factor


def get_pose_to_pose_factor(
    odom_measurement: OdometryMeasurement,
    i_symbol: int,
    j_symbol: int,
    factor_model: str = "between",
) -> Union[BetweenFactorPose2, BetweenFactorPose3]:
    """Get the odometry factor from the odometry measurement.

    Args:
        odom_measurement (POSE_MEASUREMENT_TYPES): the odometry measurement
        i_symbol (int): the symbol for the first pose
        j_symbol (int): the symbol for the second pose

    Returns:
        Union[BetweenFactorPose2, BetweenFactorPose3]: the relative pose factor
    """
    assert (
        factor_model in VALID_BETWEEN_FACTOR_MODELS
    ), f"Invalid factor model: {factor_model}. Valid models are: {VALID_BETWEEN_FACTOR_MODELS}"
    if factor_model == "SESync":
        odom_factor = _get_between_se_sync_factor(odom_measurement, i_symbol, j_symbol)
    elif factor_model == "between":
        odom_factor = _get_between_factor(odom_measurement, i_symbol, j_symbol)
    else:
        raise ValueError(f"Unknown factor model: {factor_model}")
    return odom_factor


class GtsamEstimator(Estimator):
    """
    A concrete implementation of the Estimator interface using GTSAM.
    This class should implement the GTSAM-specific logic for adding measurements and updating the state.
    """

    def __init__(self, mode: EstimatorMode, dimension: int, odom_factor_type: str):
        super().__init__(mode, dimension)

        # Initialize GTSAM-specific variables here
        self.factor_graph = NonlinearFactorGraph()
        self.values = Values()
        self.odom_factor_type = odom_factor_type

    def add_range(self, range_measurement: RangeMeasurement) -> None:
        pose_symbol = get_symbol_from_name(range_measurement.key1)
        landmark_symbol = get_symbol_from_name(range_measurement.key2)

        variance = range_measurement.variance
        range_noise = noiseModel.Isotropic.Sigma(1, variance)
        dist = range_measurement.distance

        # If the landmark is actually secretly a pose, then we use RangeFactorPose2
        if "L" not in range_measurement.key2:
            if self.dimension == 2:
                range_factor = RangeFactorPose2(
                    pose_symbol, landmark_symbol, dist, range_noise
                )
            elif self.dimension == 3:
                range_factor = RangeFactorPose3(
                    pose_symbol, landmark_symbol, dist, range_noise
                )
            else:
                raise ValueError(f"Unknown dimension: {self.dimension}")
        else:
            if self.dimension == 2:
                range_factor = RangeFactor2D(
                    pose_symbol, landmark_symbol, dist, range_noise
                )
            elif self.dimension == 3:
                range_factor = RangeFactor3D(
                    pose_symbol, landmark_symbol, dist, range_noise
                )
            else:
                raise ValueError(f"Unknown dimension: {self.dimension}")

        self.factor_graph.push_back(range_factor)

    def add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        # the indices of the related poses in the odometry measurement
        i_symbol = get_symbol_from_name(odom_measurement.key1)
        j_symbol = get_symbol_from_name(odom_measurement.key1)

        # add the factor to the factor graph
        odom_factor = get_pose_to_pose_factor(
            odom_measurement, i_symbol, j_symbol, self.odom_factor_type
        )
        self.factor_graph.push_back(odom_factor)

        raise NotImplementedError(
            "GTSAM odometry measurement addition not implemented."
        )

    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        raise NotImplementedError("GTSAM depth measurement addition not implemented.")

    def get_pose(self, key: Key) -> Union[Pose2D, Pose3D]:
        """
        Get the current pose from the GTSAM estimator.

        Args:
            key: The key for which to retrieve the pose.

        Returns:
            The pose corresponding to the given key.
        """
        raise NotImplementedError("GTSAM get_pose not implemented.")

    def get_point(self, key: Key) -> Union[Point2D, Point3D]:
        """
        Get the current point estimate from the GTSAM estimator.

        Args:
            key: The key for which to retrieve the point estimate.

        Returns:
            The point estimate corresponding to the given key.
        """
        raise NotImplementedError("GTSAM get_point not implemented.")

    def update(self):
        raise NotImplementedError("GTSAM update not implemented.")
