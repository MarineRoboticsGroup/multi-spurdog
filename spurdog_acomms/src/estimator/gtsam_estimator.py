import numpy as np
import re
import rospy  # type: ignore

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
    PoseTranslationPrior2D,
    PoseTranslationPrior3D,
    # ISAM2
    ISAM2Params,
    ISAM2DoglegParams,
    ISAM2,
    # LM
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    Symbol,
)

rospy.logwarn("Using python SESyncFactor2d - will be much slower")
from .custom_factors.SESyncFactor2d import RelativePose2dFactor

rospy.logwarn("Using python SESyncFactor3d - will be much slower")
from .custom_factors.SESyncFactor3d import RelativePose3dFactor

rospy.logwarn("Using python PoseToPointFactor - will be much slower")
from .custom_factors.PoseToPointFactor import (
    PoseToPoint2dFactor,
    PoseToPoint3dFactor,
)

from .estimator import Estimator
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
    get_theta_from_rotation_matrix,
    get_translation_from_transformation_matrix,
    get_measurement_precisions_from_covariance_matrix,
    get_diag_relpose_covar,
)

from typing import Union, List

VALID_BETWEEN_FACTOR_MODELS = ["SESync", "between"]


def get_gtsam_symbol_from_key(key: Key) -> int:
    """
    Returns the symbol from a variable key
    """
    assert isinstance(key, Key), "Key must be of type Key"
    return symbol(key.char, key.index)


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
    return Pose3(Rot3(rot_matrix), np.array([tx, ty, tz]))  # type: ignore


def get_relative_pose_from_odom_measurement(
    odom_measurement: OdometryMeasurement,
) -> Union[Pose2, Pose3]:
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
        rospy.logerr(err)
        raise ValueError(err)


def _get_between_factor(
    odom_measurement: OdometryMeasurement, i_sym: int, j_sym: int
) -> Union[BetweenFactorPose2, BetweenFactorPose3]:
    odom_noise = noiseModel.Diagonal.Sigmas(
        np.diag(odom_measurement.covariance.covariance_matrix)  # type: ignore
    )
    rel_pose = get_relative_pose_from_odom_measurement(odom_measurement)
    if isinstance(odom_measurement, OdometryMeasurement2D):
        odom_factor = BetweenFactorPose2(i_sym, j_sym, rel_pose, odom_noise)  # type: ignore
    elif isinstance(odom_measurement, OdometryMeasurement3D):
        odom_factor = BetweenFactorPose3(i_sym, j_sym, rel_pose, odom_noise)  # type: ignore
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
) -> Union[
    BetweenFactorPose2, BetweenFactorPose3, RelativePose2dFactor, RelativePose3dFactor
]:
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


def solve_with_isam2(
    graph: NonlinearFactorGraph, initial_vals: Values, return_all_iterates: bool = False
) -> Union[Values, List[Values]]:
    if return_all_iterates:
        raise NotImplementedError("ISAM2 does not support returning all iterates")

    parameters = ISAM2Params()
    parameters.setOptimizationParams(ISAM2DoglegParams())
    rospy.logdebug(f"ISAM Params: {parameters}")
    isam_solver = ISAM2(parameters)
    isam_solver.update(graph, initial_vals)
    result = isam_solver.calculateEstimate()
    return result


def solve_with_levenberg_marquardt(
    graph: NonlinearFactorGraph, initial_vals: Values, return_all_iterates: bool = False
) -> Union[Values, List[Values]]:

    def _check_all_variables_have_initialization():
        # check that the variables in the graph are all in the initial values
        # otherwise, the optimizer will throw an error
        init_vals_vars = initial_vals.keys()
        graph_vars = graph.keyVector()

        graph_var_set = set(graph_vars)
        init_vals_var_set = set(init_vals_vars)

        in_graph_not_init_vals = graph_var_set - init_vals_var_set
        in_init_vals_not_graph = init_vals_var_set - graph_var_set

        if len(in_graph_not_init_vals) > 0:
            graph_vars_as_symbols = [Symbol(key) for key in in_graph_not_init_vals]
            raise ValueError(
                f"Variables in graph but not in initial values: {graph_vars_as_symbols}"
            )

        if len(in_init_vals_not_graph) > 0:
            init_vals_vars_as_symbols = [Symbol(key) for key in in_init_vals_not_graph]
            raise ValueError(
                f"Variables in initial values but not in graph: {init_vals_vars_as_symbols}"
            )

    _check_all_variables_have_initialization()

    params = LevenbergMarquardtParams()
    params.setMaxIterations(50)
    params.setRelativeErrorTol(0.0)
    params.setAbsoluteErrorTol(0.0)
    params.setVerbosity("TERMINATION")
    params.setVerbosityLM("LAMBDA")
    try:
        rospy.logerr("Running Levenberg-Marquardt optimization...")
        optimizer = LevenbergMarquardtOptimizer(graph, initial_vals, params)
        result = optimizer.optimize()
    except RuntimeError as e:
        rospy.logerr(f"Failed to run LevenbergMarquardtOptimizer: {e}")
        rospy.logerr(f"Graph keys: {graph.keyVector()}")
        rospy.logerr(f"Initial values: {initial_vals.keys()}")
        raise e

    rospy.logwarn(
        f"Levenberg-Marquardt optimization complete with final error: {optimizer.error()}"
    )

    initial_error = graph.error(initial_vals)
    final_error = graph.error(result)
    rospy.logwarn(
        f"Initial error: {initial_error:<.6f} | Final error: {final_error:<.6f}"
    )
    return result

    # results = [optimizer.values()]
    # currentError = np.inf
    # newError = optimizer.error()
    # rel_err_tol = params.getRelativeErrorTol()
    # abs_err_tol = params.getAbsoluteErrorTol()
    # err_tol = params.getErrorTol()
    # max_iter = params.getMaxIterations()

    # converged = False
    # curr_iter = 0

    # while not converged and curr_iter < max_iter:
    #     optimizer.iterate()
    #     results.append(optimizer.values())

    #     currentError = newError
    #     newError = optimizer.error()

    #     within_rel_err_tol = (
    #         abs(newError - currentError) < rel_err_tol * currentError
    #     )
    #     within_abs_err_tol = abs(newError - currentError) < abs_err_tol
    #     within_err_tol = newError < err_tol

    #     converged = within_rel_err_tol or within_abs_err_tol or within_err_tol
    #     curr_iter += 1

    # return results


class GtsamEstimator(Estimator):
    """
    A concrete implementation of the Estimator interface using GTSAM.
    This class should implement the GTSAM-specific logic for adding measurements and updating the state.
    """

    def __init__(self, mode: EstimatorMode, dimension: int, odom_factor_type: str):
        super().__init__(mode, dimension)

        if mode == EstimatorMode.GTSAM_DOG:
            raise NotImplementedError(
                "GTSAM Dogleg mode is not implemented. Use GTSAM Levenberg-Marquardt for now."
            )

        # Initialize GTSAM-specific variables here
        self.factor_graph = NonlinearFactorGraph()
        self.odom_factor_type = odom_factor_type
        self.gtsam_estimate = Values()

    def _specific_add_range(self, range_measurement: RangeMeasurement) -> None:
        pose_symbol = get_gtsam_symbol_from_key(range_measurement.key1)
        landmark_symbol = get_gtsam_symbol_from_key(range_measurement.key2)

        variance = range_measurement.variance
        range_noise = noiseModel.Isotropic.Sigma(1, variance)
        dist = range_measurement.distance

        # If the landmark is actually secretly a pose, then we use RangeFactorPose2
        is_pose_landmark_measure = range_measurement.key2.is_landmark
        if not is_pose_landmark_measure:
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

    def _specific_add_odometry(self, odom_measurement: OdometryMeasurement) -> None:
        # the indices of the related poses in the odometry measurement
        i_symbol = get_gtsam_symbol_from_key(odom_measurement.key1)
        j_symbol = get_gtsam_symbol_from_key(odom_measurement.key2)

        # add the factor to the factor graph
        odom_factor = get_pose_to_pose_factor(
            odom_measurement, i_symbol, j_symbol, self.odom_factor_type
        )
        self.factor_graph.push_back(odom_factor)

    def add_pose_prior(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Add a pose prior to the estimator.

        Args:
            pose: The pose to be added as a prior.
        """
        sym = get_gtsam_symbol_from_key(pose.key)
        rospy.loginfo(f"Adding pose prior for key: {pose.key}")
        assert (
            pose.marginal_covariance is not None
        ), "Pose marginal covariance must be set before adding a prior."
        if isinstance(pose, Pose2D):
            pose_init = get_pose2_from_matrix(pose.transformation_matrix)
            noise_model = noiseModel.Diagonal.Sigmas(
                np.sqrt(pose.marginal_covariance.diagonal())
            )
            prior_factor = PriorFactorPose2(sym, pose_init, noise_model)
        elif isinstance(pose, Pose3D):
            pose_init = get_pose3_from_matrix(pose.transformation_matrix)
            noise_model = noiseModel.Diagonal.Sigmas(
                np.sqrt(pose.marginal_covariance.diagonal())
            )
            prior_factor = PriorFactorPose3(sym, pose_init, noise_model)
        else:
            raise ValueError(f"Unknown pose type: {type(pose)}")

        self.factor_graph.push_back(prior_factor)

    def _specific_initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Estimator-specific code to initialize the pose of a key in the
        estimator.

        Args:
            key: The key for which to initialize the pose.
            pose: The initial pose to set for the key.
        """
        sym = get_gtsam_symbol_from_key(pose.key)
        if isinstance(pose, Pose2D):
            pose_init = get_pose2_from_matrix(pose.transformation_matrix)
        elif isinstance(pose, Pose3D):
            pose_init = get_pose3_from_matrix(pose.transformation_matrix)
        else:
            raise ValueError(f"Unknown pose type: {type(pose)}")

        self.gtsam_estimate.insert(sym, pose_init)

        # if the pose is also index 0, we add a prior to fix the gauge freedom
        if pose.key.index == 0:
            rospy.logwarn(
                f"Adding prior to pose {pose.key} to fix gauge freedom (index 0)"
            )
            try:
                if pose.marginal_covariance is None:
                    pose.marginal_covariance = np.eye(3 if isinstance(pose, Pose2D) else 6) * 1e-6
                self.add_pose_prior(pose)
            except Exception as e:
                rospy.logerr(f"Failed to add prior to pose {pose.key}: {e}")

    def _specific_initialize_point(self, point: Union[Point2D, Point3D]) -> None:
        """
        Initialize the point of a key in the estimator.

        Args:
            key: The key for which to initialize the point.
            point: The initial point to set for the key.
        """
        assert isinstance(
            point, (Point2D, Point3D)
        ), "Point must be of type Point2D or Point3D"
        sym = get_gtsam_symbol_from_key(point.key)
        self.gtsam_estimate.insert(sym, np.array(point.position))

        #### TEMPORARY CODE -- ADD PRIORS ####
        rospy.logwarn(
            f"Adding prior to point {point.key} during initialization (temporary code)"
        )
        try:
            if point.marginal_covariance is None:
                point.marginal_covariance = np.eye(2 if isinstance(point, Point2D) else 3)
            if isinstance(point, Point2D):
                point_init = np.array(point.position)
                noise_model = noiseModel.Diagonal.Sigmas(
                    np.sqrt(point.marginal_covariance.diagonal())
                )
                prior_factor = PriorFactorPoint2(sym, point_init, noise_model)
            elif isinstance(point, Point3D):
                point_init = np.array(point.position)
                noise_model = noiseModel.Diagonal.Sigmas(
                    np.sqrt(point.marginal_covariance.diagonal())
                )
                prior_factor = PriorFactorPoint3(sym, point_init, noise_model)
            else:
                raise ValueError(f"Unknown point type: {type(point)}")

            self.factor_graph.push_back(prior_factor)
        except Exception as e:
            rospy.logerr(f"Failed to add prior to point {point.key}: {e}")

    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        """We will add a poor man's depth measurement by placing a prior
        on the translation of the pose. We will add very low precision
        to the x and y translation, and high precision to the z translation.

        Args:
            depth_measurement (DepthMeasurement): the depth measurement to be added.
        """
        depth = depth_measurement.depth
        depth_precision = 1 / depth_measurement.variance
        other_translation_precision = (
            depth_precision / 100000.0
        )  # very low precision for x and y
        pose_symbol = get_gtsam_symbol_from_key(depth_measurement.key)

        # how many dimensions do we need to fill for a psuedo depth measurement?
        fill_dim = self.dimension - 1

        # make the noise model for the translation prior
        noise_model = noiseModel.Diagonal.Sigmas(
            np.array(
                [other_translation_precision] * fill_dim
                + [depth_precision]  # high precision for z translation
            )
        )
        fake_translation = np.array(
            [0.0] * fill_dim + [depth]  # fake translation in the z direction
        )
        prior_factor = PoseTranslationPrior3D(
            pose_symbol,
            fake_translation,
            noise_model,
        )
        self.factor_graph.push_back(prior_factor)

    def get_pose_from_estimator(self, key: Key) -> Union[Pose2D, Pose3D]:
        """
        Get the current pose from the GTSAM estimator.

        Args:
            key: The key for which to retrieve the pose.

        Returns:
            The pose corresponding to the given key.
        """
        pose_symbol = get_gtsam_symbol_from_key(key)
        if self.dimension == 2:
            pose_result2d = self.gtsam_estimate.atPose2(pose_symbol)  # type:ignore
            pose2d = Pose2D(
                key=key,
                position=(pose_result2d.x(), pose_result2d.y()),
                orientation=get_theta_from_rotation_matrix(
                    pose_result2d.rotation().matrix()
                ),
            )
            return pose2d
        elif self.dimension == 3:
            pose_result3d = self.gtsam_estimate.atPose3(pose_symbol)  # type:ignore
            pose3d = Pose3D(
                key=key,
                position=tuple(pose_result3d.translation()),
                orientation=tuple(
                    get_quat_from_rotation_matrix(pose_result3d.rotation().matrix())
                ),
            )
            return pose3d
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")

    def get_point_from_estimator(self, key: Key) -> Union[Point2D, Point3D]:
        """
        Get the current point estimate from the GTSAM estimator.

        Args:
            key: The key for which to retrieve the point estimate.

        Returns:
            The point estimate corresponding to the given key.
        """
        point_symbol = get_gtsam_symbol_from_key(key)
        point_result = self.gtsam_estimate.atPoint2(point_symbol) if self.dimension == 2 else self.gtsam_estimate.atPoint3(point_symbol)  # type: ignore

        if self.dimension == 2:
            point2d = Point2D(
                key=key, position=(float(point_result[0]), float(point_result[1]))
            )
            return point2d
        elif self.dimension == 3:
            point3d = Point3D(
                key=key,
                position=(
                    float(point_result[0]),
                    float(point_result[1]),
                    float(point_result[2]),
                ),
            )
            return point3d
        else:
            raise ValueError(f"Unknown dimension: {self.dimension}")

    def update(self):
        """
        Update the estimator by running optimization and syncing results to current_estimate.
        """
        rospy.loginfo("Running GTSAM optimization...")
        # Run GTSAM optimization
        result = solve_with_levenberg_marquardt(
            self.factor_graph, self.gtsam_estimate, return_all_iterates=False
        )

        # Ensure we have a single Values object, not a list
        assert isinstance(result, Values), "Expected Values object from optimization"
        self.gtsam_estimate = result

        # Update current_estimate with optimized values
        # Get all keys from the GTSAM result
        result_gtsam_keys = self.gtsam_estimate.keys()
        gtsam_symbols = [Symbol(k) for k in result_gtsam_keys]
        result_keys = [
            Key(f"{chr(Symbol(k).chr())}{Symbol(k).index()}") for k in result_gtsam_keys
        ]
        rospy.loginfo(f"Updating current estimates for keys: {result_keys}")

        #### TEST CODE ####
        import copy
        current_estimate_copy = copy.deepcopy(self.current_estimate)
        #### END TEST CODE ####

        for key, symbol in zip(result_keys, gtsam_symbols):

            # Convert GTSAM symbol back to our Key type
            key_str = f"{chr(symbol.chr())}{symbol.index()}"
            key = Key(key=key_str)
            rospy.logdebug(f"Updating estimate for key: {key}")

            # Check if it's a pose or point and update accordingly
            if key.is_landmark:
                # It's a point/landmark
                point = self.get_point_from_estimator(key)
                self.current_estimate.update_variable(point)
                rospy.logdebug(f"Updated point for key {key}: {point}")
            else:
                # It's a pose
                pose = self.get_pose_from_estimator(key)
                self.current_estimate.update_variable(pose)
                rospy.logdebug(f"Updated pose for key {key}: {pose}")

        #### TEST CODE ####
        # self.print_difference_between_estimates(current_estimate_copy)
        #### END TEST CODE ####


if __name__ == "__main__":
    pass
