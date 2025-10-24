"""
GTSAM-based estimator implementation.

This module provides the GtsamEstimator class, which implements the Estimator
interface using GTSAM's factor graph optimization.
"""
import numpy as np
import copy
import rospy  # type: ignore

from gtsam.gtsam import (
    NonlinearFactorGraph,
    Values,
    RangeFactor2D,
    RangeFactor3D,
    RangeFactorPose2,
    RangeFactorPose3,
    noiseModel,
    PriorFactorPose2,
    PriorFactorPose3,
    PriorFactorPoint2,
    PriorFactorPoint3,
    PoseTranslationPrior3D,
    Symbol,
)

from typing import Union

from ...estimator import Estimator
from ...types.key import Key
from ...types.measurements import (
    RangeMeasurement,
    OdometryMeasurement,
    DepthMeasurement,
)
from ...types.enums import EstimatorMode
from ...types.variables import Pose3D, Pose2D, Point2D, Point3D
from ...utils.transformations import (
    get_quat_from_rotation_matrix,
    get_theta_from_rotation_matrix,
)

from .conversions import (
    get_gtsam_symbol_from_key,
    get_pose2_from_matrix,
    get_pose3_from_matrix,
)
from .factors import get_pose_to_pose_factor
from .solvers import solve_with_levenberg_marquardt


class GtsamEstimator(Estimator):
    """
    A concrete implementation of the Estimator interface using GTSAM.
    
    This class implements GTSAM-specific logic for adding measurements,
    managing the factor graph, and running optimization.
    """

    def __init__(
        self, 
        mode: EstimatorMode, 
        dimension: int, 
        odom_factor_type: str,
        enable_diagnostics: bool = False,
        try_gauss_newton_fallback: bool = False
    ):
        """
        Initialize the GTSAM estimator.
        
        Args:
            mode: The estimator mode (should be GTSAM_LM or GTSAM_DOG).
            dimension: The dimension of the state space (2 or 3).
            odom_factor_type: Type of odometry factor ("between" or "SESync").
            enable_diagnostics: If True, run optimization diagnostics (default: False).
            try_gauss_newton_fallback: If True, try Gauss-Newton if LM fails (default: False).
            
        Raises:
            NotImplementedError: If GTSAM_DOG mode is used (not yet implemented).
        """
        super().__init__(mode, dimension)

        if mode == EstimatorMode.GTSAM_DOG:
            raise NotImplementedError(
                "GTSAM Dogleg mode is not implemented. Use GTSAM Levenberg-Marquardt for now."
            )

        # Initialize GTSAM-specific variables
        self.factor_graph = NonlinearFactorGraph()
        self.odom_factor_type = odom_factor_type
        self.gtsam_estimate = Values()
        
        # Optimization settings
        self.enable_diagnostics = enable_diagnostics
        self.try_gauss_newton_fallback = try_gauss_newton_fallback

    def _specific_add_range(self, range_measurement: RangeMeasurement) -> None:
        """
        Add a range measurement to the factor graph.
        
        Args:
            range_measurement: The range measurement to add.
        """
        pose_symbol = get_gtsam_symbol_from_key(range_measurement.key1)
        landmark_symbol = get_gtsam_symbol_from_key(range_measurement.key2)

        variance = range_measurement.variance
        range_noise = noiseModel.Isotropic.Sigma(1, variance)
        dist = range_measurement.distance

        # If the landmark is actually secretly a pose, then we use RangeFactorPose2/3
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
        """
        Add an odometry measurement to the factor graph.
        
        Args:
            odom_measurement: The odometry measurement to add.
        """
        # The indices of the related poses in the odometry measurement
        i_symbol = get_gtsam_symbol_from_key(odom_measurement.key1)
        j_symbol = get_gtsam_symbol_from_key(odom_measurement.key2)

        # Add the factor to the factor graph
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
        
        variances = pose.marginal_covariance.diagonal()
        noise_model = noiseModel.Diagonal.Variances(variances)
        if isinstance(pose, Pose2D):
            pose_init = get_pose2_from_matrix(pose.transformation_matrix)
            prior_factor = PriorFactorPose2(sym, pose_init, noise_model)
        elif isinstance(pose, Pose3D):
            pose_init = get_pose3_from_matrix(pose.transformation_matrix)
            prior_factor = PriorFactorPose3(sym, pose_init, noise_model)
        else:
            raise ValueError(f"Unknown pose type: {type(pose)}")

        self.factor_graph.push_back(prior_factor)

    def _specific_initialize_pose(self, pose: Union[Pose2D, Pose3D]) -> None:
        """
        Estimator-specific code to initialize the pose of a key in the estimator.

        Args:
            pose: The pose to initialize.
        """
        sym = get_gtsam_symbol_from_key(pose.key)
        if isinstance(pose, Pose2D):
            pose_init = get_pose2_from_matrix(pose.transformation_matrix)
        elif isinstance(pose, Pose3D):
            pose_init = get_pose3_from_matrix(pose.transformation_matrix)
        else:
            raise ValueError(f"Unknown pose type: {type(pose)}")

        self.gtsam_estimate.insert(sym, pose_init)

        # If the pose is also index 0, we add a prior to fix the gauge freedom
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
            point: The point to initialize.
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

            variances = point.marginal_covariance.diagonal()
            noise_model = noiseModel.Diagonal.Variances(variances)
            if isinstance(point, Point2D):
                point_init = np.array(point.position)
                prior_factor = PriorFactorPoint2(sym, point_init, noise_model)
            elif isinstance(point, Point3D):
                point_init = np.array(point.position)
                prior_factor = PriorFactorPoint3(sym, point_init, noise_model)
            else:
                raise ValueError(f"Unknown point type: {type(point)}")

            self.factor_graph.push_back(prior_factor)
        except Exception as e:
            rospy.logerr(f"Failed to add prior to point {point.key}: {e}")

    def add_depth(self, depth_measurement: DepthMeasurement) -> None:
        """
        Add a depth measurement to the estimator.
        
        We implement this as a "poor man's depth measurement" by placing a prior
        on the translation of the pose. We add very low precision to the x and y
        translation, and high precision to the z translation.

        Args:
            depth_measurement: The depth measurement to be added.
        """
        depth = depth_measurement.depth
        depth_precision = 1 / depth_measurement.variance
        other_translation_precision = (
            depth_precision / 100000.0
        )  # Very low precision for x and y
        pose_symbol = get_gtsam_symbol_from_key(depth_measurement.key)

        # How many dimensions do we need to fill for a pseudo depth measurement?
        fill_dim = self.dimension - 1

        # Make the noise model for the translation prior
        noise_model = noiseModel.Diagonal.Sigmas(
            np.array(
                [other_translation_precision] * fill_dim
                + [depth_precision]  # High precision for z translation
            )
        )
        fake_translation = np.array(
            [0.0] * fill_dim + [depth]  # Fake translation in the z direction
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

        # Run GTSAM optimization
        rospy.loginfo("Running GTSAM optimization...")
        result = solve_with_levenberg_marquardt(
            self.factor_graph, 
            self.gtsam_estimate, 
            return_all_iterates=False,
            enable_diagnostics=self.enable_diagnostics,
            try_gauss_newton=self.try_gauss_newton_fallback
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
