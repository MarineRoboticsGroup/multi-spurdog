#!/usr/bin/env python3
from .estimator import Estimator
from .gtsam_estimator import GtsamEstimator
from .estimator_helpers import (
    Key,
    KeyPair,
    RangeMeasurement,
    OdometryMeasurement,
    OdometryMeasurement2D,
    OdometryMeasurement3D,
    convert_odom3_to_odom2,
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

from spurdog_acomms.msg import (
    Bar30SoundSpeed,
    RangeFactorStamped,
    PoseFactorStamped,
    AcommsCycleStatus,
    ReceivedSignalStats,
    BasicGraphUpdate,
    AdvancedGraphUpdate,
)

import rospy

from typing import Union, List, Optional, Set, Dict, Tuple

import math
import numpy as np

class EstimatorManager:
    def __init__(
        self,
        agent_name: str,
        mode: EstimatorMode = EstimatorMode.GTSAM_LM,
        dimension: int = 3,
    ):
        """
        Initializes the EstimatorManager with the specified mode and dimension.
        Args:
            agent_name (str): The name of the agent using this estimator.
            mode (EstimatorMode): The mode of the estimator.
            dimension (int): The dimension of the state space (2 or 3).
        """
        self.dimension = dimension
        self.agent_name = agent_name

        # construct the estimator based on the mode
        if mode == EstimatorMode.GTSAM_LM:
            self.estimator = GtsamEstimator(
                mode=EstimatorMode.GTSAM_LM,
                dimension=dimension,
                odom_factor_type="between",
            )
        elif mode == EstimatorMode.CORA:
            from .cora_estimator import CoraEstimator
            self.estimator = CoraEstimator(mode=EstimatorMode.CORA, dimension=dimension)
        else:
            raise ValueError(f"Unsupported estimator mode: {mode}")

        # construct the subscriber
        self.pose_subscriber = rospy.Subscriber(
            f"{self.agent_name}/pose_factor",
            PoseFactorStamped,
            self.handle_pose_factor,
        )
        self.range_subscriber = rospy.Subscriber(
            f"{self.agent_name}/range_factor",
            RangeFactorStamped,
            self.handle_range_factor,
        )

        self.has_received_data = False

    def _add_pose_prior(self, msg: PoseFactorStamped):
        """
        Add a pose prior to the estimator.
        Args:
            msg (PoseFactorStamped): The pose factor message containing the prior information.
        """
        if msg.key1 != msg.key2:
            raise ValueError(
                f"Pose prior must have identical keys, got {msg.key1} and {msg.key2}"
            )

            # initialize the pose with the provided pose
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        # covariance needs to be a 6x6 matrix
        new_pose = Pose3D(
            key=msg.key1,
            position=position,
            orientation=orientation,
            marginal_covariance=np.array(msg.pose.covariance),
        )
        # initialize and add prior
        try:
            rospy.logdebug(f"Initializing pose for key: {msg.key1}")
            self.estimator.initialize_pose(new_pose)
        except Exception:
            # initialization may be a no-op for some estimator implementations
            rospy.logdebug(f"initialize_pose failed for {msg.key1}")
        try:
            rospy.logdebug(f"Adding pose prior for key: {msg.key1}")
            self.estimator.add_pose_prior(new_pose)
        except Exception as e:
            rospy.logwarn(f"add_pose_prior failed: {e}")

    def _add_odom_measurement(self, msg: PoseFactorStamped):
        if msg.key1 == msg.key2:
            raise ValueError(
                f"Odometry measurement must have different keys, got {msg.key1} and {msg.key2}"
            )

        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # ROS PoseWithCovariance uses a 6x6 covariance stored in row-major order
        cov_list = list(msg.pose.covariance)
        try:
            cov_mat = np.array(cov_list).reshape((6, 6))
        except Exception:
            cov_mat = np.zeros((6, 6))

        # extract standard deviations and correlations for RelPoseCovar6
        sx = float(np.sqrt(max(0.0, cov_mat[0, 0])))
        sy = float(np.sqrt(max(0.0, cov_mat[1, 1])))
        sz = float(np.sqrt(max(0.0, cov_mat[2, 2])))
        spsi = float(np.sqrt(max(0.0, cov_mat[3, 3])))

        # guard against zero sigmas
        if sx == 0.0:
            sx = 1e-6
        if sy == 0.0:
            sy = 1e-6
        if spsi == 0.0:
            spsi = 1e-6

        rho_xy = float(cov_mat[0, 1] / (sx * sy)) if (sx * sy) != 0 else 0.0
        rho_xpsi = float(cov_mat[0, 3] / (sx * spsi)) if (sx * spsi) != 0 else 0.0
        rho_ypsi = float(cov_mat[1, 3] / (sy * spsi)) if (sy * spsi) != 0 else 0.0

        covar = RelPoseCovar6(
            relative_pose_sigma_x=sx,
            relative_pose_sigma_y=sy,
            relative_pose_sigma_z=sz,
            relative_pose_sigma_psi=spsi,
            relative_pose_rho_xy=rho_xy,
            relative_pose_rho_xpsi=rho_xpsi,
            relative_pose_rho_ypsi=rho_ypsi,
        )

        odom = OdometryMeasurement3D(
            key_pair=KeyPair(msg.key1, msg.key2),
            relative_translation=(tx, ty, tz),
            relative_rotation=(qx, qy, qz, qw),
            covariance=covar,
        )

        if self.dimension == 2:
            try:
                odom = convert_odom3_to_odom2(odom)
            except Exception as e:
                rospy.logerr(f"Failed to convert 3D odometry to 2D: {e}")
                return

        try:
            rospy.logdebug(f"Adding odometry measurement: {odom}")
            self.estimator.add_odometry(odom)
        except Exception as e:
            rospy.logerr(f"Failed to add odometry measurement: {e}")

    def handle_pose_factor(self, msg: PoseFactorStamped):
        """
        Process the graph update with the provided data.
        This method should be called after adding all measurements.

        PoseFactorStamped
            Header header
            string key1
            string key2
            geometry_msgs/PoseWithCovariance pose
        """

        # if keys are the same, then this is a pose prior
        try:
            if msg.key1 == msg.key2:
                self._add_pose_prior(msg)
            else:
                self._add_odom_measurement(msg)

        except Exception as e:
            rospy.logerr(f"Error in handle_pose_factor: {e}")

        self.has_received_data = True

    def handle_range_factor(self, msg: RangeFactorStamped):
        """
        Process the range factor message and add it to the estimator.


        # RangeFactorStamped
        #   Header header
        #   string key1
        #   string key2
        #   float32 meas_range
        #   float32 range_sigma
        #   float32 depth1
        #   float32 depth2

        """
        try:
            if msg.key1 == msg.key2:
                raise ValueError(
                    f"Range measurement must have different keys, got {msg.key1} and {msg.key2}"
                )

            kp = KeyPair(msg.key1, msg.key2)
            # treat range_sigma as standard deviation
            try:
                variance = float(msg.range_sigma) ** 2
            except Exception:
                variance = (
                    float(msg.meas_range) * 0.01 if msg.meas_range != 0.0 else 1.0
                )

            depth1 = None
            depth2 = None
            # if depths are finite and not zero, use them
            if (
                math.isfinite(msg.depth1)
                and math.isfinite(msg.depth2)
                and (msg.depth1 != 0.0 or msg.depth2 != 0.0)
            ):
                depth1 = float(msg.depth1)
                depth2 = float(msg.depth2)

            range_meas = RangeMeasurement(
                key_pair=kp,
                distance=float(msg.meas_range),
                variance=variance,
                depth1=depth1,
                depth2=depth2,
            )

            try:
                self.estimator.add_range(range_meas)
            except Exception as e:
                rospy.logwarn(f"Failed to add range measurement: {e}")

        except Exception as e:
            rospy.logerr(f"Error in handle_range_factor: {e}")

        self.has_received_data = True

    def update(self):
        """
        Update the estimator with the current measurements.
        """
        if not self.has_received_data:
            rospy.logdebug("No new data received, skipping update.")
            return

        try:
            self.estimator.update()
        except Exception as e:
            rospy.logwarn(f"Estimator update failed: {e}")

        self.has_received_data = False

class EstimatorBankManager:
    """
    Manages a collection of estimators (one per agent name provided) and
    provides a unified interface for adding measurements and retrieving poses.
    """

    def __init__(
        self,
        agents: Set[str],
        mode: EstimatorMode = EstimatorMode.GTSAM_LM,
        dimension: int = 3,
    ):
        """
        Initializes the EstimatorManager with the specified mode and dimension.
        Args:
            agents (Set[str]): A set of agent names to initialize the estimator for.
            mode (EstimatorMode): The mode of the estimator.
            dimension (int): The dimension of the state space (2 or 3).
        """
        if agents is None:
            raise ValueError("Agents set cannot be None")
        if len(agents) == 0:
            raise ValueError("Agents set cannot be empty")

        # set the dimension
        self.dimension = dimension
        self.agents = agents
        self.estimators: Dict[str, EstimatorManager] = {}

        for a in agents:
            if not isinstance(a, str) or len(a) == 0:
                raise ValueError(f"Invalid agent name: '{a}'")

            self.estimators[a] = EstimatorManager(
                agent_name=a,
                mode=mode,
                dimension=dimension,
            )

    def update_estimators(self):
        """
        Update all estimators in the bank.
        """
        for agent, estimator in self.estimators.items():
            try:
                estimator.update()
            except Exception as e:
                rospy.logwarn(f"Estimator update failed for agent {agent}: {e}")
