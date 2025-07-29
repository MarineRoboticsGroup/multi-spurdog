from estimator import Estimator
from gtsam_estimator import GtsamEstimator
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
    get_theta_from_rotation_matrix,
    get_translation_from_transformation_matrix,
    get_measurement_precisions_from_covariance_matrix,
    get_diag_relpose_covar,
)

from spurdog_acomms.msg import(
    Bar30SoundSpeed, RangeFactorStamped, PoseFactorStamped,
    AcommsCycleStatus, ReceivedSignalStats,
    BasicGraphUpdate, AdvancedGraphUpdate,
)

import rospy

from typing import Union, List, Optional, Set

class EstimatorManager:
    """
    Manages the estimator and provides a unified interface for adding
    measurements and retrieving poses.
    """

    def __init__(self, mode: EstimatorMode = EstimatorMode.GTSAM_LM, dimension: int = 3, agents: Optional[Set[str]] = None):
        """
        Initializes the EstimatorManager with the specified mode and dimension.
        Args:
            mode (EstimatorMode): The mode of the estimator.
            dimension (int): The dimension of the state space (2 or 3).
            agents (Optional[Set[str]]): A set of agent names to initialize the estimator for.
        """

        # construct the estimator based on the mode
        if mode == EstimatorMode.GTSAM_LM:
            self.estimator = GtsamEstimator(
                mode=EstimatorMode.GTSAM_LM, dimension=dimension, odom_factor_type="between"
            )
        else:
            raise ValueError(f"Unsupported estimator mode: {mode}")

        # set the dimension
        self.dimension = dimension

        # if agent names are provided, initialize the agents set and set a flag
        # to indicate that we have a fixed set of agents
        self.agents: Set[str] = set()
        if agents is not None:
            self.agents = agents
        self._flexible_agents = agents is None


        # Initialize the ROS subscribers for pose and range factors
        self.pose_factor_listener = rospy.Subscriber(
            "pose_factor",
            PoseFactorStamped,
            self.handle_pose_factor,
            queue_size=10,
        )
        self.range_factor_listener = rospy.Subscriber(
            "range_factor",
            RangeFactorStamped,
            self.handle_range_factor,
            queue_size=10,
        )


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
        if msg.key1 == msg.key2:
            # initialize the pose with the provided pose
            self.estimator.initialize_pose(
                Pose3D(
                    key=msg.key1,
                    position=(
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                    ),
                    orientation=(
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w,
                    ),
                    marginal_covariance=msg.pose.covariance,
                )
            )

        raise NotImplementedError(
            "handle_pose_factor method is not implemented yet."
        )


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
        raise NotImplementedError(
            "handle_range_factor method is not implemented yet."
        )


