#!/usr/bin/env python3
"""
Simple ROS node that wraps ROSEstimatorManager and publishes periodic state estimates.

This node is now a thin entry point that uses the refactored estimator.ros module
for all ROS-specific functionality.
"""

import rospy
from estimator.ros import (
    ROSEstimatorManager,
    publish_pose_msgs,
    create_node_and_label_markers,
    create_edge_marker,
    create_range_edge_marker,
)
from estimator.types.enums import EstimatorMode
from spurdog_acomms.msg import PoseFactorStamped
from visualization_msgs.msg import MarkerArray


def main() -> None:
    rospy.init_node("estimator_node")

    # Get ROS parameters
    dimension = rospy.get_param("~dimension", 3)
    mode_name = rospy.get_param("~mode", "GTSAM_LM")
    agents = rospy.get_param("~agents", ["actor_0"])

    if isinstance(agents, str):
        agents = [agent.strip() for agent in agents.split(",") if agent.strip()]

    print(f"Estimator node starting with agents: {agents}")

    # Parse mode
    try:
        mode = getattr(EstimatorMode, mode_name)
    except Exception:
        rospy.logwarn(f"Invalid mode '{mode_name}', defaulting to GTSAM_LM")
        mode = EstimatorMode.GTSAM_LM

    if not agents:
        rospy.logerr("No agents specified, estimator will not process any data.")
        return

    # Create ROS estimator manager (handles subscribers automatically)
    manager = ROSEstimatorManager(
        "actor_0",
        mode=mode,
        dimension=dimension,
        enable_diagnostics=False,
        try_gauss_newton_fallback=False,
    )

    # Set up publishers
    pose_pub = rospy.Publisher("/estimator/pose", PoseFactorStamped, queue_size=2)
    marker_pub = rospy.Publisher("/estimator/markers", MarkerArray, queue_size=1)

    def timer_cb(event) -> None:
        """Timer callback for periodic updates and publishing."""
        # Update estimator
        manager.update()

        # Publish pose messages
        publish_pose_msgs(manager.estimator, manager.most_recent_pose_keys, pose_pub)

        # Create and publish RViz visualization markers
        all_vars = manager.get_all_estimated_variables()
        rospy.loginfo_throttle(
            5.0, f"Publishing {len(all_vars)} estimated variables as markers"
        )

        # Get node and label markers
        node_label_markers, pose_keys, pose_positions, point_positions = (
            create_node_and_label_markers(all_vars)
        )

        # Build marker array
        marker_arr = MarkerArray()
        marker_arr.markers.extend(node_label_markers)

        # Add edge marker (odometry trajectory)
        edge_marker = create_edge_marker(pose_keys, pose_positions)
        if edge_marker:
            marker_arr.markers.append(edge_marker)

        # Add range edge marker
        var_positions = {**pose_positions, **point_positions}
        range_edge_marker = create_range_edge_marker(manager.estimator, var_positions)
        if range_edge_marker:
            marker_arr.markers.append(range_edge_marker)

        # Publish all markers
        marker_pub.publish(marker_arr)

    # Set up timer for periodic updates
    rate = rospy.get_param("~rate", 0.1)
    rospy.Timer(rospy.Duration(1.0 / float(rate)), timer_cb)

    rospy.loginfo("Estimator node started")
    rospy.spin()


if __name__ == "__main__":
    main()
