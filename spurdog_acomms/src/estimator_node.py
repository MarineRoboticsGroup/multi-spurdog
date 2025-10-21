#!/usr/bin/env python3
"""
Simple ROS node that wraps EstimatorManager and publishes periodic state estimates.
"""

import rospy
from estimator.estimator_manager import EstimatorBankManager
from estimator.estimator_helpers import EstimatorMode
from spurdog_acomms.msg import PoseFactorStamped


def main():
    rospy.init_node("estimator_node")

    dimension = rospy.get_param("~dimension", 3)
    mode_name = rospy.get_param("~mode", "GTSAM_LM")
    agents = rospy.get_param("~agents", ["actor_0"])

    try:
        mode = getattr(EstimatorMode, mode_name)
    except Exception:
        rospy.logwarn(f"Invalid mode '{mode_name}', defaulting to GTSAM_LM")
        mode = EstimatorMode.GTSAM_LM

    if not agents:
        rospy.logerr("No agents specified, estimator will not process any data.")
        return

    manager = EstimatorBankManager(mode=mode, dimension=dimension, agents=set(agents))

    # publishers per-agent
    _pose_pub_map = {}
    for agent in manager.agents:
        topic = f"/{agent}/estimator/pose"
        _pose_pub_map[agent] = rospy.Publisher(topic, PoseFactorStamped, queue_size=2)

    def timer_cb(event):
        try:
            manager.update_estimators()
        except Exception as e:
            rospy.logwarn(f"Estimator update failed: {e}")
            return

        # publish poses for known agents
        for key in list(manager.agents):
            rospy.logwarn_once(f"Not publishing pose for agent {key}, logic not implemented yet.")
            continue

            try:
                pose = manager.estimator.get_pose(key)
            except Exception:
                continue

            msg = PoseFactorStamped()
            msg.key1 = key
            msg.key2 = key
            # Pose3D has .position and .orientation tuples
            try:
                msg.pose.pose.position.x = float(pose.position[0])
                msg.pose.pose.position.y = float(pose.position[1])
                msg.pose.pose.position.z = float(pose.position[2])
            except Exception:
                pass
            try:
                msg.pose.pose.orientation.x = float(pose.orientation[0])
                msg.pose.pose.orientation.y = float(pose.orientation[1])
                msg.pose.pose.orientation.z = float(pose.orientation[2])
                msg.pose.pose.orientation.w = float(pose.orientation[3])
            except Exception:
                pass

            pub = manager._pose_pub_map.get(key)
            if pub:
                pub.publish(msg)

    rate = rospy.get_param("~rate", 1.0)
    rospy.Timer(rospy.Duration(1.0 / float(rate)), timer_cb)

    rospy.loginfo("Estimator node started")
    rospy.spin()


if __name__ == "__main__":
    main()
