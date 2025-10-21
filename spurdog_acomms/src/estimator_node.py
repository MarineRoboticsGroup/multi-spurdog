#!/usr/bin/env python3
"""
Simple ROS node that wraps EstimatorManager and publishes periodic state estimates.
"""

import rospy
from estimator.estimator_manager import EstimatorBankManager, EstimatorManager
from estimator.estimator_helpers import EstimatorMode, get_quat_from_theta
from spurdog_acomms.msg import PoseFactorStamped


def main():
    rospy.init_node("estimator_node")

    dimension = rospy.get_param("~dimension", 3)
    mode_name = rospy.get_param("~mode", "GTSAM_LM")
    agents = rospy.get_param("~agents", ["actor_0"])

    if isinstance(agents, str):
        agents = [agent.strip() for agent in agents.split(",") if agent.strip()]

    print(f"Estimator node starting with agents: {agents}")

    try:
        mode = getattr(EstimatorMode, mode_name)
    except Exception:
        rospy.logwarn(f"Invalid mode '{mode_name}', defaulting to GTSAM_LM")
        mode = EstimatorMode.GTSAM_LM

    if not agents:
        rospy.logerr("No agents specified, estimator will not process any data.")
        return

    manager = EstimatorManager("actor_0", mode=mode, dimension=dimension)

    # publishers per-agent
    topic = f"/estimator/pose"
    _pose_pub_map = rospy.Publisher(topic, PoseFactorStamped, queue_size=2)

    def timer_cb(event):
        try:
            manager.update()
        except Exception as e:
            rospy.logwarn(f"Estimator update failed: {e}")
            return

        # publish poses for known agents
        for agent, key in manager.most_recent_pose_keys.items():
            # rospy.logwarn_once(f"Not publishing pose for agent {key}, logic not implemented yet.")
            # continue

            try:
                pose = manager.estimator.get_pose_from_estimator(key)
            except Exception:
                continue

            msg = PoseFactorStamped()
            msg.key1 = key
            msg.key2 = key
            # Pose3D has .position and .orientation tuples
            try:
                msg.pose.pose.position.x = float(pose.position[0])
                msg.pose.pose.position.y = float(pose.position[1])
                if isinstance(pose.position, tuple) and len(pose.position) > 2:
                    msg.pose.pose.position.z = float(pose.position[2])
                else:
                    msg.pose.pose.position.z = 0.0
            except Exception:
                pass
            try:
                if isinstance(pose.orientation, float):
                    # 2D case, convert theta to quaternion
                    quat = get_quat_from_theta(pose.orientation)
                    msg.pose.pose.orientation.x = float(quat[0])
                    msg.pose.pose.orientation.y = float(quat[1])
                    msg.pose.pose.orientation.z = float(quat[2])
                    msg.pose.pose.orientation.w = float(quat[3])
                elif isinstance(pose.orientation, tuple) and len(pose.orientation) == 4:
                    msg.pose.pose.orientation.x = float(pose.orientation[0])
                    msg.pose.pose.orientation.y = float(pose.orientation[1])
                    msg.pose.pose.orientation.z = float(pose.orientation[2])
                    msg.pose.pose.orientation.w = float(pose.orientation[3])
                else:
                    rospy.logerr_once(
                        f"Pose orientation for key {key} has unexpected format: {pose.orientation}"
                    )
            except Exception:
                pass

            _pose_pub_map.publish(msg)

    rate = rospy.get_param("~rate", 10.0)
    rospy.Timer(rospy.Duration(1.0 / float(rate)), timer_cb)

    rospy.loginfo("Estimator node started")
    rospy.spin()


if __name__ == "__main__":
    main()
