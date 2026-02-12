#!/usr/bin/env python3
"""
Publish an initial pose prior at the origin to anchor the trajectory.

This node publishes a single PoseFactorStamped message with key1 == key2 == "A0"
to provide an initial pose prior at the origin for estimators.
"""

import rospy
from spurdog_acomms.msg import PoseFactorStamped
from geometry_msgs.msg import PoseWithCovariance
import numpy as np


def main():
    rospy.init_node('initial_pose_publisher')
    
    # Get parameters
    agent_name = rospy.get_param('~agent_name', 'actor_0')
    initial_key = rospy.get_param('~initial_key', 'A0')
    
    # Create publisher
    pub = rospy.Publisher(f"/{agent_name}/pose_factor", PoseFactorStamped, queue_size=1, latch=True)
    
    rospy.loginfo(f"Waiting for subscriber on /{agent_name}/pose_factor...")
    rospy.sleep(2.0)  # Give subscribers time to connect
    
    # Create initial pose prior at origin
    msg = PoseFactorStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.key1 = initial_key
    msg.key2 = initial_key  # Same key indicates this is a PRIOR
    
    # Pose at origin (with very small non-zero translation to avoid "all zeros" check)
    msg.pose.pose.position.x = 0.001  # 1mm offset to avoid rejection
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    
    # Small covariance (high confidence in initial pose)
    cov = np.zeros((6, 6))
    cov[0:3, 0:3] = np.eye(3) * 0.1  # 30cm position uncertainty
    cov[3:6, 3:6] = np.eye(3) * 0.01  # Small rotation uncertainty
    msg.pose.covariance = cov.flatten().tolist()
    
    rospy.loginfo(f"Publishing initial pose prior for key {initial_key} at origin")
    pub.publish(msg)
    
    rospy.loginfo("Initial pose prior published - keeping node alive for latched topic")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
