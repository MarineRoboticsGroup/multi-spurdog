"""
ROS publisher utilities for estimator.

Handles publishing pose estimates as ROS messages.
LOCAL FRAME TRANSFORMATION: Poses are optimized in local frame (first pose at origin)
but published in world frame for visualization and downstream processing.
"""

import rospy
import numpy as np
from typing import Dict, Optional
from scipy.spatial.transform import Rotation as R

from spurdog_acomms.msg import PoseFactorStamped

from ..types.key import Key
from ..estimator import Estimator
from ..utils.transformations import get_quat_from_theta


def publish_pose_msgs(
    estimator: Estimator,
    most_recent_pose_keys: Dict[str, Key],
    pose_pub: rospy.Publisher,
) -> None:
    """
    Publish PoseFactorStamped messages for all known agents.
    
    For each agent, retrieves the most recent pose estimate and publishes
    it as a PoseFactorStamped message (with key1 == key2 to indicate a state
    estimate rather than a factor).
    
    Args:
        estimator: The estimator to query for poses
        most_recent_pose_keys: Dict mapping agent names to their most recent Keys
        pose_pub: ROS publisher for PoseFactorStamped messages
    """
    for agent, key in most_recent_pose_keys.items():
        try:
            pose = estimator.get_pose_from_estimator(key)
        except Exception as e:
            rospy.logerr(f"Failed to get pose for key {key} | error: {e}")
            continue
        
        # Create message
        msg = PoseFactorStamped()
        msg.key1 = key
        msg.key2 = key  # Same keys indicates this is a state estimate, not a factor
        
        # Set position
        try:
            msg.pose.pose.position.x = float(pose.position[0])
            msg.pose.pose.position.y = float(pose.position[1])
            if isinstance(pose.position, tuple) and len(pose.position) > 2:
                msg.pose.pose.position.z = float(pose.position[2])
            else:
                msg.pose.pose.position.z = 0.0
        except Exception as e:
            rospy.logerr(
                f"Pose position for key {key} has unexpected format: {pose.position} | error: {e}"
            )
            continue
        
        # Set orientation
        try:
            if isinstance(pose.orientation, float):
                # 2D pose: orientation is a single angle (yaw)
                quat = get_quat_from_theta(pose.orientation)
                msg.pose.pose.orientation.x = float(quat[0])
                msg.pose.pose.orientation.y = float(quat[1])
                msg.pose.pose.orientation.z = float(quat[2])
                msg.pose.pose.orientation.w = float(quat[3])
            elif isinstance(pose.orientation, tuple) and len(pose.orientation) == 4:
                # 3D pose: orientation is a quaternion
                msg.pose.pose.orientation.x = float(pose.orientation[0])
                msg.pose.pose.orientation.y = float(pose.orientation[1])
                msg.pose.pose.orientation.z = float(pose.orientation[2])
                msg.pose.pose.orientation.w = float(pose.orientation[3])
            else:
                rospy.logerr(
                    f"Pose orientation for key {key} has unexpected format: {pose.orientation}"
                )
                continue
        except Exception as e:
            rospy.logerr(
                f"Pose orientation for key {key} has unexpected format: {pose.orientation} | error: {e}"
            )
            continue
        
        # Publish the message
        pose_pub.publish(msg)


def publish_all_pose_estimates(
    estimator: Estimator,
    pose_pub: rospy.Publisher,
    world_frame_reference: Optional[Dict] = None,
    use_local_frame: bool = True,
) -> None:
    """
    Publish PoseFactorStamped messages for ALL optimized poses in the estimator.
    
    This is useful after batch optimization to publish the complete trajectory.
    Each pose is published with key1 == key2 to indicate a state estimate.
    
    LOCAL FRAME TRANSFORMATION:
    - Poses are optimized in local frame (first pose at origin)
    - Before publishing, transform back to world frame using world_frame_reference
    - This ensures mission_processor and visualization receive world coordinates
    
    Args:
        estimator: The estimator containing optimized poses (in LOCAL frame)
        pose_pub: ROS publisher for PoseFactorStamped messages
        world_frame_reference: Dict with 'position', 'rotation', 'rotation_inv' for transformation
        use_local_frame: If True, apply local→world transformation (default: True)
    """
    # Get all poses from the current estimate
    for key, pose in estimator.current_estimate.pose_map.items():
        # Create message
        msg = PoseFactorStamped()
        msg.key1 = str(key)  # Convert Key to string for ROS message
        msg.key2 = str(key)  # Same keys indicates this is a state estimate, not a factor
        
        # Extract pose in LOCAL frame (as optimized)
        position_local = np.array(pose.position)
        
        # Transform from LOCAL frame to WORLD frame for publishing
        if use_local_frame and world_frame_reference is not None:
            # Apply inverse transformation: world = R0 @ local + p0
            position_world = (
                world_frame_reference['rotation'].apply(position_local) + 
                world_frame_reference['position']
            )
            
            # Transform orientation
            if isinstance(pose.orientation, float):
                # 2D pose: orientation is yaw angle
                # Create rotation from yaw, transform to world frame
                R_local = R.from_euler('z', pose.orientation)
                R_world = world_frame_reference['rotation'] * R_local
                orientation_world = R_world.as_quat()
            elif isinstance(pose.orientation, tuple) and len(pose.orientation) == 4:
                # 3D pose: orientation is quaternion
                R_local = R.from_quat(pose.orientation)
                R_world = world_frame_reference['rotation'] * R_local
                orientation_world = R_world.as_quat()
            else:
                rospy.logerr(
                    f"[publish_all] Unexpected orientation format for key {key}: {pose.orientation}"
                )
                continue
        else:
            # No transformation - publish in local frame (for rollback/comparison)
            position_world = position_local
            if isinstance(pose.orientation, float):
                orientation_world = get_quat_from_theta(pose.orientation)
            elif isinstance(pose.orientation, tuple) and len(pose.orientation) == 4:
                orientation_world = pose.orientation
            else:
                rospy.logerr(
                    f"[publish_all] Unexpected orientation format for key {key}: {pose.orientation}"
                )
                continue
        
        # Set position (in WORLD frame after transformation)
        try:
            msg.pose.pose.position.x = float(position_world[0])
            msg.pose.pose.position.y = float(position_world[1])
            if len(position_world) > 2:
                msg.pose.pose.position.z = float(position_world[2])
            else:
                msg.pose.pose.position.z = 0.0
        except Exception as e:
            rospy.logerr(
                f"[publish_all] Pose position for key {key} has unexpected format: {position_world} | error: {e}"
            )
            continue
        
        # Set orientation (in WORLD frame after transformation)
        try:
            msg.pose.pose.orientation.x = float(orientation_world[0])
            msg.pose.pose.orientation.y = float(orientation_world[1])
            msg.pose.pose.orientation.z = float(orientation_world[2])
            msg.pose.pose.orientation.w = float(orientation_world[3])
        except Exception as e:
            rospy.logerr(
                f"[publish_all] Pose orientation for key {key} has unexpected format: {orientation_world} | error: {e}"
            )
            continue
        
        # Publish the message
        pose_pub.publish(msg)
        # Small delay to prevent flooding ROS messaging system and ensure delivery
        rospy.sleep(0.01)  # 10ms delay between messages (100 msgs/sec max)
    
    rospy.loginfo(f"Published {len(estimator.current_estimate.pose_map)} optimized poses")
