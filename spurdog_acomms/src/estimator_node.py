#!/usr/bin/env python3
"""
Simple ROS node that wraps EstimatorManager and publishes periodic state estimates.
"""


import rospy
from estimator.estimator_manager import EstimatorManager
from estimator.estimator_helpers import EstimatorMode, get_quat_from_theta, Pose2D, Pose3D, Point2D, Point3D, Key
from spurdog_acomms.msg import PoseFactorStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as ROSPoint
from typing import Dict


def main() -> None:
    rospy.init_node("estimator_node")

    dimension = rospy.get_param("~dimension", 3)
    mode_name = rospy.get_param("~mode", "CORA")
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

    # Publisher for RViz markers
    marker_pub = rospy.Publisher("/estimator/markers", MarkerArray, queue_size=1)



    from typing import Any, List, Tuple, Optional

    def make_marker(
        idx: int,
        key: Key,
        var: Any,
        color: Tuple[float, float, float, float],
        shape: int = Marker.SPHERE,
        scale: float = 0.2,
    ns: str = "estimator",
    text: Optional[str] = None,
        z_offset: float = 0.0
    ) -> Marker:
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = idx
        m.type = shape
        m.action = Marker.ADD
        if hasattr(var, 'position'):
            pos = var.position
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = (float(pos[2]) if len(pos) > 2 else 0.0) + z_offset
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime = rospy.Duration(0)
        if shape == Marker.TEXT_VIEW_FACING:
            m.text = text if text is not None else str(key)
        else:
            m.text = ""
        return m

    def publish_pose_msgs(manager: EstimatorManager, pose_pub: rospy.Publisher) -> None:
        """Publish PoseFactorStamped messages for all known agents."""
        for agent, key in manager.most_recent_pose_keys.items():
            try:
                pose = manager.estimator.get_pose_from_estimator(key)
            except Exception as e:
                rospy.logerr(f"Failed to get pose for key {key} | error: {e}")
                continue
            msg = PoseFactorStamped()
            msg.key1 = key
            msg.key2 = key
            try:
                msg.pose.pose.position.x = float(pose.position[0])
                msg.pose.pose.position.y = float(pose.position[1])
                if isinstance(pose.position, tuple) and len(pose.position) > 2:
                    msg.pose.pose.position.z = float(pose.position[2])
                else:
                    msg.pose.pose.position.z = 0.0
            except Exception as e:
                rospy.logerr(f"Pose position for key {key} has unexpected format: {pose.position} | error: {e}")
            try:
                if isinstance(pose.orientation, float):
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
                    rospy.logerr(f"Pose orientation for key {key} has unexpected format: {pose.orientation}")
            except Exception as e:
                rospy.logerr(f"Pose orientation for key {key} has unexpected format: {pose.orientation} | error: {e}")
            pose_pub.publish(msg)

    def create_node_and_label_markers(
        all_vars: Dict[Key, Any]
    ) -> Tuple[List[Marker], List[Key], Dict[Key, Tuple[float, float, float]], Dict[Key, Tuple[float, float, float]]]:
        """Return node and label markers, pose_keys (Key), and pose_positions (Key: tuple). Also adds single-letter and key labels in separate namespaces for toggling."""
        marker_arr: List[Marker] = []
        idx = 0
        color_pose: Tuple[float, float, float, float] = (0.1, 0.4, 1.0, 1.0)
        color_point: Tuple[float, float, float, float] = (0.0, 1.0, 0.2, 1.0)
        color_label: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 0.9)
        color_letter: Tuple[float, float, float, float] = (1.0, 0.7, 0.2, 1.0)
        pose_keys: List[Key] = []
        pose_positions: Dict[Key, Tuple[float, float, float]] = {}
        point_positions: Dict[Key, Tuple[float, float, float]] = {}
        for key, var in all_vars.items():
            if isinstance(var, (Pose2D, Pose3D)):
                marker_arr.append(make_marker(idx, key, var, color_pose, shape=Marker.SPHERE, scale=0.25, ns="estimator"))
                pose_keys.append(key)
                pos = var.position
                pose_positions[key] = (float(pos[0]), float(pos[1]), float(pos[2]) if len(pos) > 2 else 0.0)
                idx += 1
                # Full key label (in its own namespace, above the node)
                label = make_marker(10000+idx, key, var, color_label, shape=Marker.TEXT_VIEW_FACING, scale=0.18, ns="estimator_labels", text=str(key), z_offset=0.35)
                marker_arr.append(label)
                idx += 1
                # Single-letter label (in its own namespace, above the node, slightly offset)
                letter_marker = make_marker(20000+idx, key, var, color_letter, shape=Marker.TEXT_VIEW_FACING, scale=0.16, ns="estimator_letters", text=key.char, z_offset=0.22)
                marker_arr.append(letter_marker)
                idx += 1
            elif isinstance(var, (Point2D, Point3D)):
                point_positions[key] = (float(var.position[0]), float(var.position[1]), float(var.position[2]) if len(var.position) > 2 else 0.0)
                marker_arr.append(make_marker(idx, key, var, color_point, shape=Marker.SPHERE, scale=0.18, ns="estimator"))
                idx += 1
                # Full key label (in its own namespace, above the point)
                label = make_marker(10000+idx, key, var, color_label, shape=Marker.TEXT_VIEW_FACING, scale=0.14, ns="estimator_labels", text=str(key), z_offset=0.22)
                marker_arr.append(label)
                idx += 1
                # Single-letter label (in its own namespace, above the point, slightly offset)
                letter_marker = make_marker(20000+idx, key, var, color_letter, shape=Marker.TEXT_VIEW_FACING, scale=0.12, ns="estimator_letters", text=key.char, z_offset=0.13)
                marker_arr.append(letter_marker)
                idx += 1
        return marker_arr, pose_keys, pose_positions, point_positions

    def create_edge_marker(
        pose_keys: List[Key],
        pose_positions: Dict[Key, Tuple[float, float, float]]
    ) -> Optional[Marker]:
        """Return a Marker for edges between adjacent pose variables (using Key objects)."""
        from collections import defaultdict
        agent_tracks: Dict[str, List[Tuple[int, Key]]] = defaultdict(list)
        for k in pose_keys:
            agent, idx_str = k.char, k.index
            agent_tracks[agent].append((int(idx_str), k))
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.header.stamp = rospy.Time.now()
        edge_marker.ns = "estimator_edges"
        edge_marker.id = 50000
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.06
        edge_marker.color.r = 1.0
        edge_marker.color.g = 0.6
        edge_marker.color.b = 0.0
        edge_marker.color.a = 1.0
        edge_marker.lifetime = rospy.Duration(0)
        edge_marker.points = []
        for agent, idx_key_list in agent_tracks.items():
            idx_key_list.sort()
            for i in range(len(idx_key_list)-1):
                k1 = idx_key_list[i][1]
                k2 = idx_key_list[i+1][1]
                if k1 in pose_positions and k2 in pose_positions:
                    p1 = pose_positions[k1]
                    p2 = pose_positions[k2]
                    pt1 = ROSPoint(x=p1[0], y=p1[1], z=p1[2])
                    pt2 = ROSPoint(x=p2[0], y=p2[1], z=p2[2])
                    edge_marker.points.append(pt1)
                    edge_marker.points.append(pt2)
        return edge_marker if edge_marker.points else None

    def create_range_edge_marker(
        manager: EstimatorManager,
        var_positions: Dict[Key, Tuple[float, float, float]]
    ) -> Optional[Marker]:
        """Return a Marker for range edges (range factors) between variables, using Key objects and coordinate tuples."""
        # This assumes manager.estimator.range_measurement_pairs is a list of (Key, Key)
        range_edges = manager.estimator.range_measurement_pairs

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "estimator_range_edges"
        marker.id = 60000
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.8
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        marker.points = []
        for k1, k2 in range_edges:
            if k1 in var_positions and k2 in var_positions:
                p1 = var_positions[k1]
                p2 = var_positions[k2]
                pt1 = ROSPoint(x=p1[0], y=p1[1], z=p1[2])
                pt2 = ROSPoint(x=p2[0], y=p2[1], z=p2[2])
                marker.points.append(pt1)
                marker.points.append(pt2)

        return marker if marker.points else None

    def timer_cb(event) -> None:
        try:
            manager.update()
        except Exception as e:
            rospy.logwarn(f"Estimator update failed: {e}")
            return

        # Publish pose messages
        publish_pose_msgs(manager, _pose_pub_map)

        # --- RViz visualization ---
        all_vars = manager.get_all_estimated_variables()
        rospy.loginfo_throttle(5.0, f"Publishing {len(all_vars)} estimated variables as markers")
        node_label_markers, pose_keys, pose_positions, point_positions = create_node_and_label_markers(all_vars)
        marker_arr = MarkerArray()
        marker_arr.markers.extend(node_label_markers)
        edge_marker = create_edge_marker(pose_keys, pose_positions)
        if edge_marker:
            marker_arr.markers.append(edge_marker)
        # Add range edge marker
        var_positions = {**pose_positions, **point_positions}
        range_edge_marker = create_range_edge_marker(manager, var_positions)
        if range_edge_marker:
            marker_arr.markers.append(range_edge_marker)
        marker_pub.publish(marker_arr)

    rate = rospy.get_param("~rate", 0.1)
    rospy.Timer(rospy.Duration(1.0 / float(rate)), timer_cb)

    rospy.loginfo("Estimator node started")
    rospy.spin()


if __name__ == "__main__":
    main()
