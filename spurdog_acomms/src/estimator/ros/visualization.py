"""
RViz visualization utilities for estimator.

Creates markers for poses, points, edges, and range measurements.
"""

import rospy
from typing import Dict, List, Tuple, Optional, Any
from collections import defaultdict

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as ROSPoint

from ..types.key import Key
from ..types.variables import Pose2D, Pose3D, Point2D, Point3D
from ..estimator import Estimator


def make_marker(
    idx: int,
    key: Key,
    var: Any,
    color: Tuple[float, float, float, float],
    shape: int = Marker.SPHERE,
    scale: float = 0.2,
    ns: str = "estimator",
    text: Optional[str] = None,
    z_offset: float = 0.0,
) -> Marker:
    """
    Create a single visualization marker.
    
    Args:
        idx: Unique marker ID
        key: Key associated with this variable
        var: Variable (Pose2D, Pose3D, Point2D, or Point3D) to visualize
        color: RGBA color tuple (r, g, b, a) with values 0-1
        shape: Marker shape constant from visualization_msgs.msg.Marker
        scale: Size of the marker
        ns: Namespace for the marker (for grouping/toggling)
        text: Text to display (for TEXT_VIEW_FACING markers)
        z_offset: Vertical offset for positioning labels
        
    Returns:
        Marker message ready to publish
    """
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = idx
    m.type = shape
    m.action = Marker.ADD
    
    # Set position from variable
    if hasattr(var, 'position'):
        pos = var.position
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = (float(pos[2]) if len(pos) > 2 else 0.0) + z_offset
    
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    m.lifetime = rospy.Duration(0)
    
    # Set text for text markers
    if shape == Marker.TEXT_VIEW_FACING:
        m.text = text if text is not None else str(key)
    else:
        m.text = ""
    
    return m


def create_node_and_label_markers(
    all_vars: Dict[Key, Any]
) -> Tuple[List[Marker], List[Key], Dict[Key, Tuple[float, float, float]], Dict[Key, Tuple[float, float, float]]]:
    """
    Create visualization markers for all estimated variables.
    
    Creates sphere markers for poses and points, plus text labels showing
    both full key names and single-letter agent identifiers (in separate
    namespaces for independent toggling in RViz).
    
    Args:
        all_vars: Dictionary mapping Keys to estimated variables
        
    Returns:
        Tuple of:
        - List of all markers (nodes + labels)
        - List of pose Keys
        - Dict mapping pose Keys to (x, y, z) positions
        - Dict mapping point Keys to (x, y, z) positions
    """
    marker_arr: List[Marker] = []
    idx = 0
    
    # Colors for different element types
    color_pose: Tuple[float, float, float, float] = (0.1, 0.4, 1.0, 1.0)  # Blue
    color_point: Tuple[float, float, float, float] = (0.0, 1.0, 0.2, 1.0)  # Green
    color_label: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 0.9)  # White
    color_letter: Tuple[float, float, float, float] = (1.0, 0.7, 0.2, 1.0)  # Orange
    
    pose_keys: List[Key] = []
    pose_positions: Dict[Key, Tuple[float, float, float]] = {}
    point_positions: Dict[Key, Tuple[float, float, float]] = {}
    
    for key, var in all_vars.items():
        if isinstance(var, (Pose2D, Pose3D)):
            # Create pose node marker
            marker_arr.append(
                make_marker(idx, key, var, color_pose, shape=Marker.SPHERE, scale=0.25, ns="estimator")
            )
            pose_keys.append(key)
            
            # Extract position
            pos = var.position
            pose_positions[key] = (float(pos[0]), float(pos[1]), float(pos[2]) if len(pos) > 2 else 0.0)
            idx += 1
            
            # Full key label (in its own namespace, above the node)
            label = make_marker(
                10000 + idx, key, var, color_label,
                shape=Marker.TEXT_VIEW_FACING, scale=0.18,
                ns="estimator_labels", text=str(key), z_offset=0.35
            )
            marker_arr.append(label)
            idx += 1
            
            # Single-letter label (in its own namespace, slightly offset)
            letter_marker = make_marker(
                20000 + idx, key, var, color_letter,
                shape=Marker.TEXT_VIEW_FACING, scale=0.16,
                ns="estimator_letters", text=key.char, z_offset=0.22
            )
            marker_arr.append(letter_marker)
            idx += 1
            
        elif isinstance(var, (Point2D, Point3D)):
            # Extract position
            point_positions[key] = (
                float(var.position[0]),
                float(var.position[1]),
                float(var.position[2]) if len(var.position) > 2 else 0.0
            )
            
            # Create point node marker
            marker_arr.append(
                make_marker(idx, key, var, color_point, shape=Marker.SPHERE, scale=0.18, ns="estimator")
            )
            idx += 1
            
            # Full key label (in its own namespace, above the point)
            label = make_marker(
                10000 + idx, key, var, color_label,
                shape=Marker.TEXT_VIEW_FACING, scale=0.14,
                ns="estimator_labels", text=str(key), z_offset=0.22
            )
            marker_arr.append(label)
            idx += 1
            
            # Single-letter label (in its own namespace, slightly offset)
            letter_marker = make_marker(
                20000 + idx, key, var, color_letter,
                shape=Marker.TEXT_VIEW_FACING, scale=0.12,
                ns="estimator_letters", text=key.char, z_offset=0.13
            )
            marker_arr.append(letter_marker)
            idx += 1
    
    return marker_arr, pose_keys, pose_positions, point_positions


def create_edge_marker(
    pose_keys: List[Key],
    pose_positions: Dict[Key, Tuple[float, float, float]]
) -> Optional[Marker]:
    """
    Create a marker showing edges between consecutive pose variables.
    
    Creates LINE_LIST marker connecting sequential poses for each agent
    (A0->A1->A2, B0->B1->B2, etc.).
    
    Args:
        pose_keys: List of pose Keys
        pose_positions: Dict mapping pose Keys to (x, y, z) positions
        
    Returns:
        LINE_LIST Marker or None if no edges to visualize
    """
    # Group poses by agent
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
    edge_marker.scale.x = 0.06  # Line width
    edge_marker.color.r = 1.0
    edge_marker.color.g = 0.6
    edge_marker.color.b = 0.0
    edge_marker.color.a = 1.0
    edge_marker.lifetime = rospy.Duration(0)
    edge_marker.points = []
    
    # Create edges between consecutive poses for each agent
    for agent, idx_key_list in agent_tracks.items():
        idx_key_list.sort()  # Sort by index
        for i in range(len(idx_key_list) - 1):
            k1 = idx_key_list[i][1]
            k2 = idx_key_list[i + 1][1]
            if k1 in pose_positions and k2 in pose_positions:
                p1 = pose_positions[k1]
                p2 = pose_positions[k2]
                pt1 = ROSPoint(x=p1[0], y=p1[1], z=p1[2])
                pt2 = ROSPoint(x=p2[0], y=p2[1], z=p2[2])
                edge_marker.points.append(pt1)
                edge_marker.points.append(pt2)
    
    return edge_marker if edge_marker.points else None


def create_range_edge_marker(
    estimator: Estimator,
    var_positions: Dict[Key, Tuple[float, float, float]]
) -> Optional[Marker]:
    """
    Create a marker showing range measurement edges.
    
    Creates LINE_LIST marker connecting variables that have range
    measurements between them.
    
    Args:
        estimator: Estimator with range_measurement_pairs attribute
        var_positions: Dict mapping Keys to (x, y, z) positions for all variables
        
    Returns:
        LINE_LIST Marker or None if no range edges to visualize
    """
    # Get range measurement pairs from estimator
    range_edges = estimator.range_measurement_pairs

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "estimator_range_edges"
    marker.id = 60000
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.04  # Line width
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.8
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0)
    marker.points = []
    
    # Create lines between variables with range measurements
    for k1, k2 in range_edges:
        if k1 in var_positions and k2 in var_positions:
            p1 = var_positions[k1]
            p2 = var_positions[k2]
            pt1 = ROSPoint(x=p1[0], y=p1[1], z=p1[2])
            pt2 = ROSPoint(x=p2[0], y=p2[1], z=p2[2])
            marker.points.append(pt1)
            marker.points.append(pt2)

    return marker if marker.points else None
