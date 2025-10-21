#!/usr/bin/env python3
"""
Measurement graph visualizer for /<agent>/{pose_factor, range_factor} topics.
Draws nodes for unique keys and lines for measurements.

Extras:
- Optional native Python visualization (matplotlib):
    Keys are assumed to be of the form "<Char><Index>" (e.g., "A10", "L17").
    Each character gets its own horizontal track; the index is used as the position
    along the track. Enable with ~py_plot:=true.
"""
import math
import re
import threading
from typing import Dict, Tuple, Set, Optional

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped


class MeasurementGraphViz:
    def __init__(self):
        self.agent = rospy.get_param("~agent", "actor_1")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.rate = float(rospy.get_param("~rate", 2.0))
        # Matplotlib (native Python) plotting options
        self.py_plot: bool = bool(rospy.get_param("~py_plot", False))
        self.py_plot_hz: float = float(rospy.get_param("~py_plot_hz", 2.0))
        self.py_track_spacing: float = float(rospy.get_param("~py_track_spacing", 1.0))
        self.py_index_scale: float = float(rospy.get_param("~py_index_scale", 1.0))
        self.py_aspect_equal: bool = bool(rospy.get_param("~py_aspect_equal", False))

        # State
        self._lock = threading.Lock()
        self.positions: Dict[str, Tuple[float, float, float]] = {}
        self.keys_order = []  # preserve order of discovery
        self.odom_edges: Set[Tuple[str, str]] = set()
        self.range_edges: Set[Tuple[str, str]] = set()
        # For python plotting: track per leading-letter rows
        self._track_map: Dict[str, float] = {}
        self._track_order: Dict[str, int] = {}

        # Subscribers for this agent
        pose_topic = f"/{self.agent}/pose_factor"
        range_topic = f"/{self.agent}/range_factor"
        rospy.loginfo(f"[graph_viz] Subscribing to: {pose_topic}, {range_topic}")
        self.pose_sub = rospy.Subscriber(pose_topic, PoseFactorStamped, self._pose_cb, queue_size=50)
        self.range_sub = rospy.Subscriber(range_topic, RangeFactorStamped, self._range_cb, queue_size=50)

        # Publisher
        self.pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)

        # Timer to publish RViz markers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self._publish)

        # Optional: start native Python plotting
        self._plt = None
        self._ax = None
        if self.py_plot:
            try:
                import matplotlib
                matplotlib.use('TkAgg')  # try interactive backend
                import matplotlib.pyplot as plt
                plt.ion()
                self._plt = plt
                self._fig = plt.figure("Measurement Graph (" + self.agent + ")", figsize=(8, 6))
                self._ax = self._fig.add_subplot(111)
                rospy.loginfo("[graph_viz] Native Python plotting enabled")
                self._py_timer = rospy.Timer(rospy.Duration(1.0 / max(0.1, self.py_plot_hz)), self._update_py_plot)
            except Exception as e:
                rospy.logwarn(f"[graph_viz] Failed to initialize matplotlib plotting: {e}")
                self.py_plot = False

    def _normalize_edge(self, a: str, b: str) -> Tuple[str, str]:
        return (a, b) if a <= b else (b, a)

    def _ensure_key(self, key: str) -> None:
        with self._lock:
            if key in self.positions:
                return

        pos = self._py_pos(key)

        with self._lock:
            assert pos is not None, f"Cannot determine position for key '{key}'"
            x, y = pos
            z = 0.0
            self.positions[key] = (x, y, z)
            self.keys_order.append(key)

            # register track if needed for python plotting
            letter, _ = self._parse_key(key)
            if letter is not None and letter not in self._track_map:
                order = len(self._track_map)
                self._track_order[letter] = order
                self._track_map[letter] = -order * self.py_track_spacing

    def _pose_cb(self, msg: PoseFactorStamped):
        k1, k2 = msg.key1, msg.key2
        # Add keys
        self._ensure_key(k1)
        self._ensure_key(k2)
        # Only add edge if it's a between-measurement (not a prior)
        if k1 != k2:
            with self._lock:
                self.odom_edges.add(self._normalize_edge(k1, k2))

    def _range_cb(self, msg: RangeFactorStamped):
        k1, k2 = msg.key1, msg.key2
        # if keys have not been seen yet, print a warning that range seen before odom
        if k1 not in self.positions:
            rospy.logwarn(f"[graph_viz] Range measurement received for unknown key '{k1}' before any pose measurements.")
        if k2 not in self.positions:
            rospy.logwarn(f"[graph_viz] Range measurement received for unknown key '{k2}' before any pose measurements.")

        self._ensure_key(k1)
        self._ensure_key(k2)
        if k1 != k2:
            with self._lock:
                self.range_edges.add(self._normalize_edge(k1, k2))

    def _make_node_marker(self, idx: int, key: str, pos: Tuple[float, float, float]) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = f"{self.agent}_nodes"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.r = 0.1
        m.color.g = 0.4
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)
        return m

    def _make_text_marker(self, idx: int, key: str, pos: Tuple[float, float, float]) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = f"{self.agent}_labels"
        m.id = 10000 + idx
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2] + 0.35
        m.scale.z = 0.25
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.9
        m.text = key
        m.lifetime = rospy.Duration(0)
        return m

    def _make_edges_marker(self, edges: Set[Tuple[str, str]], marker_id: int, color: Tuple[float, float, float], name: str="") -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = f"{self.agent}_edges{name}"
        m.id = marker_id
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)
        for a, b in sorted(edges):
            pa = Point()
            pb = Point()
            ax, ay, az = self.positions[a]
            bx, by, bz = self.positions[b]
            pa.x, pa.y, pa.z = ax, ay, az
            pb.x, pb.y, pb.z = bx, by, bz
            m.points.append(pa)
            m.points.append(pb)
        return m

    def _publish(self, _event):
        if self.pub.get_num_connections() == 0:
            return
        with self._lock:
            arr = MarkerArray()
            # Nodes and labels
            for idx, key in enumerate(self.keys_order):
                pos = self.positions[key]
                arr.markers.append(self._make_node_marker(idx, key, pos))
                arr.markers.append(self._make_text_marker(idx, key, pos))
            # Edges: pose (odom) in orange, range in green
            arr.markers.append(self._make_edges_marker(self.odom_edges, 20000, (1.0, 0.6, 0.0), "odom"))
            arr.markers.append(self._make_edges_marker(self.range_edges, 20001, (0.0, 1.0, 0.2), "range"))
        self.pub.publish(arr)

    # ===== Native Python plotting utilities =====
    def _parse_key(self, key: str) -> Tuple[Optional[str], Optional[int]]:
        assert key[0].isupper(), f"Cannot parse key '{key}' because it does not start with an uppercase letter"
        assert key[1:].isdigit(), f"Cannot parse key '{key}' because the part after the first character is not all digits"
        letter, idx = key[0], int(key[1:])
        return letter, idx

    def _py_pos(self, key: str) -> Optional[Tuple[float, float]]:
        letter, idx = self._parse_key(key)
        # print(f"[graph_viz] Parsed key '{key}': letter='{letter}', idx={idx}")
        if letter is None or idx is None:
            return None
        # y is track for this letter, x is scaled index
        if letter not in self._track_map:
            # print(f"[graph_viz] Creating track for letter '{letter}'")
            # ensure track exists
            with self._lock:
                # print(f"[graph_viz] Acquired lock to create track for letter '{letter}'")
                if letter not in self._track_map:
                    order = len(self._track_map)
                    self._track_order[letter] = order
                    self._track_map[letter] = -order * self.py_track_spacing
        # print(f"[graph_viz] Retrieved track for letter '{letter}'")
        y = self._track_map[letter]
        x = idx * self.py_index_scale
        if letter == "L":
            y *= 3
            x += (idx + 1) * 10
        return x, y

    def _update_py_plot(self, _event):
        if not self._ax or not self._plt:
            return
        self._ax.clear()
        # snapshot under lock
        with self._lock:
            keys = list(self.keys_order)
            odom_edges = list(self.odom_edges)
            range_edges = list(self.range_edges)

        # draw tracks as faint lines
        for letter, order in sorted(self._track_order.items(), key=lambda kv: kv[1]):
            y = -order * self.py_track_spacing
            self._ax.plot([0, max(1, 1 + len(keys)) * self.py_index_scale], [y, y], color=(0.8, 0.8, 0.8), linewidth=0.5)
            self._ax.text(-0.8 * self.py_index_scale, y, letter, va='center', ha='right', fontsize=9, color=(0.6, 0.6, 0.6))

        # nodes
        xs, ys, labels = [], [], []
        for k in keys:
            pos = self._py_pos(k)
            if pos is None:
                continue
            x, y = pos
            xs.append(x)
            ys.append(y)
            labels.append(k)
        if xs:
            self._ax.scatter(xs, ys, s=40, c=[(0.1, 0.4, 1.0)], zorder=3)
            for x, y, lab in zip(xs, ys, labels):
                self._ax.text(x + 0.1 * self.py_index_scale, y + 0.05 * self.py_track_spacing, lab, fontsize=8)

        # edges
        def _draw_edges(edges, color):
            for a, b in edges:
                pa = self._py_pos(a)
                pb = self._py_pos(b)
                if pa is None or pb is None:
                    continue
                (x1, y1), (x2, y2) = pa, pb
                self._ax.plot([x1, x2], [y1, y2], color=color, linewidth=1.5, zorder=2)

        _draw_edges(odom_edges, (1.0, 0.6, 0.0))
        _draw_edges(range_edges, (0.0, 1.0, 0.2))

        self._ax.set_title(f"Measurement Graph — {self.agent}")
        self._ax.set_xlabel("Index")
        self._ax.set_ylabel("Track (by leading letter)")
        if self.py_aspect_equal:
            self._ax.set_aspect('equal', adjustable='datalim')
        self._ax.grid(True, which='both', axis='both', alpha=0.2)
        self._plt.tight_layout()
        self._plt.pause(0.001)


def main():
    rospy.init_node("measurement_graph_viz")
    viz = MeasurementGraphViz()
    rospy.loginfo("measurement_graph_viz started")
    rospy.spin()


if __name__ == "__main__":
    main()
