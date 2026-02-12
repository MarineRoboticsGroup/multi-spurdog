#!/usr/bin/env python3
"""
Mission data processor for Fall 2025 single agent missions.

Collects data from GTSAM and CORA estimators, GPS fixes, and generates comparison plots.
Plots include:
- Initial poses
- Odometry trajectory
- GTSAM optimized trajectory
- CORA optimized trajectory
- Surfacing position (first/last GPS fix)
- Landmark positions
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import os
from collections import defaultdict
from typing import Dict, List, Tuple, Optional
from scipy.spatial.transform import Rotation

from spurdog_acomms.msg import PoseFactorStamped, RangeFactorStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import NavSatFix

# Simple Key class to avoid dependency on estimator module
class Key:
    def __init__(self, char: str, index: int):
        self.char = char
        self.index = index
    
    def __repr__(self):
        return f"{self.char}{self.index}"
    
    def __hash__(self):
        return hash((self.char, self.index))
    
    def __eq__(self, other):
        return self.char == other.char and self.index == other.index


class MissionProcessor:
    """Processes mission data and generates comparison plots."""
    
    def __init__(self):
        rospy.init_node('mission_processor', anonymous=True)
        
        # Parameters
        self.mission_name = rospy.get_param("~mission_name", "unknown")
        self.output_dir = rospy.get_param("~output_dir", "/tmp/mission_results")
        self.dimension = rospy.get_param("~dimension", 3)
        
        # Optional surface timestamp for filtering dive data only
        # If set, will only process data BEFORE this timestamp
        self.surface_timestamp = rospy.get_param("~surface_timestamp", None)
        if self.surface_timestamp:
            rospy.loginfo(f"Filtering data to before surface time: {self.surface_timestamp}")
        
        # Landmark positions
        self.landmarks = rospy.get_param("/landmarks", {})
        
        # Data storage
        self.odometry_poses = []  # List of (timestamp, x, y, z, qx, qy, qz, qw)
        self.gtsam_poses = {}     # Dict[Key, (x, y, z)]
        self.cora_poses = {}      # Dict[Key, (x, y, z)]
        self.gps_fixes = []       # List of (timestamp, lat, lon, alt)
        self.range_measurements = []  # List of (timestamp, key1, key2, range)
        
        # First/last pose keys for tracking
        self.first_pose_key = None
        self.last_pose_key = None
        
        # Track last message time to detect when bag finishes
        self.last_message_time = rospy.Time.now()
        self.bag_finished = False
        self.final_plots_generated = False
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        rospy.loginfo(f"Mission processor initialized for {self.mission_name}")
        rospy.loginfo(f"Output directory: {self.output_dir}")
        
        # Subscribers
        self.pose_factor_sub = rospy.Subscriber(
            "/actor_0/pose_factor", PoseFactorStamped, self.pose_factor_callback, queue_size=100
        )
        self.range_factor_sub = rospy.Subscriber(
            "/actor_0/range_factor", RangeFactorStamped, self.range_factor_callback, queue_size=100
        )
        # Subscribe to integrated_state for absolute position trajectory
        self.integrated_state_sub = rospy.Subscriber(
            "/actor_0/integrated_state", PoseWithCovarianceStamped, self.integrated_state_callback, queue_size=200
        )
        self.gps_sub = rospy.Subscriber(
            "/actor_0/gps", PoseWithCovarianceStamped, self.gps_callback, queue_size=10
        )
        self.navsat_sub = rospy.Subscriber(
            "/actor_0/navsat", NavSatFix, self.navsat_callback, queue_size=10
        )
        
        # Subscribe to GTSAM estimator output
        self.gtsam_estimate_sub = rospy.Subscriber(
            "/gtsam_estimator/pose", PoseFactorStamped, self.gtsam_estimate_callback, queue_size=100
        )
        rospy.loginfo(f"[mission_processor] Subscribed to GTSAM poses on: {self.gtsam_estimate_sub.resolved_name}")
        
        # Subscribe to CORA estimator output
        self.cora_estimate_sub = rospy.Subscriber(
            "/cora_estimator/pose", PoseFactorStamped, self.cora_estimate_callback, queue_size=100
        )
        rospy.loginfo(f"[mission_processor] Subscribed to CORA poses on: {self.cora_estimate_sub.resolved_name}")
        
        # Timer for periodic plotting
        self.plot_timer = rospy.Timer(rospy.Duration(5.0), self.plot_timer_callback)
        
        # Shutdown hook to generate final plots
        rospy.on_shutdown(self.shutdown_callback)
        
        rospy.loginfo("Mission processor ready")
    
    def pose_factor_callback(self, msg: PoseFactorStamped):
        """Collect odometry pose factors (these are relative poses, not used for trajectory)."""
        timestamp = msg.header.stamp.to_sec()
        
        # Filter by surface timestamp if set
        if self.surface_timestamp and timestamp >= self.surface_timestamp:
            rospy.logdebug(f"Filtered pose factor after surface time: {timestamp}")
            return
        
        # Track first and last pose keys
        key1 = Key(msg.key1[0], int(msg.key1[1:]))
        key2 = Key(msg.key2[0], int(msg.key2[1:]))
        
        if self.first_pose_key is None:
            self.first_pose_key = key1
        self.last_pose_key = key2
        
        rospy.logdebug(f"Odometry factor: {msg.key1} -> {msg.key2}")
    
    def range_factor_callback(self, msg: RangeFactorStamped):
        """Collect range measurements."""
        self.last_message_time = rospy.Time.now()  # Track bag messages
        timestamp = msg.header.stamp.to_sec()
        
        # Filter by surface timestamp if set
        if self.surface_timestamp and timestamp >= self.surface_timestamp:
            rospy.logdebug(f"Filtered range factor after surface time: {timestamp}")
            return
        
        self.range_measurements.append((timestamp, msg.key1, msg.key2, msg.meas_range))
        rospy.logdebug(f"Range: {msg.key1} <-> {msg.key2}: {msg.meas_range:.2f}m")
    
    def integrated_state_callback(self, msg: PoseWithCovarianceStamped):
        """Collect integrated state for absolute position trajectory."""
        self.last_message_time = rospy.Time.now()  # Track when we receive messages
        
        timestamp = msg.header.stamp.to_sec()
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Filter by surface timestamp if set
        if self.surface_timestamp and timestamp >= self.surface_timestamp:
            rospy.logdebug(f"Filtered odometry after surface time: {timestamp}")
            return
        
        # Store absolute positions for trajectory plotting
        self.odometry_poses.append((timestamp, position.x, position.y, position.z,
                                   orientation.x, orientation.y, orientation.z, orientation.w))
        
        # Set first orientation as ROS parameter for estimators to use
        # This ensures optimizers work in a world-aligned frame instead of starting at identity rotation
        if len(self.odometry_poses) == 1:
            rospy.set_param('/estimator/first_pose_orientation', 
                           [orientation.x, orientation.y, orientation.z, orientation.w])
            rospy.loginfo(f"Set first pose orientation parameter: quat=({orientation.x:.4f}, {orientation.y:.4f}, {orientation.z:.4f}, {orientation.w:.4f})")
        
        rospy.logdebug(f"Integrated state: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})")
    
    def gps_callback(self, msg: PoseWithCovarianceStamped):
        """Collect GPS fixes (as PoseWithCovarianceStamped)."""
        self.last_message_time = rospy.Time.now()  # Track bag messages
        timestamp = msg.header.stamp.to_sec()
        position = msg.pose.pose.position
        self.gps_fixes.append((timestamp, position.x, position.y, position.z))
        rospy.logdebug(f"GPS fix: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})")
        
        # Set first GPS fix as initial position parameter for estimators
        if len(self.gps_fixes) == 1:
            rospy.set_param('/estimator/first_pose_position', 
                          [position.x, position.y, position.z])
            rospy.loginfo(f"Set first GPS fix as initial position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f})")
    
    def navsat_callback(self, msg: NavSatFix):
        """Collect GPS fixes (as NavSatFix)."""
        self.last_message_time = rospy.Time.now()  # Track bag messages
        # Store as (timestamp, lat, lon, alt) - will need to convert to local frame
        timestamp = msg.header.stamp.to_sec()
        self.gps_fixes.append((timestamp, msg.latitude, msg.longitude, msg.altitude))
        rospy.logdebug(f"NavSat fix: ({msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.2f})")
    
    def gtsam_estimate_callback(self, msg: PoseFactorStamped):
        """Collect GTSAM optimized poses."""
        # Don't update last_message_time here - only track bag messages (integrated_state, gps, ranges)
        # Otherwise estimator incremental updates prevent bag completion detection
        
        rospy.loginfo(f"[CALLBACK] GTSAM callback received for key: {msg.key2}")
        try:
            # Extract key and position (just use the string directly as the key)
            key_str = msg.key2  # e.g., "A0"
            pose = msg.pose.pose
            position = (pose.position.x, pose.position.y, pose.position.z)
            self.gtsam_poses[key_str] = position
            rospy.loginfo(f"[CALLBACK] GTSAM pose added: {key_str} at {position}")
        except Exception as e:
            rospy.logerr(f"Error in gtsam_estimate_callback: {e}")
    
    def cora_estimate_callback(self, msg: PoseFactorStamped):
        """Collect CORA optimized poses."""
        # Don't update last_message_time here - only track bag messages (integrated_state, gps, ranges)
        # Otherwise estimator incremental updates prevent bag completion detection
        
        rospy.loginfo(f"[CALLBACK] CORA callback received for key: {msg.key2}")
        try:
            # Extract key and position (just use the string directly as the key)
            key_str = msg.key2  # e.g., "A0"
            pose = msg.pose.pose
            position = (pose.position.x, pose.position.y, pose.position.z)
            self.cora_poses[key_str] = position
            rospy.loginfo(f"[CALLBACK] CORA pose added: {key_str} at {position}")
        except Exception as e:
            rospy.logerr(f"Error in cora_estimate_callback: {e}")
    
    def plot_timer_callback(self, event):
        """Periodic plotting to monitor progress and detect bag completion."""
        if len(self.odometry_poses) > 0:
            rospy.loginfo_throttle(
                30.0, 
                f"Data collected: {len(self.odometry_poses)} odometry poses, "
                f"{len(self.gtsam_poses)} GTSAM poses, {len(self.cora_poses)} CORA poses, "
                f"{len(self.gps_fixes)} GPS fixes, {len(self.range_measurements)} ranges"
            )
        
        # Check if bag has finished (no messages for 5 seconds)
        if not self.bag_finished:
            time_since_last_msg = (rospy.Time.now() - self.last_message_time).to_sec()
            if time_since_last_msg > 5.0 and len(self.odometry_poses) > 0:
                rospy.loginfo(f"No messages received for {time_since_last_msg:.1f}s - bag appears finished")
                self.bag_finished = True
                
                # Estimators need time to finish incremental updates, so wait before triggering shutdown
                # This allows them to process any remaining messages and optimize
                rospy.loginfo("Waiting 10 seconds for estimators to finish incremental processing...")
                rospy.sleep(10.0)
                
                # Now trigger shutdown which will run estimator shutdown hooks (final optimization)
                rospy.loginfo("Triggering shutdown to run final batch optimizations...")
                # Send SIGTERM to estimator nodes to trigger their shutdown hooks
                import subprocess
                subprocess.call(["rosnode", "kill", "/gtsam_estimator"])
                subprocess.call(["rosnode", "kill", "/cora_estimator"])
                
                # Wait for estimators to complete shutdown hooks and publish all poses
                # GTSAM is fast (~1s), CORA can take 10-15 seconds for full optimization
                rospy.loginfo("Waiting up to 60 seconds for final batch optimization and publishing...")
                wait_start = rospy.Time.now()
                max_wait = 60.0
                expected_poses = len(self.odometry_poses) - 1
                rate = rospy.Rate(2)  # Check twice per second
                
                while (rospy.Time.now() - wait_start).to_sec() < max_wait and not rospy.is_shutdown():
                    gtsam_count = len(self.gtsam_poses)
                    cora_count = len(self.cora_poses)
                    
                    rospy.loginfo_throttle(5.0, f"Poses collected: {gtsam_count} GTSAM, {cora_count} CORA (expecting ~{expected_poses})")
                    
                    # If we have close to expected number of poses, break early
                    if gtsam_count >= expected_poses * 0.9 and cora_count >= expected_poses * 0.9:
                        rospy.loginfo(f"Received sufficient poses: {gtsam_count} GTSAM, {cora_count} CORA")
                        break
                    
                    rate.sleep()
                
                # Final brief wait to catch any last messages
                rospy.sleep(2.0)
                
                if not self.final_plots_generated:
                    rospy.loginfo(f"Generating final plots with {len(self.gtsam_poses)} GTSAM, {len(self.cora_poses)} CORA poses...")
                    self.generate_plots()
                    self.final_plots_generated = True
                    rospy.loginfo("Mission processor complete - shutting down")
                    rospy.signal_shutdown("Mission processing complete")
    
    def generate_plots(self):
        """Generate comparison plots of all trajectories."""
        rospy.loginfo("Generating comparison plots...")
        
        # Create figure with single XY plot
        fig = plt.figure(figsize=(12, 10))
        
        # 2D XY plot only
        ax_xy = fig.add_subplot(1, 1, 1)
        self.plot_2d_trajectory(ax_xy, 'x', 'y', 'XY Plane - Mission Trajectory')
        
        # Add title
        fig.suptitle(f'Mission: {self.mission_name.upper()}', fontsize=16, fontweight='bold')
        
        # Adjust layout
        plt.tight_layout()
        
        # Save plot
        output_file = os.path.join(self.output_dir, f'{self.mission_name}_comparison.png')
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        rospy.loginfo(f"Plot saved to: {output_file}")
        
        # Also save a CSV of the data
        self.save_data_to_csv()
        
        # Show plot (optional, comment out for headless operation)
        # plt.show()
    
    def plot_2d_trajectory(self, ax, x_axis: str, y_axis: str, title: str):
        """Plot 2D trajectory on specified axes - ALL DATA IN WORLD FRAME."""
        # Map axis names to indices
        axis_map = {'x': 1, 'y': 2, 'z': 3}
        x_idx = axis_map[x_axis]
        y_idx = axis_map[y_axis]
        
        # Plot odometry trajectory  
        if self.odometry_poses:
            odom_data = np.array(self.odometry_poses)
            
            # All data (odometry, GTSAM, CORA, GPS, landmarks) in WORLD FRAME - NO transformations
            odom_x = odom_data[:, x_idx]
            odom_y = odom_data[:, y_idx]
            
            rospy.logwarn(f"Odometry X range: {odom_x.max() - odom_x.min():.2f}m, "
                         f"Y range: {odom_y.max() - odom_y.min():.2f}m, "
                         f"Z range: {odom_data[:, 3].max() - odom_data[:, 3].min():.2f}m")
            
            ax.plot(odom_x, odom_y, 'k-', linewidth=1.5, alpha=0.7, label='Odometry')

        
        # Plot GTSAM trajectory
        if self.gtsam_poses:
            # Keys are strings like "A0", "A1", etc. - sort by numeric index
            gtsam_positions = sorted([(int(k[1:]), v) for k, v in self.gtsam_poses.items()])
            gtsam_array = np.array([pos for _, pos in gtsam_positions])
            # GTSAM poses in world frame - no transformations
            gtsam_x = gtsam_array[:, 0]
            gtsam_y = gtsam_array[:, 1]
            rospy.logwarn(f"GTSAM range: X=[{gtsam_x.min():.2f}, {gtsam_x.max():.2f}], "
                         f"Y=[{gtsam_y.min():.2f}, {gtsam_y.max():.2f}], "
                         f"Z=[{gtsam_array[:, 2].min():.2f}, {gtsam_array[:, 2].max():.2f}]")
            ax.plot(gtsam_x, gtsam_y, 
                   'b-o', linewidth=2, markersize=4, alpha=0.6, label='GTSAM')
        
        # Plot CORA trajectory
        if self.cora_poses:
            # Keys are strings like "A0", "A1", etc. - sort by numeric index
            cora_positions = sorted([(int(k[1:]), v) for k, v in self.cora_poses.items()])
            cora_array = np.array([pos for _, pos in cora_positions])
            # CORA poses in world frame - no transformations
            cora_x = cora_array[:, 0]
            cora_y = cora_array[:, 1]
            rospy.logwarn(f"CORA range: X=[{cora_x.min():.2f}, {cora_x.max():.2f}], "
                         f"Y=[{cora_y.min():.2f}, {cora_y.max():.2f}], "
                         f"Z=[{cora_array[:, 2].min():.2f}, {cora_array[:, 2].max():.2f}]")
            ax.plot(cora_x, cora_y, 
                   'r-s', linewidth=2, markersize=4, alpha=0.6, label='CORA')
        
        # Plot surfacing position (last GPS fix) - NO transformation
        if self.gps_fixes:
            gps_array = np.array(self.gps_fixes)
            # Last GPS (surface) in world frame
            if len(gps_array) > 0:
                gps_x = gps_array[-1, x_idx]
                gps_y = gps_array[-1, y_idx]
                ax.plot(gps_x, gps_y, 
                       'mv', markersize=12, label='Surfacing Position', zorder=10)
        
        # Plot landmarks in world frame - NO transformation
        for name, position in self.landmarks.items():
            x_pos = position[0] if x_axis == 'x' else (position[1] if x_axis == 'y' else position[2])
            y_pos = position[0] if y_axis == 'x' else (position[1] if y_axis == 'y' else position[2])
            ax.plot(x_pos, y_pos, 'g*', markersize=15, label=f'Landmark {name}')
            # Add circle to show approximate range uncertainty
            circle = Circle((x_pos, y_pos), 2.0, fill=False, edgecolor='g', 
                          linestyle='--', alpha=0.3)
            ax.add_patch(circle)
        
        # Labels and formatting
        ax.set_xlabel(f'{x_axis.upper()} (m)', fontsize=12)
        ax.set_ylabel(f'{y_axis.upper()} (m)', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
    
    def plot_3d_trajectory(self, ax):
        """Plot 3D trajectory."""
        # Plot odometry
        if self.odometry_poses:
            odom_data = np.array(self.odometry_poses)
            ax.plot(odom_data[:, 1], odom_data[:, 2], odom_data[:, 3], 
                   'k-', linewidth=1, alpha=0.5, label='Odometry')
            ax.scatter(odom_data[0, 1], odom_data[0, 2], odom_data[0, 3], 
                      c='g', s=100, marker='o', label='Start')
        
        # Plot GTSAM
        if self.gtsam_poses:
            gtsam_positions = sorted([(k.index, v) for k, v in self.gtsam_poses.items()])
            gtsam_array = np.array([pos for _, pos in gtsam_positions])
            ax.plot(gtsam_array[:, 0], gtsam_array[:, 1], gtsam_array[:, 2], 
                   'b-o', linewidth=2, markersize=4, label='GTSAM')
        
        # Plot CORA
        if self.cora_poses:
            cora_positions = sorted([(k.index, v) for k, v in self.cora_poses.items()])
            cora_array = np.array([pos for _, pos in cora_positions])
            ax.plot(cora_array[:, 0], cora_array[:, 1], cora_array[:, 2], 
                   'r-s', linewidth=2, markersize=4, label='CORA')
        
        # Plot GPS fixes
        if self.gps_fixes:
            gps_array = np.array(self.gps_fixes)
            ax.scatter(gps_array[0, 1], gps_array[0, 2], gps_array[0, 3], 
                      c='c', s=150, marker='^', label='GPS Start')
            if len(gps_array) > 1:
                ax.scatter(gps_array[-1, 1], gps_array[-1, 2], gps_array[-1, 3], 
                          c='m', s=150, marker='v', label='GPS End')
        
        # Plot landmarks
        for name, position in self.landmarks.items():
            ax.scatter(position[0], position[1], position[2], 
                      c='r', s=200, marker='*', label=f'Landmark {name}')
        
        # Labels
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_zlabel('Z (m)', fontsize=12)
        ax.set_title('3D Trajectory', fontsize=14)
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
    
    def save_data_to_csv(self):
        """Save collected data to CSV files in WORLD FRAME (no transformation)."""
        import csv
        
        rospy.loginfo("Saving all data in WORLD FRAME (no coordinate transformation)")
        
        # Save odometry poses in world frame (no transformation)
        if self.odometry_poses:
            odom_file = os.path.join(self.output_dir, f'{self.mission_name}_odometry.csv')
            with open(odom_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                for pose in self.odometry_poses:
                    timestamp = pose[0]
                    world_pos = np.array([pose[1], pose[2], pose[3]])
                    world_quat = np.array([pose[4], pose[5], pose[6], pose[7]])
                    
                    # Save in world frame (no transformation)
                    writer.writerow([timestamp, world_pos[0], world_pos[1], world_pos[2],
                                   world_quat[0], world_quat[1], world_quat[2], world_quat[3]])
            rospy.loginfo(f"Odometry data saved to: {odom_file}")
        
        # Save GTSAM poses in world frame (no transformation)
        if self.gtsam_poses:
            gtsam_file = os.path.join(self.output_dir, f'{self.mission_name}_gtsam.csv')
            with open(gtsam_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['key_char', 'key_index', 'x', 'y', 'z'])
                # Keys are now strings like "A0", "A1", "L0", etc.
                for key_str, pos in sorted(self.gtsam_poses.items(), key=lambda x: (x[0][0], int(x[0][1:]))):
                    key_char = key_str[0]
                    key_index = int(key_str[1:])
                    
                    # Save in world frame (no transformation)
                    writer.writerow([key_char, key_index, pos[0], pos[1], pos[2]])
            rospy.loginfo(f"GTSAM data saved to: {gtsam_file}")
        
        # Save CORA poses in world frame (no transformation)
        if self.cora_poses:
            cora_file = os.path.join(self.output_dir, f'{self.mission_name}_cora.csv')
            with open(cora_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['key_char', 'key_index', 'x', 'y', 'z'])
                # Keys are now strings like "A0", "A1", "L0", etc.
                for key_str, pos in sorted(self.cora_poses.items(), key=lambda x: (x[0][0], int(x[0][1:]))):
                    key_char = key_str[0]
                    key_index = int(key_str[1:])
                    
                    # Save in world frame (no transformation)
                    writer.writerow([key_char, key_index, pos[0], pos[1], pos[2]])
            rospy.loginfo(f"CORA data saved to: {cora_file}")
        
        # Save GPS fixes in world frame (no transformation)
        if self.gps_fixes:
            gps_file = os.path.join(self.output_dir, f'{self.mission_name}_gps.csv')
            with open(gps_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x_or_lat', 'y_or_lon', 'z_or_alt'])
                for fix in self.gps_fixes:
                    timestamp = fix[0]
                    world_pos = np.array([fix[1], fix[2], fix[3]])
                    
                    # Save in world frame (no transformation)
                    writer.writerow([timestamp, world_pos[0], world_pos[1], world_pos[2]])
            rospy.loginfo(f"GPS data saved to: {gps_file}")
        
        # Save range measurements
        if self.range_measurements:
            range_file = os.path.join(self.output_dir, f'{self.mission_name}_ranges.csv')
            with open(range_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'key1', 'key2', 'range'])
                writer.writerows(self.range_measurements)
            rospy.loginfo(f"Range data saved to: {range_file}")
    
    def shutdown_callback(self):
        """Generate final plots on shutdown."""
        rospy.loginfo("Shutting down - waiting for final estimator optimizations...")
        # Keep collecting data for 30 more seconds to catch final optimization results
        # CORA can take 10-15 seconds for final batch optimization
        end_time = rospy.Time.now() + rospy.Duration(30.0)
        rate = rospy.Rate(10)  # 10 Hz
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("Generating final plots...")
        self.generate_plots()
        rospy.loginfo("Mission processor shutdown complete")


def main():
    try:
        processor = MissionProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
