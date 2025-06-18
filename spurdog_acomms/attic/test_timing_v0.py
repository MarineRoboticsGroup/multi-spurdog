#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from spurdog_acomms.srv import (
    PreintegrateImu,
    PreintegrateImuRequest,
    PreintegrateImuResponse)
from spurdog_acomms.msg import (
    PoseFactorStamped,
    RangeFactorStamped)

class ImuDebug:
    """This is a node to run the Imu Debug for the vehicle.
    It will handle the ping requests, responses, and the range data collection for a ping-only cycle.
    It is untested on ping + data cycles, but should work with minimal changes."""
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.landmarks = {"L0":[-74.5193539608157,-38.9298973079931,1.5], "L1":[66.5150726324041,25.969767675496275,1.5]} # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = float(rospy.get_param("sound_speed", 1486))

        # Check services
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nav_state = rospy.Subscriber("nav_state", PoseStamped, self.on_nav_state) # GPS messages are also NMEA messages
        self.gps = rospy.Subscriber("gps", PoseWithCovarianceStamped, self.on_gps) # GPS messages are also NMEA messages
        self.cv7 = rospy.Subscriber("cv7_ahrs", Imu, self.on_cv7) # CV7 messages are also IMU messages
        self.navigator = rospy.Subscriber("navigator_ahrs", Imu, self.on_navigator) # Navigator messages are also IMU messages
        self.txf = rospy.Subscriber("modem/txf", Header, self.on_txf) # Range factor messages are also IMU messages
        self.modem_time = None
        self.cv7_time = None
        self.navigator_time = None
        self.gps_time = None
        self.nav_state_time = None
        # log the time deltas
        self.time_deltas = []
        self.timestamps= []
        self.cv7_data = {
            "timestamp": [],
            "roll": [],
            "pitch": [],
            "yaw": [],
            "qx": [],
            "qy": [],
            "qz": [],
            "qw": [],
            "ax": [],
            "ay": [],
            "az": [],
            "gx": [],
            "gy": [],
            "gz": [],
        }
        self.navigator_data = {
            "timestamp": [],
            "roll": [],
            "pitch": [],
            "yaw": [],
            "qx": [],
            "qy": [],
            "qz": [],
            "qw": [],
            "ax": [],
            "ay": [],
            "az": [],
            "gx": [],
            "gy": [],
            "gz": [],
        }
        self.dead_reckon = {
            "timestamp": [],
            "x": [],
            "y": [],
            "z": [],
            "roll": [],
            "pitch": [],
            "yaw": [],
        }
        self.navigator_offset = 0
        # Start the cycle
        rospy.loginfo("[%s] Extractor Node Configured" % rospy.Time.now())
        rospy.sleep(1) # allow for modem to configure
    def convert_quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        quaternion = [qx, qy, qz, qw]
        euler = spt.Rotation.from_quat(quaternion).as_euler('xyz', degrees=True)
        return euler[0], euler[1], euler[2]
    def on_nav_state(self, msg):
        """Callback for the nav_state topic."""
        #rospy.loginfo("[%s] Nav State: %s" % (rospy.Time.now(), msg))
        self.nav_state_time = msg.header.stamp
        #self.compare_times()
        # Here you can add logic to handle the nav state, e.g., trigger preintegration
        # Store the nav state data for later use
        self.dead_reckon["timestamp"].append(msg.header.stamp.to_sec())
        self.dead_reckon["x"].append(msg.pose.position.x)
        self.dead_reckon["y"].append(msg.pose.position.y)
        self.dead_reckon["z"].append(msg.pose.position.z)
        roll, pitch, yaw = self.convert_quaternion_to_euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.dead_reckon["roll"].append(roll)
        self.dead_reckon["pitch"].append(pitch)
        self.dead_reckon["yaw"].append(yaw)
    def on_gps(self, msg):
        """Callback for the gps topic."""
        #rospy.loginfo("[%s] GPS: %s" % (rospy.Time.now(), msg))
        self.gps_time = msg.header.stamp
        #self.compare_times()
        # Here you can add logic to handle the GPS data, e.g., trigger preintegration
    def on_cv7(self, msg):
        """Callback for the cv7_ahrs topic."""
        #rospy.loginfo("[%s] CV7 AHRS: %s" % (rospy.Time.now(), msg))
        self.cv7_time = msg.header.stamp
        #self.compare_times()
        # Here you can add logic to handle the CV7 AHRS data, e.g., trigger preintegration
        # Store the CV7 data for later use
        self.cv7_data["timestamp"].append(msg.header.stamp.to_sec())
        roll, pitch, yaw = self.convert_quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.cv7_data["roll"].append(roll)
        self.cv7_data["pitch"].append(pitch)
        self.cv7_data["yaw"].append(yaw)
        self.cv7_data["qx"].append(msg.orientation.x)
        self.cv7_data["qy"].append(msg.orientation.y)
        self.cv7_data["qz"].append(msg.orientation.z)
        self.cv7_data["qw"].append(msg.orientation.w)
        self.cv7_data["ax"].append(msg.linear_acceleration.x)
        self.cv7_data["ay"].append(msg.linear_acceleration.y)
        self.cv7_data["az"].append(msg.linear_acceleration.z)
        self.cv7_data["gx"].append(msg.angular_velocity.x)
        self.cv7_data["gy"].append(msg.angular_velocity.y)
        self.cv7_data["gz"].append(msg.angular_velocity.z)
    def on_navigator(self, msg):
        """Callback for the navigator_ahrs topic."""
        #rospy.loginfo("[%s] Navigator AHRS: %s" % (rospy.Time.now(), msg))
        self.navigator_time = msg.header.stamp - rospy.Duration(self.navigator_offset)
        #self.compare_times()
        # Here you can add logic to handle the Navigator AHRS data, e.g., trigger preintegration
        # Store the Navigator data for later use
        self.navigator_data["timestamp"].append(msg.header.stamp.to_sec())
        roll, pitch, yaw = self.convert_quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.navigator_data["roll"].append(roll)
        self.navigator_data["pitch"].append(pitch)
        self.navigator_data["yaw"].append(yaw)
        self.navigator_data["qx"].append(msg.orientation.x)
        self.navigator_data["qy"].append(msg.orientation.y)
        self.navigator_data["qz"].append(msg.orientation.z)
        self.navigator_data["qw"].append(msg.orientation.w)
        self.navigator_data["ax"].append(msg.linear_acceleration.x)
        self.navigator_data["ay"].append(msg.linear_acceleration.y)
        self.navigator_data["az"].append(msg.linear_acceleration.z)
        self.navigator_data["gx"].append(msg.angular_velocity.x)
        self.navigator_data["gy"].append(msg.angular_velocity.y)
        self.navigator_data["gz"].append(msg.angular_velocity.z)
    def on_txf(self, msg):
        """Callback for the modem txf topic."""
        #rospy.loginfo("[%s] Modem TXF: %s" % (rospy.Time.now(), msg))
        self.modem_time = msg.stamp
        #self.compare_times()
    def compare_times(self):
        """Compare the timestamps of the different messages."""
        # Log the times in seconds of the different messages
        rospy.loginfo("[%s] Comparing times - Modem: %s, CV7: %s, Navigator: %s, GPS: %s, Nav State: %s" % (
            rospy.Time.now(),
            self.modem_time,
            self.cv7_time,
            self.navigator_time,
            self.gps_time,
            self.nav_state_time
        ))
        # Log the timestamps in seconds
        self.timestamps.append({
            'node_time': rospy.Time.now().to_sec(),
            'modem_time': self.modem_time.to_sec() if self.modem_time is not None else None,
            'cv7_time': self.cv7_time.to_sec() if self.cv7_time is not None else None,
            'navigator_time': self.navigator_time.to_sec() if self.navigator_time is not None else None,
            'gps_time': self.gps_time.to_sec() if self.gps_time is not None else None,
            'nav_state_time': self.nav_state_time.to_sec() if self.nav_state_time is not None else None
        })
        # Use the navigator time as the reference time (it is a ros time)
        reference_time = rospy.Time(0, 0)  # Default to zero time if no navigator time is available
        # Convert theference time to a duration
        reference_time = rospy.Duration(reference_time.secs, reference_time.nsecs).to_sec()
        # Adjust each time as a delta from the reference time (unless they are None)
        if self.modem_time is not None:
            modem_time = (self.modem_time.secs + self.modem_time.nsecs * 1e-9) - reference_time if self.modem_time is not None else None
        else:
            modem_time = None
        if self.cv7_time is not None:
            cv7_time = (self.cv7_time.secs + self.cv7_time.nsecs * 1e-9) - reference_time if self.cv7_time is not None else None
        else:
            cv7_time = None
        if self.navigator_time is not None:
            navigator_time = (self.navigator_time.secs + self.navigator_time.nsecs * 1e-9) - reference_time if self.navigator_time is not None else None
        else:
            navigator_time = None
        if self.gps_time is not None:
            gps_time = (self.gps_time.secs + self.gps_time.nsecs * 1e-9) - reference_time if self.gps_time is not None else None
        else:
            gps_time = None
        if self.nav_state_time is not None:
            nav_state_time = (self.nav_state_time.secs + self.nav_state_time.nsecs * 1e-9) - reference_time if self.nav_state_time is not None else None
        else:
            nav_state_time = None
        # Log the times
        #rospy.loginfo("[%s] Times - Modem: %s, CV7: %s, Navigator: %s, GPS: %s, Nav State: %s" % (self.nav_state_time, modem_time, cv7_time, navigator_time, gps_time, nav_state_time))
        # Push the times to the time deltas list even if they are None
        self.time_deltas.append({
            'node_time': rospy.Time.now().to_nsec(),
            'modem_time': modem_time,
            'cv7_time': cv7_time,
            'navigator_time': navigator_time,
            'gps_time': gps_time,
            'nav_state_time': nav_state_time
        })
        return
    def plot_time_deltas(self, center_on_mean=True):
        """Plot the time deltas of all sources relative to node time."""
        if not self.time_deltas:
            rospy.logwarn("No time deltas recorded.")
            return

        df = pd.DataFrame(self.time_deltas)

        # Convert node_time to relative seconds from the first sample
        df['node_time'] = (df['node_time'] - df['node_time'].iloc[0]) * 1e-9  # nsec → sec

        # Optionally center each signal on its mean
        if center_on_mean:
            for col in df.columns:
                if col != 'node_time':
                    df[col] -= df[col].mean()

        # Plot all time sources as drift from node_time
        plt.figure(figsize=(12, 6))
        markers = ['o', '^', 's', 'x', 'd']
        colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange', 'tab:purple']

        for i, col in enumerate(df.columns):
            if col == 'node_time':
                continue
            plt.plot(df['node_time'], df[col], label=col.replace("_", " ").title(),
                    marker=markers[i % len(markers)], linestyle='-', alpha=0.7, color=colors[i % len(colors)])

        plt.axhline(0, linestyle='--', color='gray', linewidth=1)
        plt.xlabel("Node Time (s)")
        plt.ylabel("Relative Timestamp Offset (s)")
        plt.title("Sensor Timestamp Drift Relative to Node Time")
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.show()
    def log_times(self, filename=None):
        """Log the time deltas to a CSV file."""
        if not self.timestamps:
            rospy.logwarn("No time deltas recorded.")
            return

        if filename is None:
            filename = join(dirname(abspath(__file__)), "time_deltas.csv")

        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['node_time', 'modem_time', 'cv7_time', 'navigator_time', 'gps_time', 'nav_state_time']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for entry in self.timestamps:
                writer.writerow(entry)

        rospy.loginfo(f"Time deltas logged to {filename}")
        # Save the time deltas to a CSV file
    def plot_timestamp_progress(self):
        """Plot normalized sensor timestamps vs normalized node time to evaluate drift."""
        if not self.time_deltas:
            rospy.logwarn("No time deltas recorded.")
            return

        df = pd.DataFrame(self.time_deltas)

        # Convert node_time from nanoseconds to seconds
        df['node_time'] = df['node_time'] * 1e-9

        # Normalize each column to its first non-null value
        for col in df.columns:
            first_valid = df[col].dropna().iloc[0] if not df[col].dropna().empty else None
            if first_valid is not None:
                df[col] = df[col] - first_valid

        plt.figure(figsize=(12, 6))
        markers = ['o', '^', 's', 'x', 'd']
        colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange', 'tab:purple']

        for i, col in enumerate(df.columns):
            if col == 'node_time':
                continue
            if df[col].isnull().all():
                continue  # skip empty columns
            plt.plot(df['node_time'], df[col], label=f"{col.replace('_', ' ').title()}",
                    marker=markers[i % len(markers)], linestyle='-', alpha=0.7, color=colors[i % len(colors)])

        # Plot the ideal line y = x
        min_x, max_x = df['node_time'].min(), df['node_time'].max()
        plt.plot([min_x, max_x], [min_x, max_x], 'k--', label='Ideal 1:1 Line')

        plt.xlabel("Normalized Node Time (s)")
        plt.ylabel("Normalized Sensor Time (s)")
        plt.title("Relative Sensor Time Progression vs Node Time")
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.tight_layout()
        plt.show()
    def time_hack_navigator(self):
        """Apply a time hack to the navigator data."""
        # Find the difference between the CV7 and Navigator first timestamps
        if not self.cv7_data["timestamp"] or not self.navigator_data["timestamp"]:
            rospy.logwarn("No CV7 or Navigator data available for time hack.")
            return
        cv7_first_time = self.cv7_data["timestamp"][0]
        navigator_first_time = self.navigator_data["timestamp"][0]
        self.navigator_offset = cv7_first_time - navigator_first_time
        rospy.loginfo(f"Navigator time offset applied: {self.navigator_offset} seconds")
        # Apply the offset to all navigator timestamps
        self.navigator_data["timestamp"] = [t + self.navigator_offset for t in self.navigator_data["timestamp"]]
        return
    def plot_cv7_and_navigator_data(self):
        """Plot the CV7 and Navigator data."""
        self.time_hack_navigator()  # Ensure navigator data is aligned with CV7
        # Plot the CV7 data and Navigator data side by side (in a 3x3 grid)
        fig, axs = plt.subplots(3, 3, figsize=(15, 10))
        fig.suptitle("CV7 and Navigator Data", fontsize=16)
        # CV7 data
        axs[0, 0].plot(self.cv7_data["timestamp"], self.cv7_data["roll"], label="Roll", color='blue', linestyle='--')
        axs[0, 0].plot(self.navigator_data["timestamp"], self.navigator_data["roll"], label="Roll", color='orange', linestyle='--')
        axs[0, 0].set_title("CV7 Roll")
        axs[0, 0].set_xlabel("Timestamp (s)")
        axs[0, 0].set_ylabel("Roll (degrees)")
        axs[0, 0].grid(True)
        axs[0, 1].plot(self.cv7_data["timestamp"], self.cv7_data["pitch"], label="Pitch", color='tab:blue', linestyle='--')
        axs[0, 1].plot(self.navigator_data["timestamp"], self.navigator_data["pitch"], label="Pitch", color='tab:orange', linestyle='--')
        axs[0, 1].set_title("CV7 Pitch")
        axs[0, 1].set_xlabel("Timestamp (s)")
        axs[0, 1].set_ylabel("Pitch (degrees)")
        axs[0, 1].grid(True)
        axs[0, 2].plot(self.cv7_data["timestamp"], self.cv7_data["yaw"], label="Yaw", color='tab:blue', linestyle='--')
        axs[0, 2].plot(self.navigator_data["timestamp"], self.navigator_data["yaw"], label="Yaw", color='tab:orange', linestyle='--')
        axs[0, 2].set_title("CV7 Yaw")
        axs[0, 2].set_xlabel("Timestamp (s)")
        axs[0, 2].set_ylabel("Yaw (degrees)")
        axs[0, 2].grid(True)
        axs[1, 0].plot(self.cv7_data["timestamp"], self.cv7_data["ax"], label="Ax", color='tab:blue', linestyle='--')
        axs[1, 0].plot(self.navigator_data["timestamp"], self.navigator_data["ax"], label="Ax", color='tab:orange', linestyle='--')
        axs[1, 0].set_title("CV7 Ax")
        axs[1, 0].set_xlabel("Timestamp (s)")
        axs[1, 0].set_ylabel("Ax (m/s^2)")
        axs[1, 0].grid(True)
        axs[1, 1].plot(self.cv7_data["timestamp"], self.cv7_data["ay"], label="Ay", color='tab:blue', linestyle='--')
        axs[1, 1].plot(self.navigator_data["timestamp"], self.navigator_data["ay"], label="Ay", color='tab:orange', linestyle='--')
        axs[1, 1].set_title("CV7 Ay")
        axs[1, 1].set_xlabel("Timestamp (s)")
        axs[1, 1].set_ylabel("Ay (m/s^2)")
        axs[1, 1].grid(True)
        axs[1, 2].plot(self.cv7_data["timestamp"], self.cv7_data["az"], label="Az", color='tab:blue', linestyle='--')
        axs[1, 2].plot(self.navigator_data["timestamp"], self.navigator_data["az"], label="Az", color='tab:orange', linestyle='--')
        axs[1, 2].set_title("CV7 Az")
        axs[1, 2].set_xlabel("Timestamp (s)")
        axs[1, 2].set_ylabel("Az (m/s^2)")
        axs[1, 2].grid(True)
        axs[2, 0].plot(self.cv7_data["timestamp"], self.cv7_data["gx"], label="Gx", color='tab:blue', linestyle='--')
        axs[2, 0].plot(self.navigator_data["timestamp"], self.navigator_data["gx"], label="Gx", color='tab:orange', linestyle='--')
        axs[2, 0].set_title("CV7 Gx")
        axs[2, 0].set_xlabel("Timestamp (s)")
        axs[2, 0].set_ylabel("Gx (rad/s)")
        axs[2, 0].grid(True)
        axs[2, 1].plot(self.cv7_data["timestamp"], self.cv7_data["gy"], label="Gy", color='tab:blue', linestyle='--')
        axs[2, 1].plot(self.navigator_data["timestamp"], self.navigator_data["gy"], label="Gy", color='tab:orange', linestyle='--')
        axs[2, 1].set_title("CV7 Gy")
        axs[2, 1].set_xlabel("Timestamp (s)")
        axs[2, 1].set_ylabel("Gy (rad/s)")
        axs[2, 1].grid(True)
        axs[2, 2].plot(self.cv7_data["timestamp"], self.cv7_data["gz"], label="Gz", color='tab:blue', linestyle='--')
        axs[2, 2].plot(self.navigator_data["timestamp"], self.navigator_data["gz"], label="Gz", color='tab:orange', linestyle='--')
        axs[2, 2].set_title("CV7 Gz")
        axs[2, 2].set_xlabel("Timestamp (s)")
        axs[2, 2].set_ylabel("Gz (rad/s)")
        axs[2, 2].grid(True)
        # Add legends
        for ax in axs.flat:
            ax.legend(loc='upper right')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()
    def plot_cv7_against_dead_reckon(self):
        """Plot the CV7 data against the dead reckoning data."""
        if not self.cv7_data["timestamp"] or not self.dead_reckon["timestamp"]:
            rospy.logwarn("No CV7 or Dead Reckoning data available for plotting.")
            return

        fig, axs = plt.subplots(3, 1, figsize=(10, 15))
        fig.suptitle("CV7 Data vs Dead Reckoning", fontsize=16)

        # Plot Roll
        axs[0].plot(self.cv7_data["timestamp"], self.cv7_data["roll"], label="CV7 Roll", color='blue')
        axs[0].plot(self.dead_reckon["timestamp"], self.dead_reckon["roll"], label="Dead Reckoning Roll", color='orange')
        axs[0].set_title("Roll")
        axs[0].set_xlabel("Timestamp (s)")
        axs[0].set_ylabel("Roll (degrees)")
        axs[0].grid(True)
        axs[0].legend()

        # Plot Pitch
        axs[1].plot(self.cv7_data["timestamp"], self.cv7_data["pitch"], label="CV7 Pitch", color='blue')
        axs[1].plot(self.dead_reckon["timestamp"], self.dead_reckon["pitch"], label="Dead Reckoning Pitch", color='orange')
        axs[1].set_title("Pitch")
        axs[1].set_xlabel("Timestamp (s)")
        axs[1].set_ylabel("Pitch (degrees)")
        axs[1].grid(True)
        axs[1].legend()

        # Plot Yaw
        axs[2].plot(self.cv7_data["timestamp"], self.cv7_data["yaw"], label="CV7 Yaw", color='blue')
        axs[2].plot(self.dead_reckon["timestamp"], self.dead_reckon["yaw"], label="Dead Reckoning Yaw", color='orange')
        axs[2].set_title("Yaw")
        axs[2].set_xlabel("Timestamp (s)")
        axs[2].set_ylabel("Yaw (degrees)")
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()
        # Plot the CV7 data against the dead reckoning data
if __name__ == "__main__":

    try:
        cycle_mgr = ImuDebug()
        rospy.loginfo("[%s] Imu Debug Mgr Started" % rospy.Time.now())
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("[%s] Imu Debug Mgr Interrupted" % rospy.Time.now())
    except Exception as e:
        rospy.logerr("[%s] Imu Debug Mgr Error: %s" % (rospy.Time.now(), e))
    finally:
        cycle_mgr.log_times(join(dirname(abspath(__file__)), "time_deltas.csv"))
        #cycle_mgr.plot_timestamp_progress()
        #cycle_mgr.plot_time_deltas(True
        cycle_mgr.plot_cv7_and_navigator_data()
        # Hold for 60sec
        plt.show(block=True)
        cycle_mgr.plot_cv7_against_dead_reckon()
        plt.show(block=True)
        # Hold until I close the figure
        cycle_mgr.plot_cv7_against_dead_reckon()
        plt.show(block=True)
        rospy.loginfo("[%s] Imu Debug Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Imu Debug Mgr Exiting")
        exit(0)