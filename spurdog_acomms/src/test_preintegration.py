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
        self.navigator_offset = 4496
        # Start the cycle
        rospy.loginfo("[%s] Extractor Node Configured" % rospy.Time.now())
        rospy.sleep(1) # allow for modem to configure

    def on_nav_state(self, msg):
        """Callback for the nav_state topic."""
        #rospy.loginfo("[%s] Nav State: %s" % (rospy.Time.now(), msg))
        self.nav_state_time = msg.header.stamp
        self.compare_times()
        # Here you can add logic to handle the nav state, e.g., trigger preintegration
    def on_gps(self, msg):
        """Callback for the gps topic."""
        #rospy.loginfo("[%s] GPS: %s" % (rospy.Time.now(), msg))
        self.gps_time = msg.header.stamp
        self.compare_times()
        # Here you can add logic to handle the GPS data, e.g., trigger preintegration
    def on_cv7(self, msg):
        """Callback for the cv7_ahrs topic."""
        #rospy.loginfo("[%s] CV7 AHRS: %s" % (rospy.Time.now(), msg))
        self.cv7_time = msg.header.stamp
        self.compare_times()
        # Here you can add logic to handle the CV7 AHRS data, e.g., trigger preintegration
    def on_navigator(self, msg):
        """Callback for the navigator_ahrs topic."""
        #rospy.loginfo("[%s] Navigator AHRS: %s" % (rospy.Time.now(), msg))
        self.navigator_time = msg.header.stamp - rospy.Duration(self.navigator_offset)
        self.compare_times()
        # Here you can add logic to handle the Navigator AHRS data, e.g., trigger preintegration
    def on_txf(self, msg):
        """Callback for the modem txf topic."""
        #rospy.loginfo("[%s] Modem TXF: %s" % (rospy.Time.now(), msg))
        self.modem_time = msg.stamp
        self.compare_times()

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
        cycle_mgr.plot_timestamp_progress()
        #cycle_mgr.plot_time_deltas(True)
        # Hold until I close the figure

        rospy.loginfo("[%s] Imu Debug Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Imu Debug Mgr Exiting")
        exit(0)