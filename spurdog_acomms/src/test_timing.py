#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32, Bool
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class TestTiming:
    def __init__(self):
        rospy.init_node('test_timing', anonymous=True)
        self.start_time = rospy.Time.now()
        self.data = []
        self.nav_state = rospy.Subscriber("nav_state", PoseStamped, self.on_nav_state) # GPS messages are also NMEA messages
        self.gps = rospy.Subscriber("gps", PoseWithCovarianceStamped, self.on_gps) # GPS messages are also NMEA messages
        self.cv7 = rospy.Subscriber("cv7_ahrs", Imu, self.on_cv7) # CV7 messages are also IMU messages
        self.navigator = rospy.Subscriber("navigator_ahrs", Imu, self.on_navigator) # Navigator messages are also IMU messages
        self.current_stamps = {
            "nav_state": None,
            "gps": None,
            "cv7": None,
            "navigator": None
        }
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
        self.nav_state_data = {
            "timestamp": [],
            "x": [],
            "y": [],
            "z": [],
            "roll": [],
            "pitch": [],
            "yaw": [],
        }
        self.gps_data = {
            "timestamp": [],
            "x": [],
            "y": [],
            "z": [],
            "position_covariance": [],
        }
        rospy.loginfo("[%s] Time Test Node Configured" % rospy.Time.now())
        rospy.sleep(1) # allow for modem to configure

    def convert_quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        quaternion = [qx, qy, qz, qw]
        euler = spt.Rotation.from_quat(quaternion).as_euler('xyz', degrees=True)
        return euler[0], euler[1], euler[2]

    def on_navigator(self, msg):
        """Callback for the navigator_ahrs topic."""
        #rospy.loginfo("[%s] Navigator AHRS: %s" % (rospy.Time.now(), msg))
        self.navigator_time = msg.header.stamp
        #self.compare_times()
        # Here you can add logic to handle the Navigator AHRS data, e.g., trigger preintegration
        # Store the Navigator data for later use
        self.current_stamps["navigator"] = msg.header.stamp
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

    def on_cv7(self, msg):
        """Callback for the cv7_ahrs topic."""
        #rospy.loginfo("[%s] CV7 AHRS: %s" % (rospy.Time.now(), msg))
        self.cv7_time = msg.header.stamp
        #self.compare_times()
        # Here you can add logic to handle the CV7 AHRS data, e.g., trigger preintegration
        # Store the CV7 data for later use
        self.current_stamps["cv7"] = msg.header.stamp
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

    def on_nav_state(self, msg):
        """Callback for the nav_state topic."""
        #rospy.loginfo("[%s] Nav State: %s" % (rospy.Time.now(), msg))
        self.nav_state_time = msg.header.stamp
        #self.compare_times()
        # Here you can add logic to handle the nav state, e.g., trigger preintegration
        # Store the nav state data for later use
        self.current_stamps["nav_state"] = msg.header.stamp
        self.nav_state_data["timestamp"].append(msg.header.stamp.to_sec())
        self.nav_state_data["x"].append(msg.pose.position.x)
        self.nav_state_data["y"].append(msg.pose.position.y)
        self.nav_state_data["z"].append(msg.pose.position.z)
        roll, pitch, yaw = self.convert_quaternion_to_euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.nav_state_data["roll"].append(roll)
        self.nav_state_data["pitch"].append(pitch)
        self.nav_state_data["yaw"].append(yaw)
        self.compare_times()

    def on_gps(self, msg):
        """Callback for the gps topic. A PoseWithCovarianceStamped message."""
        #rospy.loginfo("[%s] GPS: %s" % (rospy.Time.now(), msg))
        self.gps_time = msg.header.stamp
        #self.compare_times()
        # Here you can add logic to handle the GPS data, e.g., trigger preintegration
        # Store the GPS data for later use
        self.current_stamps["gps"] = msg.header.stamp
        self.gps_data["timestamp"].append(msg.header.stamp.to_sec())
        self.gps_data["x"].append(msg.pose.pose.position.x)
        self.gps_data["y"].append(msg.pose.pose.position.y)
        self.gps_data["z"].append(msg.pose.pose.position.z)
        self.gps_data["position_covariance"].append(msg.pose.covariance)

    def compare_times_absolute(self):
        """Print a a comparison the current timestamps of the different messages."""
        # Print a block of text with the current timestamps of the different messages
        if all(self.current_stamps.values()):
            rospy.loginfo("Current Timestamps:")
            rospy.loginfo(f"Local Time: {rospy.Time.now().to_sec()} sec")
            for key, stamp in self.current_stamps.items():
                rospy.loginfo(f"{key}: {stamp.to_sec()} sec")
        else:
            rospy.logwarn("Not all timestamps are available yet.")
    def compare_times(self):
        """Print a comparison of the current timestamps of the different messages to the local time."""
        # Print a block of text with the current timestamps of the different messages compared to the local time
        if all(self.current_stamps.values()):
            rospy.loginfo("Current Timestamps Compared to Local Time:")
            local_time = rospy.Time.now().to_sec()
            for key, stamp in self.current_stamps.items():
                time_diff = stamp.to_sec() - local_time
                # Calculate the approximate rate that the timestamp is changing
                rospy.loginfo(f"{key}: Difference: {time_diff:.6f} sec)")
        else:
            rospy.logwarn("Not all timestamps are available yet.")
    def plot_position_data_for_comparison(self):
        """Plot the X,Y,Z,R,P,Y data for comparison
           Uses the Nav State and GPS values for X, Y, Z
            Uses the Nav State, Navigator and CV7 orientation for RPY
        """
        if not self.nav_state_data["timestamp"] or not self.gps_data["timestamp"]:
            rospy.logwarn("No data available for plotting.")
            return

        # Convert timestamps to numpy arrays
        nav_state_timestamps = np.array(self.nav_state_data["timestamp"])
        gps_timestamps = np.array(self.gps_data["timestamp"])

        # Create a figure with subplots
        fig, axs = plt.subplots(3, 2, figsize=(12, 10))
        fig.suptitle('Position and Orientation Data Comparison')

        # Plot Nav State Position
        axs[0, 0].plot(nav_state_timestamps, self.nav_state_data["x"], label='Nav State X', color='blue')
        axs[0, 0].plot(gps_timestamps, self.gps_data["x"], label='GPS X', color='orange')
        axs[0, 0].set_title('X Position Comparison')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('X Position (m)')
        axs[0, 0].legend()

        # Plot Nav State Position
        axs[1, 0].plot(nav_state_timestamps, self.nav_state_data["y"], label='Nav State Y', color='blue')
        axs[1, 0].plot(gps_timestamps, self.gps_data["y"], label='GPS Y', color='orange')
        axs[1, 0].set_title('Y Position Comparison')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Y Position (m)')
        axs[1, 0].legend()

        # Plot Nav State Position
        axs[2, 0].plot(nav_state_timestamps, self.nav_state_data["z"], label='Nav State Z', color='blue')
        axs[2, 0].plot(gps_timestamps, self.gps_data["z"], label='GPS Z', color='orange')
        axs[2, 0].set_title('Z Position Comparison')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylabel('Z Position (m)')
        axs[2, 0].legend()

        # Plot COllective Roll
        axs[0, 1].plot(nav_state_timestamps, self.nav_state_data["roll"], label='Nav State Roll', color='blue')
        axs[0, 1].plot(self.cv7_data["timestamp"], self.cv7_data["roll"], label='CV7 Roll', color='green')
        axs[0, 1].plot(self.navigator_data["timestamp"], self.navigator_data["roll"], label='Navigator Roll', color='red')
        axs[0, 1].set_title('Roll Comparison')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Roll (degrees)')
        axs[0, 1].legend()
        # Plot Collective Pitch
        axs[1, 1].plot(nav_state_timestamps, self.nav_state_data["pitch"], label='Nav State Pitch', color='blue')
        axs[1, 1].plot(self.cv7_data["timestamp"], self.cv7_data["pitch"], label='CV7 Pitch', color='green')
        axs[1, 1].plot(self.navigator_data["timestamp"], self.navigator_data["pitch"], label='Navigator Pitch', color='red')
        axs[1, 1].set_title('Pitch Comparison')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Pitch (degrees)')
        axs[1, 1].legend()
        # Plot Collective Yaw
        axs[2, 1].plot(nav_state_timestamps, self.nav_state_data["yaw"], label='Nav State Yaw', color='blue')
        axs[2, 1].plot(self.cv7_data["timestamp"], self.cv7_data["yaw"], label='CV7 Yaw', color='green')
        axs[2, 1].plot(self.navigator_data["timestamp"], self.navigator_data["yaw"], label='Navigator Yaw', color='red')
        axs[2, 1].set_title('Yaw Comparison')
        axs[2, 1].set_xlabel('Time (s)')
        axs[2, 1].set_ylabel('Yaw (degrees)')
        axs[2, 1].legend()
        # Adjust layout
        plt.subplots_adjust(hspace=0.4, wspace=0.3)
        # Add a title to the figure
        fig.suptitle('Position and Orientation Data Comparison', fontsize=16)
        plt.tight_layout(rect=[0.0, 0.03, 1.0, 0.95])
        plt.show()

    def save_plot_to_png(self, filename):
        """Save the current plot to a PNG file."""
        if not self.nav_state_data["timestamp"] or not self.gps_data["timestamp"]:
            rospy.logwarn("No data available for saving plot.")
            return

        # Create the plot
        self.plot_position_data_for_comparison()

        # Save the plot to a PNG file
        plt.savefig(filename)
        rospy.loginfo(f"Plot saved to {filename}")
        plt.close()

    def save_data_to_csv(self, filename):
        """Save the collected data to a CSV file."""
        if not self.nav_state_data["timestamp"]:
            rospy.logwarn("No data available for saving to CSV.")
            return

        # Create a DataFrame for each data type
        nav_state_df = pd.DataFrame(self.nav_state_data)
        gps_df = pd.DataFrame(self.gps_data)
        cv7_df = pd.DataFrame(self.cv7_data)
        navigator_df = pd.DataFrame(self.navigator_data)

        # Combine all DataFrames into one
        combined_df = pd.concat([nav_state_df, gps_df, cv7_df, navigator_df], axis=1)

        # Save to CSV
        combined_df.to_csv(filename, index=False)
        rospy.loginfo(f"Data saved to {filename}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        test_timing = TestTiming()
        test_timing.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        # Save data to CSV and plot to PNG before exiting
        test_timing.save_plot_to_png("test_timing_plot.png")
        test_timing.save_data_to_csv("test_timing_data.csv")
        rospy.loginfo("Test Timing Node has finished execution.")