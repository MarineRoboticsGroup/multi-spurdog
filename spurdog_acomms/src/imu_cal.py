#!/usr/bin/env python3
import rospy
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance, Pose, Quaternion
from sensor_msgs.msg import Imu
from collections import deque
import matplotlib.pyplot as plt

class IMUSensorCalibration:
    def __init__(self):
        rospy.init_node('imu_sensor_handler', anonymous=True)

        # IMU Subscriber
        self.imu_sub = rospy.Subscriber("navigator_ahrs", Imu, self.navigator_ahrs_callback)
        self.imu_sub = rospy.Subscriber("cv7_ahrs", Imu, self.cv7_ahrs_callback)

        self.navigator_ahrs_data = {
            "timestamp": [],
            'qx': [],
            'qy': [],
            'qz': [],
            'qw': [],
            'accel_x': [],
            'accel_y': [],
            'accel_z': [],
            'omega_x': [],
            'omega_y': [],
            'omega_z': [],
        }
        self.cv7_ahrs_data = {
            "timestamp": [],
            'qx': [],
            'qy': [],
            'qz': [],
            'qw': [],
            'accel_x': [],
            'accel_y': [],
            'accel_z': [],
            'omega_x': [],
            'omega_y': [],
            'omega_z': [],
        }

    def cv7_ahrs_callback(self, msg):
        """ Add each measument to the storage lists. """
        self.cv7_ahrs_data['timestamp'].append(msg.header.stamp.to_sec())
        self.cv7_ahrs_data['qx'].append(msg.orientation.x)
        self.cv7_ahrs_data['qy'].append(msg.orientation.y)
        self.cv7_ahrs_data['qz'].append(msg.orientation.z)
        self.cv7_ahrs_data['qw'].append(msg.orientation.w)
        self.cv7_ahrs_data['accel_x'].append(msg.linear_acceleration.x)
        self.cv7_ahrs_data['accel_y'].append(msg.linear_acceleration.y)
        self.cv7_ahrs_data['accel_z'].append(msg.linear_acceleration.z)
        self.cv7_ahrs_data['omega_x'].append(msg.angular_velocity.x)
        self.cv7_ahrs_data['omega_y'].append(msg.angular_velocity.y)
        self.cv7_ahrs_data['omega_z'].append(msg.angular_velocity.z)

    def navigator_ahrs_callback(self, msg):
        """ Add each measument to the storage lists. """
        self.navigator_ahrs_data['timestamp'].append(msg.header.stamp.to_sec())
        self.navigator_ahrs_data['qx'].append(msg.orientation.x)
        self.navigator_ahrs_data['qy'].append(msg.orientation.y)
        self.navigator_ahrs_data['qz'].append(msg.orientation.z)
        self.navigator_ahrs_data['qw'].append(msg.orientation.w)
        self.navigator_ahrs_data['accel_x'].append(msg.linear_acceleration.x)
        self.navigator_ahrs_data['accel_y'].append(msg.linear_acceleration.y)
        self.navigator_ahrs_data['accel_z'].append(msg.linear_acceleration.z)
        self.navigator_ahrs_data['omega_x'].append(msg.angular_velocity.x)
        self.navigator_ahrs_data['omega_y'].append(msg.angular_velocity.y)
        self.navigator_ahrs_data['omega_z'].append(msg.angular_velocity.z)

    def calculate_mean_and_sigma(self, data):
        """ Calculate the mean of the collected data. """
        # Itetarate throught he imu data and returna  dictionary with the mean values
        mean_data = {}
        sigma_data = {}
        for key in data.keys():
            if data[key]:
                mean_data[key] = np.mean(data[key])
            else:
                mean_data[key] = 0.0
            if data[key]:
                sigma_data[key] = np.std(data[key])
            else:
                sigma_data[key] = 0.0
            # Log the mean and standard deviation to terminal
            rospy.loginfo(f"Mean for {key}: {mean_data[key]}, Sigma: {sigma_data[key]}")
        return mean_data, sigma_data

    def get_mean_and_sigma(self):
        """ Get the mean and standard deviation for both IMU types. """
        if not self.navigator_ahrs_data['timestamp']:
            rospy.logwarn("No data received from navigator_ahrs.")
            nav_mean_data, nav_sigma_data = None, None
        else:
            rospy.loginfo("Data received from navigator_ahrs.")
            nav_mean_data, nav_sigma_data = self.calculate_mean_and_sigma(self.navigator_ahrs_data)
        if not self.cv7_ahrs_data['timestamp']:
            rospy.logwarn("No data received from cv7_ahrs.")
            cv7_mean_data, cv7_sigma_data = None, None
        else:
            rospy.loginfo("Data received from cv7_ahrs.")
            cv7_mean_data, cv7_sigma_data = self.calculate_mean_and_sigma(self.cv7_ahrs_data)
        return nav_mean_data, nav_sigma_data, cv7_mean_data, cv7_sigma_data

    def get_linregression_data(self, data):
        """ Get the linear regression data for the given x and y data. """
        m_data = {}
        c_data = {}
        for key in data.keys():
            if data[key]:
                # Calculate the linear regression coefficients
                x = np.arange(len(data["timestamp"]))
                y = np.array(data[key])
                m, c = np.polyfit(x, y, 1)
                m_data[key] = m
                c_data[key] = c
            else:
                m_data[key] = 0.0
                c_data[key] = 0.0
            # Log the linear regression coefficients to terminal
            rospy.loginfo(f"Linear regression for {key}: m = {m_data[key]}, c = {c_data[key]}")
        return m_data, c_data

    def get_drift(self):
        """ Calculate the drift of the IMU data. """
        # Check if we have data for the navigator_ahrs
        if not self.navigator_ahrs_data['timestamp']:
            rospy.logwarn("No data received from navigator_ahrs for drift calculation.")
            nav_m_data, nav_c_data = None, None
        else:
            rospy.loginfo("Calculating drift for navigator_ahrs.")
            nav_m_data, nav_c_data = self.get_linregression_data(self.navigator_ahrs_data)
        # Check if we have data for the cv7_ahrs
        if not self.cv7_ahrs_data['timestamp']:
            rospy.logwarn("No data received from cv7_ahrs for drift calculation.")
            cv7_m_data, cv7_c_data = None, None
        else:
            rospy.loginfo("Calculating drift for cv7_ahrs.")
            cv7_m_data, cv7_c_data = self.get_linregression_data(self.cv7_ahrs_data)
        return nav_m_data, nav_c_data, cv7_m_data, cv7_c_data

    def get_accel_bias(self, imu_data):
        """ Calculate the acceleration bias for the given IMU data. """
        # Use the mean quaternion orientation to correct for gravity
        # Upright is 0,0,0,1 quaternion (x, y, z, w)
        upright_quat = np.array([0.0, 0.0, 0.0, 1.0])
        g = 9.8066  # Gravity in m/s^2
        # Get the mean quaternion orientation
        if imu_data == self.cv7_ahrs_data:
            imu_name = "cv7_ahrs"
        elif imu_data == self.navigator_ahrs_data:
            imu_name = "navigator_ahrs"
        else:
            rospy.logerr("Unknown IMU data type.")
            return None
        # Get the mean quaternion for the IMU data
        qx_mean = np.mean(imu_data['qx'])
        qy_mean = np.mean(imu_data['qy'])
        qz_mean = np.mean(imu_data['qz'])
        qw_mean = np.mean(imu_data['qw'])
        # Get the mean orientation in rpy
        mean_quat = np.array([qx_mean, qy_mean, qz_mean, qw_mean])
        mean_rpy = R.from_quat(mean_quat).as_euler('xyz', degrees=True)
        mean_rot = R.from_quat(mean_quat).as_dcm()
        # Log the mean rpy orientation to terminal
        rospy.loginfo(f"{imu_name} mean orientation (rpy)(deg): {mean_rpy}")
        # Calculate the gravity vector in the navigator frame
        gravity_vector = np.array([0.0, 0.0, g])
        # Rotate the gravity vector into the IMU frame
        corrected_gravity_vector = mean_rot @ gravity_vector
        # Log the corrected gravity vector to terminal
        #rospy.loginfo(f"{imu_name} corrected gravity vector: {corrected_gravity_vector}")
        # Get the affect of gravity on each axis
        gravity_by_axis = {
            'accel_x': corrected_gravity_vector[0],
            'accel_y': corrected_gravity_vector[1],
            'accel_z': corrected_gravity_vector[2]
        }
        # Log the gravity effect on each axis to terminal
        #rospy.loginfo(f"{imu_name} gravity effect on axes: {Gravity_by_axis}")
        # Calculate the acceleration bias for each axis
        accel_bias = {}
        for key in imu_data.keys():
            if key.startswith('accel_'):
                # Calculate the bias as the mean acceleration minus the gravity effect
                if imu_data[key]:
                    accel_bias[key] = np.mean(imu_data[key]) - gravity_by_axis[key]
                    rospy.loginfo(f"{imu_name} {key} mean: {np.mean(imu_data[key])}, gravity: {gravity_by_axis[key]}")
                else:
                    accel_bias[key] = 0.0
                # Log the acceleration bias to terminal
                rospy.loginfo(f"{imu_name} {key} bias: {accel_bias[key]}")
            else:
                accel_bias[key] = 0.0
        # Log the acceleration bias to terminal
        rospy.loginfo(f"{imu_name} acceleration bias: {accel_bias}")

        return accel_bias

    def plot_data(self, imu_data, mean_data, sigma_data, m_data, c_data):
        """ Plot acceleration and angular velocity data with respect to time.
        """
        # Plot the data in 3 rows, 2 columsn (with acceleration in the first column and angular velocity in the second column)
        # Include the mean and standard deviation in the plots as labels
        fig, axs = plt.subplots(3, 2, figsize=(12, 12))
        if imu_data is None or mean_data is None or sigma_data is None or m_data is None or c_data is None:
            rospy.logwarn("No data to plot.")
            return
        elif imu_data == self.navigator_ahrs_data:
            title = "Navigator AHRS Data"
        else:
            title = "CV7 AHRS Data"
        fig.suptitle(title, fontsize=16)
        # Add some space between the title and the subplots
        plt.subplots_adjust(top=0.9, hspace=0.4, wspace=0.3)
        # Accel X subplot
        axs[0, 0].plot(imu_data['accel_x'], label='Accel X', linestyle='--')
        axs[0, 0].set_ylabel('Accel_X (m/s^2)')
        axs[0, 0].text(0.05, 0.95,
            f'Mean: {mean_data["accel_x"]:.6f} Sigma: {sigma_data["accel_x"]:.6f} Drift: {m_data["accel_x"]:.6f}*t + {c_data["accel_x"]:.6f}',
            transform=axs[0, 0].transAxes, fontsize=10, verticalalignment='top')
        # Accel Y subplot
        axs[0, 1].plot(imu_data['accel_y'], label='Accel Y', linestyle='--')
        axs[0, 1].set_ylabel('Accel_Y (m/s^2)')
        axs[0, 1].text(0.05, 0.95,
            f'Mean: {mean_data["accel_y"]:.6f} Sigma: {sigma_data["accel_y"]:.6f} Drift: {m_data["accel_y"]:.6f}*t + {c_data["accel_y"]:.6f}',
            transform=axs[0, 1].transAxes, fontsize=10, verticalalignment='top')
        # Accel Z subplot
        axs[1, 0].plot(imu_data['accel_z'], label='Accel Z', linestyle='--')
        axs[1, 0].set_ylabel('Accel_Z (m/s^2)')
        axs[1, 0].text(0.05, 0.95,
            f'Mean: {mean_data["accel_z"]:.6f} Sigma: {sigma_data["accel_z"]:.6f} Drift: {m_data["accel_z"]:.6f}*t + {c_data["accel_z"]:.6f}',
            transform=axs[1, 0].transAxes, fontsize=10, verticalalignment='top')
        # Omega X subplot
        axs[1, 1].plot(imu_data['omega_x'], label='Omega X', linestyle='--')
        axs[1, 1].set_ylabel('Omega_X (rad/s)')
        axs[1, 1].text(0.05, 0.95,
            f'Mean: {mean_data["omega_x"]:.6f} Sigma: {sigma_data["omega_x"]:.6f} Drift: {m_data["omega_x"]:.6f}*t + {c_data["omega_x"]:.6f}',
            transform=axs[1, 1].transAxes, fontsize=10, verticalalignment='top')
        # Omega Y subplot
        axs[2, 0].plot(imu_data['omega_y'], label='Omega Y', linestyle='--')
        axs[2, 0].set_ylabel('Omega_Y (rad/s)')
        axs[2, 0].text(0.05, 0.95,
            f'Mean: {mean_data["omega_y"]:.6f} Sigma: {sigma_data["omega_y"]:.6f} Drift: {m_data["omega_y"]:.6f}*t + {c_data["omega_y"]:.6f}',
            transform=axs[2, 0].transAxes, fontsize=10, verticalalignment='top')
        # Omega Z subplot
        axs[2, 1].plot(imu_data['omega_z'], label='Omega Z', linestyle='--')
        axs[2, 1].set_ylabel('Omega_Z (rad/s)')
        axs[2, 1].text(0.05, 0.95,
            f'Mean: {mean_data["omega_z"]:.6f} Sigma: {sigma_data["omega_z"]:.6f} Drift: {m_data["omega_z"]:.6f}*t + {c_data["omega_z"]:.6f}',
            transform=axs[2, 1].transAxes, fontsize=10, verticalalignment='top')
        # Set the x-axis label for all subplots
        for ax in axs.flat:
            ax.set_xlabel('Time (s)')
            ax.grid(True)
        #plt.tight_layout()
        # Save the figure
        rospy.loginfo(f"Plot saved as {title.replace(' ', '_').lower()}_plot.png")
        save_path = os.path.join(os.path.expanduser("~"), f"{title.replace(' ', '_').lower()}_plot.png")
        plt.savefig(save_path)
        # Log the location where the plot is saved
        rospy.loginfo(f"Plot saved at: {save_path}")
        plt.show()
        plt.close()

if __name__ == '__main__':
    imu_node = IMUSensorCalibration()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        nav_mean_data, nav_sigma_data, cv7_mean_data, cv7_sigma_data = imu_node.get_mean_and_sigma()
        nav_m_data, nav_c_data, cv7_m_data, cv7_c_data = imu_node.get_drift()
        accel_bias = imu_node.get_accel_bias(imu_node.cv7_ahrs_data)
        #imu_node.plot_data(imu_node.cv7_ahrs_data, cv7_mean_data, cv7_sigma_data, cv7_m_data, cv7_c_data)
        #imu_node.plot_data(imu_node.navigator_ahrs_data, nav_mean_data, nav_sigma_data, nav_m_data, nav_c_data)

        rospy.loginfo("IMU calibration completed. Check the plots for results.")
