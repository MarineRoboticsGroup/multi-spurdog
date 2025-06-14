#!/usr/bin/env python3
import rospy
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
        if rospy.get_param("~imu_type") == "navigator_ahrs":
            self.imu_sub = rospy.Subscriber("navigator_ahrs", Imu, self.navigator_ahrs_callback)
        elif rospy.get_param("~imu_type") == "cv7_ahrs":
            self.imu_sub = rospy.Subscriber("cv7_ahrs", Imu, self.cv7_ahrs_callback)
        else:
            rospy.logerr("Unsupported IMU type specified. Please check the parameter.")
            return
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
        if rospy.get_param("~imu_type") == "navigator_ahrs":
            mean_data, sigma_data = self.calculate_mean_and_sigma(self.navigator_ahrs_data)
        elif rospy.get_param("~imu_type") == "cv7_ahrs":
            mean_data, sigma_data = self.calculate_mean_and_sigma(self.cv7_ahrs_data)
        else:
            rospy.logerr("Unsupported IMU type specified. Please check the parameter.")
            return None, None
        return mean_data, sigma_data

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
        if rospy.get_param("~imu_type") == "navigator_ahrs":
            m_data, c_data = self.get_linregression_data(self.navigator_ahrs_data)
        elif rospy.get_param("~imu_type") == "cv7_ahrs":
            m_data, c_data = self.get_linregression_data(self.cv7_ahrs_data)
        else:
            rospy.logerr("Unsupported IMU type specified. Please check the parameter.")
            return None
        return m_data, c_data

    def plot_data(self, mean_data, sigma_data, m_data, c_data):
        """ Plot acceleration and angular velocity data with respect to time.
        """
        # Plot the data in 3 rows, 2 columsn (with acceleration in the first column and angular velocity in the second column)
        # Include the mean and standard deviation in the plots as labels
        fig, axs = plt.subplots(3, 2, figsize=(12, 12))
        #axs[0, 0].plot(self.navigator_ahrs_data['accel_x'], label='Accel X')
        axs[0, 0].plot(self.cv7_ahrs_data['accel_x'], label='Accel X CV7', linestyle='--')
        # axs[0, 0].set_title('Acceleration X')
        # axs[0, 0].set_xlabel('Time')
        axs[0, 0].set_ylabel('Accel_X (m/s^2)')
        axs[0, 0].legend()
        #axs[0, 1].plot(self.navigator_ahrs_data['omega_x'], label='Omega X')
        axs[0, 1].plot(self.cv7_ahrs_data['omega_x'], label='Omega X CV7', linestyle='--')
        # axs[0, 1].set_title('Angular Velocity X')
        # axs[0, 1].set_xlabel('Time')
        axs[0, 1].set_ylabel('Omega_X (rad/s)')
        #axs[1, 0].plot(self.navigator_ahrs_data['accel_y'], label='Accel Y')
        axs[1, 0].plot(self.cv7_ahrs_data['accel_y'], label='Accel Y CV7', linestyle='--')
        # axs[1, 0].set_title('Acceleration Y')
        # axs[1, 0].set_xlabel('Time')
        axs[1, 0].set_ylabel('Accel_Y (m/s^2)')
        #axs[1, 1].plot(self.navigator_ahrs_data['omega_y'], label='Omega Y')
        axs[1, 1].plot(self.cv7_ahrs_data['omega_y'], label='Omega Y CV7', linestyle='--')
        # axs[1, 1].set_title('Angular Velocity Y')
        # axs[1, 1].set_xlabel('Time')
        axs[1, 1].set_ylabel('Omega_Y (rad/s)')
        #axs[2, 0].plot(self.navigator_ahrs_data['accel_z'], label='Accel Z')
        axs[2, 0].plot(self.cv7_ahrs_data['accel_z'], label='Accel Z CV7', linestyle='--')
        # axs[2, 0].set_title('Acceleration Z')
        # axs[2, 0].set_xlabel('Time')
        axs[2, 0].set_ylabel('Accel_Z (m/s^2)')
        #axs[2, 1].plot(self.navigator_ahrs_data['omega_z'], label='Omega Z')
        axs[2, 1].plot(self.cv7_ahrs_data['omega_z'], label='Omega Z CV7', linestyle='--')
        # axs[2, 1].set_title('Angular Velocity Z')
        # axs[2, 1].set_xlabel('Time')
        axs[2, 1].set_ylabel('Omega_Z (rad/s)')
        # Include the mean and standar deviation values as a text note on each plot
        # The accel_x plot will have the mean and sigma value of accel_x only and so on
        axs[0, 0].text(0.05, 0.95, f'Mean: {mean_data["accel_x"]:.6f} Sigma: {sigma_data["accel_x"]:.6f} Drift: {m_data["accel_x"]:.6f}*t + {c_data["accel_x"]:.6f}',transform=axs[0, 0].transAxes, fontsize=10, verticalalignment='top')
        axs[0, 1].text(0.05, 0.95, f'Mean: {mean_data["omega_x"]:.6f} Sigma: {sigma_data["omega_x"]:.6f} Drift: {m_data["omega_x"]:.6f}*t + {c_data["omega_x"]:.6f}',transform=axs[0, 1].transAxes, fontsize=10, verticalalignment='top')
        axs[1, 0].text(0.05, 0.95, f'Mean: {mean_data["accel_y"]:.6f} Sigma: {sigma_data["accel_y"]:.6f} Drift: {m_data["accel_y"]:.6f}*t + {c_data["accel_y"]:.6f}',transform=axs[1, 0].transAxes, fontsize=10, verticalalignment='top')
        axs[1, 1].text(0.05, 0.95, f'Mean: {mean_data["omega_y"]:.6f} Sigma: {sigma_data["omega_y"]:.6f} Drift: {m_data["omega_y"]:.6f}*t + {c_data["omega_y"]:.6f}',transform=axs[1, 1].transAxes, fontsize=10, verticalalignment='top')
        axs[2, 0].text(0.05, 0.95, f'Mean: {mean_data["accel_z"]:.6f} Sigma: {sigma_data["accel_z"]:.6f} Drift: {m_data["accel_z"]:.6f}*t + {c_data["accel_z"]:.6f}',transform=axs[2, 0].transAxes, fontsize=10, verticalalignment='top')
        axs[2, 1].text(0.05, 0.95, f'Mean: {mean_data["omega_z"]:.6f} Sigma: {sigma_data["omega_z"]:.6f} Drift: {m_data["omega_z"]:.6f}*t + {c_data["omega_z"]:.6f}',transform=axs[2, 1].transAxes, fontsize=10, verticalalignment='top')
        plt.tight_layout()
        plt.show()
        # Save the plot to a file
        plt.savefig('/home/morrisjp/bags/June/13June/imu_calibration_plot.png')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        imu_node = IMUSensorCalibration()
        imu_node.run()
        mean_data, sigma_data = imu_node.get_mean_and_sigma()
        m_data, c_data = imu_node.get_drift()
        if mean_data and sigma_data:
            imu_node.plot_data(mean_data, sigma_data, m_data, c_data)
    except rospy.ROSInterruptException:
        pass
