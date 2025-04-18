#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import Imu
from spurdog_acomms.srv import PreintegrateIMU, PreintegrateIMUResponse
from collections import deque

class IMUSensorHandler:
    def __init__(self):
        rospy.init_node('imu_sensor_handler', anonymous=True)
        # Parameters
        self.buffer_size = rospy.get_param("~buffer_size", 1000)  # Max number of messages to store
        self.imu_buffer = deque(maxlen=self.buffer_size)
        self.available_noise_params = {
            'navigator_ahrs': {
                'accel_noise': np.array([0.01, 0.01, 0.01]),
                'gyro_noise': np.array([0.01, 0.01, 0.01]),
                'accel_bias': np.array([0.0, 0.0, 0.0]),
                'gyro_bias': np.array([0.0, 0.0, 0.0])
            },
            'microstrain_cv7': {
                'accel_noise': np.array([0.005, 0.005, 0.005]),
                'gyro_noise': np.array([0.005, 0.005, 0.005]),
                'accel_bias': np.array([0.0, 0.0, 0.0]),
                'gyro_bias': np.array([0.0, 0.0, 0.0])
            }
        }
        self.noise_params = {}

        # IMU Subscriber
        if rospy.get_param("~imu_type") == "navigator_ahrs":
            self.noise_params = self.available_noise_params['navigator_ahrs']
            self.imu_sub = rospy.Subscriber("navigator_ahrs", Imu, self.navigator_ahrs_callback)
        elif rospy.get_param("~imu_type") == "microstrain_cv7":
            self.noise_params = self.available_noise_params['microstrain_cv7']
            self.imu_sub = rospy.Subscriber("cv7_ahrs", Imu, self.cv7_ahrs_callback)
        else:
            rospy.logerr("Unsupported IMU type specified. Please check the parameter.")
            return

        # IMU Preintegration Service
        self.service = rospy.Service("preintegrate_imu", PreintegrateIMU, self.handle_preintegration_request)

    def navigator_ahrs_callback(self, msg):
        """
        Callback function to store IMU messages in a time-indexed list.
        """
        timestamp = msg.header.stamp.to_sec()
        self.imu_buffer.append((timestamp, msg))
        rospy.loginfo(f"Stored IMU message at time {timestamp}")

    def cv7_ahrs_callback(self, msg):
        """
        Callback function to store IMU messages in a time-indexed list.
        """
        timestamp = msg.header.stamp.to_sec()
        orientation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
        omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        linear_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        # Correct linear acceleration for gravity
        orientation, linear_acc, omega = self.convert_cv7_to_rh_axes_and_scale(orientation, linear_acc, omega)
        # Remove gravity from linear acceleration
        n_gravity = np.array([0, 0, -9.80665])
        linear_acc -= n_gravity
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = linear_acc
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = omega
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = R.from_matrix(orientation).as_quat()
        # Store the modified message
        self.imu_buffer.append((timestamp, msg))
        rospy.loginfo(f"Stored IMU message at time {timestamp}")

    def convert_cv7_to_rh_axes_and_scale(self, orientation, accel, omega):
        """
        Convert CV7 orientation from quaternion to right-handed rotation matrix.
        """
        orientation = R.from_euler('z', 180, degrees=True).apply(orientation)
        accel_rh = R.from_euler('z', 180, degrees=True).apply(accel)
        omega_rh = R.from_euler('z', 180, degrees=True).apply(omega)
        accel_scaled = accel_rh * -9.81  # Scale to m/s²

        return orientation, accel_scaled, omega_rh

    def handle_preintegration_request(self, req):
        """
        Process an IMU preintegration request between t1 and t2.
        """
        t1, t2 = req.t1, req.t2
        selected_msgs = [msg for ts, msg in self.imu_buffer if t1 <= ts <= t2]

        if len(selected_msgs) < 2:
            rospy.logwarn("Not enough IMU data to perform preintegration!")
            # For testing: return a test response with zero values
            # In a real scenario, you might want to raise an exception or return an error response
            preintegrated_meas = PoseWithCovariance()
            preintegrated_meas.pose.position.x = 0.0
            preintegrated_meas.pose.position.y = 0.0
            preintegrated_meas.pose.position.z = 0.0
            preintegrated_meas.pose.orientation.x = 0.0
            preintegrated_meas.pose.orientation.y = 0.0
            preintegrated_meas.pose.orientation.z = 0.0
            preintegrated_meas.pose.orientation.w = 1.0
            cov = np.diag([1.0] * 6)  # 6x6 covariance matrix
            preintegrated_meas.covariance = cov.flatten().tolist()  # Flatten covariance for ROS message
            return preintegrated_meas
        else:
            dX, dR, cov = self.preintegrate_imu_basic(selected_msgs)
        # Placeholder: Compute relative pose delta and covariance here

        preintegrated_meas = PoseWithCovariance()
        preintegrated_meas.pose.position.x = dX[0]
        preintegrated_meas.pose.position.y = dX[1]
        preintegrated_meas.pose.position.z = dX[2]
        preintegrated_meas.pose.orientation = R.from_matrix(dR).as_quat()
        preintegrated_meas.covariance = cov.flatten().tolist()  # Flatten covariance for ROS message

        rospy.loginfo(f"Processing IMU preintegration from {t1} to {t2}.")

        return PreintegrateIMUResponse(preintegrated_meas)

    def preintegrate_imu_basic(self, imu_msgs):
        """
        Preintegrate IMU measurements to compute relative pose, rotation, and covariance.

        Args:
            imu_msgs (list of sensor_msgs/Imu): List of IMU messages, sorted by time.
            accel_sigma (float): Standard deviation of linear acceleration noise.
            gyro_sigma (float): Standard deviation of angular velocity noise.

        Returns:
            relative_position (np.ndarray): 3x1 relative displacement (meters).
            relative_rotation (np.ndarray): 3x3 rotation matrix (SO(3)).
            covariance (np.ndarray): 6x6 covariance matrix of pose change.
        """

        # Gravity vector (z-down, consistent with CV7 IMU)
        #g = np.array([0, 0, -9.80665])  # m/s²
        g = np.array([0,0,0])
        # Initialize state
        delta_p = np.zeros(3)  # Position change
        delta_v = np.zeros(3)  # Velocity change
        delta_R = np.eye(3)  # Rotation change

        # Initialize covariance (6x6: [rotation(3x3), position(3x3)])
        covariance = np.zeros((6, 6))

        prev_msg = imu_msgs[0]
        prev_timestamp = prev_msg.header.stamp.to_sec()

        # Convert noise to covariance
        accel_cov = (self.noise_params['accel_noise'] ** 2) * np.eye(3)
        gyro_cov = (self.noise_params['gyro_noise'] ** 2) * np.eye(3)

        for msg in imu_msgs[1:]:
            # Time difference
            t = msg.header.stamp.to_sec()
            dt = t - prev_timestamp
            if dt <= 0:
                continue  # Skip invalid timestamps

            # Extract IMU measurements
            w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])  # rad/s
            a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])  # m/s²

            # Correct acceleration for gravity
            a_corrected = a - g

            # Compute incremental rotation using SO(3) approximation (small-angle)
            w_dt = w * dt
            dR = R.from_rotvec(w_dt).as_matrix()  # Convert angular velocity to rotation matrix
            delta_R = delta_R @ dR  # Integrate rotation

            # Compute velocity and position updates (midpoint method)
            delta_v += delta_R @ a_corrected * dt
            delta_p += delta_v * dt + 0.5 * delta_R @ a_corrected * (dt ** 2)

            # Update covariance (first-order propagation)
            F = np.eye(6)
            F[:3, :3] = dR.T  # Rotation part
            F[3:, :3] = -dt * delta_R @ np.cross(np.eye(3), a_corrected)  # Coupling of position and rotation

            L = np.zeros((6, 6))
            L[:3, :3] = gyro_cov
            L[3:, 3:] = accel_cov * (dt ** 2)  # Accelerometer noise affects position

            covariance = F @ covariance @ F.T + L

            # Update previous timestamp
            prev_timestamp = t

        return delta_p, delta_R, covariance

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        imu_node = IMUSensorHandler()
        imu_node.run()
    except rospy.ROSInterruptException:
        pass
