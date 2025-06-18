// imu_preintegrator_node.cpp
// This is a preintegration node which uses GTSAM PreintegrationBase, not manifold preintegration.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "imu_preintegrator.hpp"
#include "spurdog_acomms/PreintegrateImu.h"

#include <deque>
#include <mutex>
#include <memory>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


ImuPreintegratorNode::ImuPreintegratorNode() {
  imu_sub_ = nh_.subscribe("cv7_ahrs", 10000, &ImuPreintegratorNode::imuCallback, this);
  preint_srv_ = nh_.advertiseService("preintegrate_imu", &ImuPreintegratorNode::handlePreintegrate, this);

  // Load the noise parameters from the launch file
  double accel_noise_sigma, gyro_noise_sigma;
  double accel_bias_rw_sigma, gyro_bias_rw_sigma;
  nh_.param("accel_noise_sigma", accel_noise_sigma, 0.02);
  nh_.param("gyro_noise_sigma", gyro_noise_sigma, 0.001);
  nh_.param("accel_bias_rw_sigma", accel_bias_rw_sigma, 0.0001);
  nh_.param("gyro_bias_rw_sigma", gyro_bias_rw_sigma, 0.00001);

  // Load the bias values from the launch file
  double accel_bias_x, accel_bias_y, accel_bias_z;
  double gyro_bias_x, gyro_bias_y, gyro_bias_z;
  nh_.param("accel_bias_x", accel_bias_x, 0.0);
  nh_.param("accel_bias_y", accel_bias_y, 0.0);
  nh_.param("accel_bias_z", accel_bias_z, 0.0);
  nh_.param("gyro_bias_x", gyro_bias_x, 0.0);
  nh_.param("gyro_bias_y", gyro_bias_y, 0.0);
  nh_.param("gyro_bias_z", gyro_bias_z, 0.0);

  // Initialize the bias
  bias = gtsam::imuBias::ConstantBias(
      gtsam::Vector3(accel_bias_x, accel_bias_y, accel_bias_z),
      gtsam::Vector3(gyro_bias_x, gyro_bias_y, gyro_bias_z));

  // Initialize for NED using MakeSharedD with gravity pointing down
  preint_params_= gtsam::PreintegrationParams::MakeSharedD(0.0); // Gravity in m/s^2
  // Manufacturers Quoted Values:
  //preint_params_->accelerometerCovariance = gtsam::I_3x3 * 1.3e-5;
  //preint_params_->gyroscopeCovariance = gtsam::I_3x3 * 2.5e-7;
  preint_params_->accelerometerCovariance = gtsam::I_3x3 * accel_noise_sigma * accel_noise_sigma;
  preint_params_->gyroscopeCovariance = gtsam::I_3x3 * gyro_noise_sigma * gyro_noise_sigma;
  preint_params_->integrationCovariance = gtsam::I_3x3 * 1e-8;
  // preint_params_->biasAccCovariance = gtsam::I_3x3 * accel_bias_rw_sigma * accel_bias_rw_sigma;
  // preint_params_->biasOmegaCovariance = gtsam::I_3x3 * gyro_bias_rw_sigma * gyro_bias_rw_sigma;

  ti_ = ros::Time(0);
  prevState = gtsam::NavState(
      gtsam::Rot3(),  // Identity rotation (no initial orientation)
      gtsam::Point3(0, 0, 0),   // Initial position
      gtsam::Vector3(0, 0, 0)    // Initial velocity
  );
  req_seq_num_ = 0;  // Initialize sequence number for IMU messages
}

// Load each IMU message into a buffer
void ImuPreintegratorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  imu_buffer_.push_back(*msg);
  if (ti_ == ros::Time(0)) {
    ti_ = msg->header.stamp;
    ROS_WARN("First IMU message received at time: %f", ti_.toSec());
  }
}

// Correct Measurements for gravity
gtsam::Vector3 CorrectForGravity(const gtsam::Vector3& acc, const gtsam::Quaternion& q) {
  // Gravity in world frame
  gtsam::Vector3 gravity(0, 0, 9.81); // NED frame: z is down
  // Convert the IMU acceleration to the NED frame
  gtsam::Rot3 imu_rot = gtsam::Rot3(q).inverse();
  // Rotate gravity to the IMU frame
  gtsam::Vector3 gravity_in_imu = imu_rot.rotate(gravity);
  // ROS_INFO("Gravity in IMU frame: [%.3f, %.3f, %.3f] RPY: [%.3f, %.3f, %.3f]",
  //           gravity_in_imu.x(), gravity_in_imu.y(), gravity_in_imu.z(),
  //           imu_rot.roll() * 180.0 / M_PI,
  //           imu_rot.pitch() * 180.0 / M_PI,
  //           imu_rot.yaw() * 180.0 / M_PI);
  // Correct for gravity in the NED frame
  gtsam::Vector3 corrected_acc = acc - gravity_in_imu;
  // ROS_INFO("Initial: [%.3f, %.3f, %.3f], Corrected: [%.3f, %.3f, %.3f]",
  //           acc.x(), acc.y(), acc.z(),
  //          corrected_acc.x(), corrected_acc.y(), corrected_acc.z());
  return corrected_acc;
}

// Handle the preintegration request
bool ImuPreintegratorNode::handlePreintegrate(spurdog_acomms::PreintegrateImu::Request &req,
                          spurdog_acomms::PreintegrateImu::Response &res) {
  ros::Time tj = req.final_time;
  // Check if the requested time is valid
  if (ti_.toSec() == 0 || tj <= ti_) {
    ROS_WARN("Invalid preintegration window with start time %f and end time %f",
             ti_.toSec(), tj.toSec());
    return false;
  }
  // Check if there is anything in the IMU buffer
  if (imu_buffer_.empty()) {
    ROS_WARN("IMU buffer is empty, cannot preintegrate");
    return false;
  }
  // If this is the first time we've requested preintegration, clear the imu buffer
  if (req_seq_num_ == 0) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    ti_ = req.final_time;
    // get the latest IMU message in the buffer
    ROS_WARN("First preintegration request, clearing IMU buffer and resetting start time to %f", ti_.toSec());
    req_seq_num_++;
    // Get the orientation from the latest IMU message
    const auto& latest_imu_msg = imu_buffer_.back();
    gtsam::Quaternion q(latest_imu_msg.orientation.w,
                        latest_imu_msg.orientation.x,
                        latest_imu_msg.orientation.y,
                        latest_imu_msg.orientation.z);
    imu_buffer_.clear();
    // Set the prevState orientation to the latest IMU orientation
    prevState = gtsam::NavState(
        gtsam::Rot3(q),  // Use the latest IMU orientation
        prevState.position(),  // Keep the previous position
        prevState.velocity()   // Keep the previous velocity
    );
    // Convert prevState Attitude to RPY
    gtsam::Vector3 rpy = prevState.attitude().rpy();
    ROS_INFO("Resetting prevState to latest IMU orientation: RPY = [%.1f, %.1f, %.1f] degrees",
              rpy(0) * 180.0 / M_PI, rpy(1) * 180.0 / M_PI, rpy(2) * 180.0 / M_PI);
    return false;
  }
  req_seq_num_++;
  // We have passed initial validation checks, so we can proceed with preintegration
  // Reset the preintegration object and load current params
  gtsam::PreintegratedImuMeasurements pim(preint_params_, bias);

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  ros::Time last_msg_time = ti_;

  // Iterate through the IMU buffer and integrate measurements that fall within the time window [ti_, tj]
  while (!imu_buffer_.empty()) {
    const auto& imu_msg = imu_buffer_.front();
    ros::Time t = imu_msg.header.stamp;

    if (t <= ti_) {
      imu_buffer_.pop_front();  // discard stale messages
      continue;
    }
    if (t > tj) {
      break;  // stop once we're past the integration window
    }

    double dt = (t - last_msg_time).toSec();
    // Extract the IMU measurments
    gtsam::Quaternion q(imu_msg.orientation.w, imu_msg.orientation.x,
                        imu_msg.orientation.y, imu_msg.orientation.z);
    gtsam::Rot3 imu_rot(q);
    // Correct the IMU measurements for gravity
    gtsam::Vector3 acc = CorrectForGravity(
        gtsam::Vector3(imu_msg.linear_acceleration.x,
                       imu_msg.linear_acceleration.y,
                       imu_msg.linear_acceleration.z),
        q);
    // gtsam::Vector3 acc(imu_msg.linear_acceleration.x,
    //                   imu_msg.linear_acceleration.y,
    //                   imu_msg.linear_acceleration.z);
    gtsam::Vector3 gyro(imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y,
                        imu_msg.angular_velocity.z);
    // Log the IMU data with RPY in degrees
    // ROS_INFO("IMU data at time %f: dt(s)= %3f, rpy = [%.3f, %.3f, %.3f], acc = [%.3f, %.3f, %.3f], gyro = [%.3f, %.3f, %.3f]",
    //          t.toSec(), dt,
    //           imu_rot.roll() * 180.0 / M_PI,
    //           imu_rot.pitch() * 180.0 / M_PI,
    //           imu_rot.yaw() * 180.0 / M_PI,
    //          acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
    // Preintegrate the IMU measurements
    pim.integrateMeasurement(acc, gyro, dt);
    last_msg_time = t;
    // Replace the current NavState attitude with the IMU rotation
    prevState = gtsam::NavState(
        imu_rot,  // Use the IMU rotation
        prevState.position(),  // Keep the previous position
        prevState.velocity()   // Keep the previous velocity
    );
    // Remove the message after adding it to the preintegration object
    imu_buffer_.pop_front();
  }
  // Check that the time window matches the measurements added
  double deltaTij = pim.deltaTij();
  // Check that deltaTij is approximately equal to the requested time difference
  if (std::abs(deltaTij - (tj - ti_).toSec()) > 0.02) {
    ROS_WARN("Time difference mismatch: requested %.6f, got %.6f", (tj - ti_).toSec(), deltaTij);
    return false;
  }

  // Correct the IMU measurements for gravity
  // Predict the final state using the preintegrated measurements
  predState = pim.predict(prevState, bias);
  pim.resetIntegration();
  // Reset the prevState attitude and velocity to the resulting predicted state
  prevState= gtsam::NavState(
      predState.attitude(),  // Use the predicted attitude
      gtsam::Point3(0, 0, 0),  // Use the predicted position
      predState.velocity()   // Use the predicted velocity
  );
  // Update the ti_
  ti_ = last_msg_time;
  // Log the elements of the PredNavState
  double x, y, z, vx, vy, vz;
  x = predState.position().x();
  y = predState.position().y();
  z = predState.position().z();
  vx = predState.velocity().x();
  vy = predState.velocity().y();
  vz = predState.velocity().z();
  // Convert attitude from ROT3 to RPY
  gtsam::Quaternion q(predState.attitude().matrix());
  gtsam::Vector3 rpy = prevState.attitude().rpy();
  // Compute the error between the predicted state and the previous state
  ROS_INFO("Predicted NavState at time %f: RPY = [%.1f, %.1f, %.1f] degrees, "
           "Position = [%.3f, %.3f, %.3f], Velocity = [%.3f, %.3f, %.3f]",
           tj.toSec(),
           rpy(0) * 180.0 / M_PI,
           rpy(1) * 180.0 / M_PI,
           rpy(2) * 180.0 / M_PI,
           x, y, z,
           vx, vy, vz);
  // Create the response message
  geometry_msgs::PoseWithCovarianceStamped rel_pose_msg;

  rel_pose_msg.header.stamp = tj;
  rel_pose_msg.header.frame_id = "imu";
  // Fill the pose
  rel_pose_msg.pose.pose.position.x = x;
  rel_pose_msg.pose.pose.position.y = y;
  rel_pose_msg.pose.pose.position.z = z;
  rel_pose_msg.pose.pose.orientation.x = q.x();
  rel_pose_msg.pose.pose.orientation.y = q.y();
  rel_pose_msg.pose.pose.orientation.z = q.z();
  rel_pose_msg.pose.pose.orientation.w = q.w();
  // Fill the covariance (row-major order)
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      rel_pose_msg.pose.covariance[i * 6 + j] = 1e-6; // Set a small covariance for simplicity
    }
  }
  res.success = true;
  res.pose_delta = rel_pose_msg;

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_preintegrator_node");
  ImuPreintegratorNode node;
  ros::spin();
  return 0;
}