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
  preint_params_= gtsam::PreintegrationParams::MakeSharedD(0.0);
  // Manufacturers Quoted Values:
  //preint_params_->accelerometerCovariance = gtsam::I_3x3 * 1.3e-5;
  //preint_params_->gyroscopeCovariance = gtsam::I_3x3 * 2.5e-7;
  preint_params_->accelerometerCovariance = gtsam::I_3x3 * accel_noise_sigma * accel_noise_sigma;
  preint_params_->gyroscopeCovariance = gtsam::I_3x3 * gyro_noise_sigma * gyro_noise_sigma;
  preint_params_->integrationCovariance = gtsam::I_3x3 * 1e-2;
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

// Correct Measurements for gravity and integrate them
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

bool ImuPreintegratorNode::handlePreintegrate(spurdog_acomms::PreintegrateImu::Request &req,
                          spurdog_acomms::PreintegrateImu::Response &res) {
  ros::Time tj = req.final_time;
  // If this is the first time we've requested preintegration, clear the imu buffer
  if (ti_.toSec() == 0 || tj <= ti_) {
    ROS_WARN("Invalid preintegration window with start time %f and end time %f",
             ti_.toSec(), tj.toSec());
    return false;
  }
  if (req_seq_num_ == 0) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.clear();
    ti_ = req.final_time;
    // get the latest IMU message in the buffer
    ROS_WARN("First preintegration request, clearing IMU buffer and resetting start time to %f", ti_.toSec());
    if (!imu_buffer_.empty()) {
      const auto& latest_imu_msg = imu_buffer_.back();
      // Get the orientation from the latest IMU message
      gtsam::Quaternion q(latest_imu_msg.orientation.w,
                          latest_imu_msg.orientation.x,
                          latest_imu_msg.orientation.y,
                          latest_imu_msg.orientation.z);
      // Set the prevState orientation to the latest IMU orientation
      prevState = gtsam::NavState(
          gtsam::Rot3(q),  // Use the latest IMU orientation
          prevState.position(),  // Keep the previous position
          prevState.velocity()   // Keep the previous velocity
      );
    }
    ROS_INFO("Cleared IMU buffer for preintegration request.");
  }
  // Increment the sequence number for the next request
  req_seq_num_++;
  // Reset to start the preintegrations
  gtsam::PreintegratedImuMeasurements pim(preint_params_);
  pim.resetIntegrationAndSetBias(bias);

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  ros::Time last_msg_time = ti_;

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
    // Correct the IMU acceleration for gravity
    gtsam::Quaternion q(imu_msg.orientation.w, imu_msg.orientation.x,
                        imu_msg.orientation.y, imu_msg.orientation.z);
    // Convert to NED frame (roll, pitch, yaw)
    gtsam::Rot3 imu_rot(q);

    gtsam::Vector3 acc(imu_msg.linear_acceleration.x,
                      imu_msg.linear_acceleration.y,
                      imu_msg.linear_acceleration.z);
    // Correct the acceleration for gravity
    acc = CorrectForGravity(acc, q);
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
    pim.integrateMeasurement(acc, gyro, dt);
    last_msg_time = t;

    imu_buffer_.pop_front();  // remove after using
  }
  // Predict the final state using the preintegrated measurements
  predState = pim.predict(prevState, bias);

  // Integrate the measurements
  double deltaTij = pim.deltaTij();
  // Check that deltaTij is approximately equal to the requested time difference
  if (std::abs(deltaTij - (tj - ti_).toSec()) > 0.02) {
    ROS_WARN("Time difference mismatch: requested %.6f, got %.6f", (tj - ti_).toSec(), deltaTij);
    return false;
  }
  const gtsam::Rot3& deltaRij = pim.deltaRij();
  const gtsam::Vector3& deltaPij = pim.deltaPij();
  gtsam::Pose3 delta_pose(deltaRij, deltaPij);
  gtsam::Matrix covariance = pim.preintMeasCov();
  double norm_deltaPij = deltaPij.norm();
  // Get the difference between the predicted position and the previous state
  gtsam::Vector3 deltaP = predState.position() - prevState.position();
  double norm_deltaP = deltaP.norm();
  ROS_INFO("Preintegrated IMU measurements over deltaTij = %.2f, |deltaPij| = %.2f, |deltaP| = %.2f", deltaTij, norm_deltaPij, norm_deltaP);
  // Log the predicted state
  ROS_INFO("Predicted state at time %f: position = [%.3f, %.3f, %.3f], "
           "velocity = [%.3f, %.3f, %.3f], orientation = [%.3f, %.3f, %.3f, %.3f]",
           tj.toSec(),
           predState.position().x(), predState.position().y(), predState.position().z(),
           predState.velocity().x(), predState.velocity().y(), predState.velocity().z(),
           predState.attitude().toQuaternion().x(), predState.attitude().toQuaternion().y(),
           predState.attitude().toQuaternion().z(), predState.attitude().toQuaternion().w());
  // Set the initial state for the next preintegration using predState
  prevState = predState;
  // Log the preintegrated measurements
  // ROS_INFO_STREAM("Preintegrated measurements from " << ti_ << " to " << tj
  //                 << ": deltaRij = " << deltaRij
  //                 << ", deltaPij = " << deltaPij
  //                 << ", covariance = " << covariance);
  // Update the ti_
  ti_ = last_msg_time;
  // Create the response message
  geometry_msgs::PoseWithCovarianceStamped rel_pose_msg;

  rel_pose_msg.header.stamp = tj;
  rel_pose_msg.header.frame_id = "imu";
  // Fill the pose
  rel_pose_msg.pose.pose.position.x = delta_pose.translation().x();
  rel_pose_msg.pose.pose.position.y = delta_pose.translation().y();
  rel_pose_msg.pose.pose.position.z = delta_pose.translation().z();
  gtsam::Quaternion q = delta_pose.rotation().toQuaternion();
  rel_pose_msg.pose.pose.orientation.x = q.x();
  rel_pose_msg.pose.pose.orientation.y = q.y();
  rel_pose_msg.pose.pose.orientation.z = q.z();
  rel_pose_msg.pose.pose.orientation.w = q.w();
  // Fill the covariance (row-major order)
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      rel_pose_msg.pose.covariance[i * 6 + j] = covariance(i, j);
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