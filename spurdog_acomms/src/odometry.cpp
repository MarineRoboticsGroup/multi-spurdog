// gyro_preintegrator.cpp
// This is a preintegration node which uses just the gyro measurements and orientation, not acceleration.
// The velocity is provided by the DVL, assumed constant, or zero
// Really just a well-informed dead reckoning node
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include "odometry.hpp"
#include "spurdog_acomms/PreintegrateImu.h"
#include "spurdog_acomms/Bar30SoundSpeed.h"
#include <deque>
#include <mutex>
#include <memory>
#include <fstream>
#include <csignal>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/ImuBias.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <Eigen/Eigenvalues>

ImuPreintegratorNode* g_node_ptr = nullptr;

ImuPreintegratorNode::ImuPreintegratorNode() {
  ros::NodeHandle private_nh_("~");
  std::string resolved_ns = ros::this_node::getNamespace();  // e.g., "/actor_0"
  ros::NodeHandle actor_ns(resolved_ns);  // Explicit NodeHandle for /actor_0
  imu_sub_ = actor_ns.subscribe("cv7_ahrs", 10, &ImuPreintegratorNode::imuCallback, this);
  nav_state_sub_ = actor_ns.subscribe("nav_state", 10, &ImuPreintegratorNode::navStateCallback, this);
  in_water_sub_ = actor_ns.subscribe("in_water", 10, &ImuPreintegratorNode::inWaterCallback, this);
  preint_srv_ = actor_ns.advertiseService("preintegrate_imu", &ImuPreintegratorNode::handlePreintegrate, this);
  dr_state_pub_ = actor_ns.advertise<geometry_msgs::PoseWithCovarianceStamped>("integrated_state", 10);
  // Load the gyro noise parameters from the launch file
  double gyro_noise_sigma, gyro_bias_rw_sigma;
  private_nh_.param("gyro_noise_sigma", gyro_noise_sigma, 4.12e-5); // 4.12e-5
  private_nh_.param("gyro_bias_rw_sigma", gyro_bias_rw_sigma, 4.07e-5); // 4.07e-5
  gyro_noise_ = gtsam::Matrix3::Identity() * std::pow(gyro_noise_sigma, 2);

  // Load the gyro bias values from the launch file
  double gyro_bias_x, gyro_bias_y, gyro_bias_z;
  private_nh_.param("gyro_bias_x", gyro_bias_x, 0.0);
  private_nh_.param("gyro_bias_y", gyro_bias_y, 0.0);
  private_nh_.param("gyro_bias_z", gyro_bias_z, 0.0);
  gyro_bias_ = gtsam::Vector3(gyro_bias_x, gyro_bias_y, gyro_bias_z);

  // Set the velocity source
  std::string velocity_source;
  private_nh_.param("velocity_source", velocity_source, std::string("constant"));
  if (velocity_source == "dvl") {
    dvl_vel_sub_ = actor_ns.subscribe("dvl_pdx", 10, &ImuPreintegratorNode::dvlVelCallback, this);
    gtsam::Vector3 dvl_noise_sigmas(0.5, 0.5, 0.5); // This is a guess
    dvl_noise_model_ = dvl_noise_sigmas.cwiseProduct(dvl_noise_sigmas).asDiagonal();
  } else {
    ROS_WARN("Velocity source not set to DVL, using default constant velocity");
  }
  constant_vel_model_ = gtsam::Vector3(0.8, 0.0, 0.0); // Default to typical velocity
  gtsam::Vector3 constant_vel_noise_sigmas(0.5, 0.25, 0.1); // Cautiously set to 5x DVL
  constant_vel_noise_model_ = constant_vel_noise_sigmas.cwiseProduct(constant_vel_noise_sigmas).asDiagonal();
  // Initialize the buffers
  ti_ = ros::Time(0); // Initialize the time to zero
  in_water_ = false; // Initialize the in-water status
  imu_buffer_ = std::deque<sensor_msgs::Imu>();
  vel_b_buffer_ = std::deque<geometry_msgs::TwistWithCovarianceStamped>();
  dr_state_and_cov_ = std::make_tuple(ros::Time(0), gtsam::Pose3(), gtsam::Matrix6::Identity().eval());
  R_ned_ = gtsam::Rot3(); // Initialize the NED rotation matrix
  Omega_ned_ = gtsam::Vector3(0.0, 0.0, 0.0); // Initialize the Omega vector
  ros::Duration(1.0).sleep(); // Wait  to allow for initial setup
}

// imuCallback
void ImuPreintegratorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push_back(*msg);
  if (ti_ == ros::Time(0)) {
    ti_ = msg->header.stamp;
    ROS_WARN("First IMU message received at time: %f", ti_.toSec());
  }
  // Update the R_ned_ with the current IMU orientation (NED frame)
  // Normalize the quaternion to avoid numerical issues
  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  q.normalize();
  R_ned_ = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
  // ROtate Rned 180deg around the X axis only
  // gtsam::Rot3 R_x180 = gtsam::Rot3::Rx(M_PI); // Rotate 180deg around X axis
  // R_ned_ = R_x180 * R_ned_; // Apply the rotation to the NED rotation
  // Rotate Rned 180deg around the X axis only
  Omega_ned_ = gtsam::Vector3(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  // If we're using constant vel, we need to add the tilt-compenstated velocity to the vel buffer
  if (vel_b_buffer_.empty()) { // A proxy for a lack of DVL or a DVL failure
    // Build a rotation from the IMU orientation in NED to the world ENU frame
    gtsam::Rot3 R_ned_to_enu((gtsam::Matrix3() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished());
    // gtsam::Rot3 R_x180((gtsam::Matrix3() << 1, 0, 0, 0, -1, 0, 0, 0, -1).finished());
    gtsam::Rot3 R_enu = R_ned_to_enu * R_ned_; // Convert from NED to ENU
    // Apply the static rotation to the constant velocity model
    // gtsam::Vector3 vel_b_enu = R_ned_to_enu.rotate(constant_vel_model_);
    // gtsam::Matrix3 vel_noise_b_enu = R_ned_to_enu.matrix() * constant_vel_noise_model_ * R_ned_to_enu.matrix().transpose();
    gtsam::Vector3 vel_b_enu = constant_vel_model_;
    gtsam::Matrix3 vel_noise_b_enu = constant_vel_noise_model_;
    // Apply a dynamic rotation to the constant velocity model
    gtsam::Vector3 vel_w_enu = R_enu.rotate(constant_vel_model_);
    gtsam::Matrix3 vel_noise_w_enu = R_enu.matrix() * constant_vel_noise_model_ * R_enu.matrix().transpose();
    // Create a TwistWithCovarianceStamped message
    geometry_msgs::TwistWithCovarianceStamped vel_b_msg;
    vel_b_msg.header = msg->header; // Copy the header
    vel_b_msg.twist.twist.linear.x = vel_b_enu.x();
    vel_b_msg.twist.twist.linear.y = vel_b_enu.y();
    vel_b_msg.twist.twist.linear.z = vel_b_enu.z();
    // Set the covariance
    gtsam::Matrix6 vel_b_covariance = gtsam::Matrix6::Zero();
    vel_b_covariance.block<3, 3>(0, 0) = vel_noise_b_enu; // Set the linear velocity covariance
    for (size_t i = 0; i < 36; ++i) {
      vel_b_msg.twist.covariance[i] = vel_b_covariance(i / 6, i % 6);
    }
    vel_b_buffer_.push_back(vel_b_msg);
    // Create a TwistWithCovarianceStamped message
    geometry_msgs::TwistWithCovarianceStamped vel_w_msg;
    vel_w_msg.header = msg->header; // Copy the header
    vel_w_msg.twist.twist.linear.x = vel_w_enu.x();
    vel_w_msg.twist.twist.linear.y = vel_w_enu.y();
    vel_w_msg.twist.twist.linear.z = vel_w_enu.z();
    // Set the covariance
    gtsam::Matrix6 vel_w_covariance = gtsam::Matrix6::Zero();
    vel_w_covariance.block<3, 3>(0, 0) = vel_noise_w_enu; // Set the linear velocity covariance
    for (size_t i = 0; i < 36; ++i) {
      vel_w_msg.twist.covariance[i] = vel_w_covariance(i / 6, i % 6);
    }
    vel_w_buffer_.push_back(vel_w_msg);
  }
}

// navStateCallback
void ImuPreintegratorNode::navStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Update the initial state and covariance based on the nav state
  ros::Time t = std::get<0>(dr_state_and_cov_);
  gtsam::Pose3 pose = std::get<1>(dr_state_and_cov_);
  gtsam::Matrix6 cov = std::get<2>(dr_state_and_cov_);
  // Get the pose from the message
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  q.normalize(); // Normalize the quaternion to avoid numerical issues
  // Build a rotation
  gtsam::Rot3 R_ned = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
  gtsam::Pose3 reported_pose = gtsam::Pose3(
      R_ned,
      gtsam::Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  // Rotate the reported pose from Vehicles' quasi-NED to ENU
  reported_pose = convertVehicleNEDToVehicleENU(reported_pose);
  if (t == ros::Time(0) &&
      pose.equals(gtsam::Pose3()) &&  // Pose3 provides an equals() method
      cov.isApprox(gtsam::Matrix6::Identity())) {
    // Initialize the initial state and covariance
    gtsam::Vector6 initial_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
    gtsam::Matrix6 initial_cov = initial_cov_sigmas.cwiseProduct(initial_cov_sigmas).asDiagonal();
    dr_state_and_cov_ = std::make_tuple(msg->header.stamp, reported_pose, initial_cov);
  }
  last_nav_report_ = std::make_pair(msg->header.stamp, reported_pose);
}

// dvlVelCallback
void ImuPreintegratorNode::dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // Rotate the DVL from body to world frame using the current orientation
  gtsam::Vector3 dvl_velocity(
      msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  gtsam::Vector3 dvl_offset(1,0,0);
  gtsam::Vector3 omega_correction = Omega_ned_.cross(dvl_offset); // Cross product to get the correction
  dvl_velocity -= omega_correction; // Apply the correction to the DVL velocity
  // Convert DVL from Body to World ENU frame
  //gtsam::Rot3 R_ned_to_enu = convertRotNEDtoENU(R_ned_); // Convert from NED to ENU
  gtsam::Rot3 R_ned_to_enu((gtsam::Matrix3() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished());
  gtsam::Rot3 R_enu = R_ned_to_enu * R_ned_; // Convert from NED to ENU
  // gtsam::Vector3 dvl_vel_b_enu = R_ned_to_enu.rotate(dvl_velocity); // Rotate the velocity
  // gtsam::Vector3 dvl_vel_w_enu = R_enu.rotate(dvl_velocity); // Rotate the velocity to world frame
  gtsam::Vector3 dvl_vel_b_enu = dvl_velocity; // Use the DVL velocity as is
  gtsam::Matrix3 dvl_noise_b_enu = dvl_noise_model_;
  // Rotate the DVL noise model from NED to ENU
  gtsam::Vector3 dvl_vel_w_enu = R_enu.rotate(dvl_velocity); // Rotate the velocity to world frame
  gtsam::Matrix3 dvl_noise_w_enu = R_enu.matrix() * dvl_noise_model_ * R_enu.matrix().transpose();
  // Create a TwistWithCovarianceStamped message
  geometry_msgs::TwistWithCovarianceStamped dvl_twist_b_enu;
  dvl_twist_b_enu.header = msg->header; // Copy the header
  dvl_twist_b_enu.twist.twist.linear.x = dvl_vel_b_enu.x();
  dvl_twist_b_enu.twist.twist.linear.y = dvl_vel_b_enu.y();
  dvl_twist_b_enu.twist.twist.linear.z = dvl_vel_b_enu.z();
  // Set the covariance (a float64[36] array)
  gtsam::Matrix6 vel_b_covariance = gtsam::Matrix6::Zero();
  vel_b_covariance.block<3, 3>(0, 0) = dvl_noise_b_enu; // Set the linear velocity covariance
  for (size_t i = 0; i < 36; ++i) {
    dvl_twist_b_enu.twist.covariance[i] = vel_b_covariance(i / 6, i % 6);
  }
  vel_b_buffer_.push_back(dvl_twist_b_enu);
  // Create a TwistWithCovarianceStamped message for the world frame
  geometry_msgs::TwistWithCovarianceStamped dvl_twist_w_enu;
  dvl_twist_w_enu.header = msg->header; // Copy the header
  dvl_twist_w_enu.twist.twist.linear.x = dvl_vel_w_enu.x();
  dvl_twist_w_enu.twist.twist.linear.y = dvl_vel_w_enu.y();
  dvl_twist_w_enu.twist.twist.linear.z = dvl_vel_w_enu.z();
  // Set the covariance (a float64[36] array)
  gtsam::Matrix6 vel_w_covariance = gtsam::Matrix6::Zero();
  vel_w_covariance.block<3, 3>(0, 0) = dvl_noise_w_enu; // Set the linear velocity covariance
  for (size_t i = 0; i < 36; ++i) {
    dvl_twist_w_enu.twist.covariance[i] = vel_w_covariance(i / 6, i % 6);
  }
  vel_w_buffer_.push_back(dvl_twist_w_enu);
}

// inWaterCallback
void ImuPreintegratorNode::inWaterCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!in_water_) {
    imu_buffer_.clear(); // Clear the IMU buffer to start fresh
    vel_b_buffer_.clear(); // Clear the DVL velocity buffer as well
    vel_w_buffer_.clear();
    // Set the dr_pose and covariance to the last nav report
    gtsam::Vector6 last_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
    gtsam::Matrix6 last_cov = last_cov_sigmas.cwiseProduct(last_cov_sigmas).asDiagonal();
    // Trial and error suggests that its Initialized Rotation: time 1751097643.3077688 Pitch(-down) 4.999999999999999, Roll(+stbd) 3.0000000000000044, Yaw(+CCW) 59.99999999999999
    // gtsam::Rot3 test_rotation = gtsam::Rot3::RzRyRx(
    //     (M_PI / 180)*2, (M_PI / 180)*30, (M_PI / 180)*60); // Example rotation, adjust as needed
    // Re-write the last_pose_rotation using the test rotation
    //last_nav_report_.second = gtsam::Pose3(test_rotation, last_nav_report_.second.translation());
    // Set the dead reckoning state and covariance
    dr_state_and_cov_ = std::make_tuple(last_nav_report_.first, last_nav_report_.second, last_cov);
    ROS_INFO("Initial state set with x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f",
          last_nav_report_.second.translation().x(), last_nav_report_.second.translation().y(), last_nav_report_.second.translation().z(),
          last_nav_report_.second.rotation().roll(), last_nav_report_.second.rotation().pitch(), last_nav_report_.second.rotation().yaw());

  } else if (!in_water_ && msg->data==true) { // Entering the water
    if (!imu_buffer_.empty()) {
      const sensor_msgs::Imu& last_msg = imu_buffer_.back();
      ti_ = last_msg.header.stamp;
      imu_buffer_.clear(); // Clear the IMU buffer to start fresh
      vel_b_buffer_.clear(); // Clear the DVL velocity buffer as well
      vel_w_buffer_.clear();
      geometry_msgs::PoseWithCovarianceStamped dr_msg;
      dr_msg.header.stamp = ti_;
      dr_msg.header.frame_id = "cv7_ahrs"; // Use the same frame
      dr_msg.pose.pose.position.x = last_nav_report_.second.translation().x();
      dr_msg.pose.pose.position.y = last_nav_report_.second.translation().y();
      dr_msg.pose.pose.position.z = last_nav_report_.second.translation().z();
      // Get the quaternion from the last nav report
      gtsam::Quaternion q = last_nav_report_.second.rotation().toQuaternion();
      dr_msg.pose.pose.orientation.x = q.x();
      dr_msg.pose.pose.orientation.y = q.y();
      dr_msg.pose.pose.orientation.z = q.z();
      dr_msg.pose.pose.orientation.w = q.w();
      // Set the covariance (a float64[36] array)
      // use a default 1.5m position sigma and 0.5 rad orientation sigma
      gtsam::Vector6 last_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
      gtsam::Matrix6 last_cov = last_cov_sigmas.cwiseProduct(last_cov_sigmas).asDiagonal();
      for (size_t i = 0; i < 36; ++i) {
        dr_msg.pose.covariance[i] = last_cov(i / 6, i % 6);
      }
      dr_state_pub_.publish(dr_msg);
      ROS_WARN("Entering water, reset preintegration start time to %f", ti_.toSec());
    }else {
      ROS_WARN("IMU buffer is empty, cannot reset preintegration start time");
    }
  } else {
    // IN the water, do nothing
  }
  in_water_ = msg->data; // Update the in-water status
}

//TODO: Improve this as a general covariance preconditioning function
gtsam::Matrix6 ImuPreintegratorNode::makePSD(const gtsam::Matrix6& input) {
    // Step 1: Force symmetry
    gtsam::Matrix6 sym = 0.5 * (input + input.transpose());

    // Step 2: Eigen decomposition
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eigensolver(sym);
    if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error("Eigen decomposition failed in makePSD");
    }

    // Step 3: Clamp eigenvalues to a minimum threshold
    Eigen::Matrix<double, 6, 1> eigvals = eigensolver.eigenvalues();
    gtsam::Matrix6 eigvecs = eigensolver.eigenvectors();
    double min_eigval = 1e-9;
    Eigen::Matrix<double, 6, 1> clamped = eigvals.cwiseMax(min_eigval);

    // Step 4: Reconstruct matrix
    gtsam::Matrix6 psd = eigvecs * clamped.asDiagonal() * eigvecs.transpose();

    return psd;
}

// Frame Conversion
gtsam::Rot3 ImuPreintegratorNode::convertRotNEDtoENU(const gtsam::Rot3& R_ned) {
    // Convert a rotation from NED to ENU frame
    gtsam::Rot3 R_x180 = gtsam::Rot3::Rx(M_PI); // Rotate 180deg around X axis
    gtsam::Rot3 R_z90 = gtsam::Rot3::Rz(M_PI/2); // Rotate by 90deg around Z axis
    gtsam::Rot3 R_ned_enu = R_z90 * R_x180; // NED to ENU rotation
    gtsam::Rot3 R_enu = R_ned_enu * R_ned; // Convert from NED to ENU
    return R_enu; // Return the rotation in ENU
}

gtsam::Matrix3 ImuPreintegratorNode::convertCovNEDToENU(const gtsam::Rot3& R_ned, gtsam::Matrix3& cov_ned) {
    // Convert a covariance matrix from NED to ENU frame
    gtsam::Rot3 R_enu = convertRotNEDtoENU(R_ned); // Get the rotation from NED to ENU
    gtsam::Matrix3 cov_enu = R_enu.matrix() * cov_ned * R_enu.matrix().transpose(); // Rotate the covariance
    return cov_enu; // Return the covariance in ENU
}

gtsam::Pose3 ImuPreintegratorNode::convertVehicleNEDToVehicleENU(const gtsam::Pose3& T_ned) {
    // Extract the rotation and translation from the NED pose
    gtsam::Rot3 R_body_ned = T_ned.rotation();
    gtsam::Rot3 R_ned_body = R_body_ned.inverse(); // Get the inverse rotation (from NED to body)
    gtsam::Point3 t_ned = T_ned.translation();
    // Convert the rotation from NED to ENU
    gtsam::Rot3 R_ned_to_enu((gtsam::Matrix3() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished());
    gtsam::Rot3 R_enu = R_ned_to_enu * R_ned_body; // Convert the rotation from NED to ENU
    // Then rotate the translation by 90deg around the Z axis CCW
    //gtsam::Rot3 R_dec = gtsam::Rot3::Rz((M_PI/180)*14); // Rotate by 90 degrees around Z axis
    //gtsam::Rot3 R_enu_new = R_dec * R_enu; // Apply the rotation to the body rotation
    // Rotate the frame by 180deg around the X axis
    //gtsam::Rot3 R_x180 = gtsam::Rot3::Rx(M_PI); // Rotate 180deg around X axis
    //R_enu_new = R_x180 * R_enu_new; // Apply the rotation to the ENU rotation
    // Now we have
    // Apply the rotation to the translation
    // Convert the translation from NED to ENU
    gtsam::Point3 t_enu(t_ned.x(), t_ned.y(), -t_ned.z()); // Vehicle X, Y are already ENU, Z is flipped
    // Rebuild the pose in ENU
    gtsam::Pose3 T_enu(R_enu, t_enu);
    return T_enu;
}

//getPreintegratedRotation
std::tuple<double, gtsam::Rot3, gtsam::Matrix3> ImuPreintegratorNode::getPreintegratedRotation(
    const ros::Time& ti, const ros::Time& tj) {
  // Check if the IMU buffer is empty
  if (imu_buffer_.empty()) {
    ROS_WARN("IMU buffer is empty, cannot preintegrate rotation");
    return std::make_tuple(0.0, gtsam::Rot3::Identity(), gtsam::Matrix3::Zero());
  }
  // Create the preintegration parameters
  auto preint_rotation_params = boost::make_shared<gtsam::PreintegratedRotationParams>(
      gyro_noise_, gyro_bias_);
  // Create the preintegrated rotation object
  gtsam::PreintegratedRotation preint_rotation(preint_rotation_params);
  // Iterate through the IMU buffer and integrate the measurements
  ros::Time last_imu_time = ti;
  gtsam::Matrix3 SigmaRij_body = gtsam::Matrix3::Zero();
  gtsam::Rot3 Ri_wb; // Initial rotation in world frame
  bool initialized = false;
  double count = 0.0; // Initialize mean dt
  // While imu buffer is not empty
  while (!imu_buffer_.empty()) {
    const auto& imu_msg = imu_buffer_.front();
    ros::Time t = imu_msg.header.stamp;
    if (t < ti) {
      ROS_WARN("IMU message at time %f is before the initial time %f, ignoring", t.toSec(), ti.toSec());
      imu_buffer_.pop_front(); // Remove this message
      continue; // Skip this message
    } else if (t > tj){
      break; // Stop integrating if we exceed the final time
    } else {
      if (!initialized) {
        last_imu_time = t;
        initialized = true;
        imu_buffer_.pop_front(); // Skip the first one (no dt)
        continue;
      }
      double dt = (t - last_imu_time).toSec();
      if (dt <= 0.0) {
        ROS_WARN("Non-positive dt (%f), skipping IMU message", dt);
        imu_buffer_.pop_front();
        continue;
      }
      //gtsam::Rot3 R_ned_to_enu((gtsam::Matrix3() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished());
      gtsam::Rot3 R_ned_to_enu = gtsam::Rot3::Identity(); // Identity rotation for NED to ENU
      gtsam::Vector3 omega_ned(imu_msg.angular_velocity.x,
                           imu_msg.angular_velocity.y,
                           imu_msg.angular_velocity.z);
      // Convert the angular velocity from NED to ENU
      gtsam::Vector3 omega_enu = R_ned_to_enu.rotate(omega_ned); // Rotate the angular velocity
      gtsam::Vector3 omega_bias_enu = R_ned_to_enu.rotate(gyro_bias_); // Rotate the gyro bias
      // If this is the
      preint_rotation.integrateMeasurement(omega_enu, omega_bias_enu, dt);
      count += 1;
      last_imu_time = t;
      imu_buffer_.pop_front();
    }
  }
  // Get the integrated rotation and covariance
  double deltaTimeij = preint_rotation.deltaTij();
  double mean_dt = count > 0 ? deltaTimeij / count : 0.0; // Calculate the mean dt
  //gtsam::Rot3 R_ned_to_enu((gtsam::Matrix3() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished());
  gtsam::Rot3 R_ned_to_enu = gtsam::Rot3::Identity(); // Identity rotation for NED to ENU
  gtsam::Matrix3 H; // H is dR/db
  gtsam::Rot3 deltaRij_body = preint_rotation.biascorrectedDeltaRij(gyro_bias_, &H);
  // Get the roll, pitch, yaw angles (in radians)
  gtsam::Vector3 rpy = deltaRij_body.rpy();  // [roll, pitch, yaw]

  // Flip the yaw angle (negate it)
  // double flipped_yaw = -rpy(2);  // rpy(2) is yaw
  // double roll = rpy(0);
  // double pitch = rpy(1);

  // // Reconstruct the Rot3 with flipped yaw
  // deltaRij_body = gtsam::Rot3::RzRyRx(roll, pitch, flipped_yaw);
  gtsam::Matrix3 gyro_noise_hz = gyro_noise_ * mean_dt; // Convert to rad^2/s^2
  // Convert the covariance from NED to ENU
  gtsam::Matrix3 gyro_noise_hz_enu = R_ned_to_enu.matrix() * gyro_noise_hz * R_ned_to_enu.matrix().transpose();
  SigmaRij_body = H * gyro_noise_hz * H.transpose(); // in ENU? (measurements are in ENU)
  preint_rotation.resetIntegration();
  return std::make_tuple(deltaTimeij, deltaRij_body, SigmaRij_body);
}

// getPreintegratedTranslation
std::tuple<double, gtsam::Point3, gtsam::Matrix3, gtsam::Matrix3> ImuPreintegratorNode::getPreintegratedTranslation(
    const ros::Time& imu_ti, const ros::Time& imu_tj) {
  // Check if the velocity buffer is empty
  if (vel_b_buffer_.empty()) {
    ROS_WARN("Velocity buffer is empty, cannot preintegrate translation");
    return std::make_tuple(0.0, gtsam::Point3(0.0, 0.0, 0.0), gtsam::Matrix3::Zero(),gtsam::Matrix3::Zero());
  }
  // Initialize the translation and covariance
  gtsam::Point3 deltatij_body(0.0, 0.0, 0.0);
  gtsam::Matrix3 Sigmatij_body = gtsam::Matrix3::Zero();
  //gtsam::Matrix3 deltaTransRotCovij = gtsam::Matrix3::Zero(); // This is cross-coupling
  double deltaTimeij = 0.0;
  // Iterate through the velocity buffer and integrate the velocities
  ros::Time last_vel_time = imu_ti;
  bool initialized = false;
  // While velocity buffer is not empty
  while (!vel_b_buffer_.empty()) {
    const auto& vel_msg = vel_b_buffer_.front();
    ros::Time t = vel_msg.header.stamp;
    if (t < imu_ti) {
      ROS_WARN("Velocity message at time %f is before the initial time %f, ignoring", t.toSec(), imu_ti.toSec());
      vel_b_buffer_.pop_front(); // Remove this message
      continue; // Skip this message
    } else if (t > imu_tj) {
      break; // Stop integrating if we exceed the final time
    } else {
      if (!initialized) {
        last_vel_time = t;
        initialized = true;
        vel_b_buffer_.pop_front(); // Skip the first one (no dt)
        continue;
      }
      double dt = (t - last_vel_time).toSec();
      if (dt <= 0.0) {
        ROS_WARN("Non-positive dt (%f), skipping velocity message", dt);
        vel_b_buffer_.pop_front();
        continue;
      }
      // Get the velocity in ENU frame
      gtsam::Vector3 v_enu(vel_msg.twist.twist.linear.x, vel_msg.twist.twist.linear.y, vel_msg.twist.twist.linear.z);
      // Get the delta position in the ENU frame
      gtsam::Point3 delta_pos = gtsam::Point3(v_enu.x() * dt,
                                                     v_enu.y() * dt,
                                                     v_enu.z() * dt);
      // Add the delta position to the total translation
      deltatij_body += delta_pos; // Accumulate the translation
      // Get the covariance of the translation
      //gtsam::Matrix3 vel_noise_enu = convertCovNEDToENU(R_ned_, vel_msg.covariance.block<3, 3>(0, 0)); // Convert the covariance from NED to ENU
      gtsam::Matrix3 vel_covariance = gtsam::Matrix3::Zero();
      // Upack the upper left 3x3 of the msg.twist.covariance into the vel_covariance matrix
      for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
          vel_covariance(i, j) = vel_msg.twist.covariance[i * 6 + j]; // Copy the covariance from the message
        }
      }
      gtsam::Matrix3 delta_pos_cov = vel_covariance * dt * dt; // Covariance of the translation in ENU frame
      // Accumulate the covariance
      if (Sigmatij_body.isZero()) {
        Sigmatij_body = delta_pos_cov; // First measurement, set the covariance
      } else {
        Sigmatij_body += delta_pos_cov; // Accumulate the covariance
      }

      // Update the deltaTij
      deltaTimeij += dt; // Accumulate the time difference
      last_vel_time = t;
      vel_b_buffer_.pop_front(); // Remove the processed velocity message
    }
  }
  gtsam::Matrix3 J_tr_rot; // Jacobian of translation w.r.t. rotation
  // Get the avergae velocity in the ENU frame
  if (deltaTimeij > 0.0) {
    // Compute the average velocity in the ENU frame
    gtsam::Vector3 average_velocity_enu = gtsam::Vector3(deltatij_body.x(),
                                                          deltatij_body.y(),
                                                          deltatij_body.z()) / deltaTimeij;
    // Get the jacobian of the translation w.r.t. the rotation
    J_tr_rot = -gtsam::skewSymmetric(average_velocity_enu); // Jacobian of translation w.r.t. rotation
  } else {
    // If no valid measurements, return zero translation and covariance
    J_tr_rot = gtsam::Matrix3::Zero();
  }
  return std::make_tuple(deltaTimeij, deltatij_body, Sigmatij_body, J_tr_rot);
}

// getPreintegratedPose
std::tuple<double, gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::getPreintegratedPose(
    const std::tuple<double, gtsam::Rot3, gtsam::Matrix3>& preint_rotation,
    const std::tuple<double, gtsam::Point3, gtsam::Matrix3, gtsam::Matrix3>& preint_translation) {
      // Build a preintegrated pose from the rotation and translation
      double deltaTimeij = std::get<0>(preint_rotation);
      gtsam::Rot3 deltaRij_body = std::get<1>(preint_rotation);
      gtsam::Matrix3 SigmaRij_body = std::get<2>(preint_rotation);
      gtsam::Point3 deltatij_body = std::get<1>(preint_translation);
      gtsam::Matrix3 Sigmatij_body = std::get<2>(preint_translation);
      gtsam::Matrix3 J_tr_rot = std::get<3>(preint_translation); // Jacobian of translation w.r.t. rotation
      // Build the pose in the world, enu frame
      //gtsam::Rot3 R_x = gtsam::Rot3::Rx(M_PI); // Rotate 180deg around X axis
      // gtsam::Rot3 R_z = gtsam::Rot3::Rz(M_PI); // Rotate by 90deg around Z axis
      //deltaRij_body = R_corr * deltaRij_body; // Apply the rotation to the deltaRij
      // ROtate the translation by R_corr
      //deltatij_body = R_corr.rotate(deltatij_body); // Rotate the translation
      gtsam::Pose3 Tij_body = gtsam::Pose3(deltaRij_body, deltatij_body); // Pose in body frame
      //Tij_body = Tij_body.compose(gtsam::Pose3(R_x, gtsam::Point3(0, 0, 0))); // Apply the rotation to the pose
      // Convert the pose from body to world frame
      // // Apply the rotation to the translation skew
      // gtsam::Matrix3 J_tr_rot = -gtsam::skewSymmetric(deltaTransij); // Jacobian of translation w.r.t. rotation
      // gtsam::Matrix3 CovRt = deltaRotCovij * J_tr_rot.transpose(); // Covariance of the rotation
      // gtsam::Matrix3 CovTr = CovRt.transpose(); // Cross-covariance of translation and rotation
      // Ensure the covariance matrices are positive semi-definite
      // Build the complete preintegrated covariance
      gtsam::Matrix3 SigmaTrRot = J_tr_rot * SigmaRij_body * J_tr_rot.transpose(); // Cross-covariance of translation and rotation
      gtsam::Matrix6 SigmaTij_body = gtsam::Matrix6::Zero();
      SigmaTij_body.block<3, 3>(0, 0) = SigmaRij_body; // Covariance of the rotation
      // SigmaTij_body.block<3, 3>(0, 3) = gtsam::Matrix3::Zero(); // Cross-covariance of translation and rotation
      // SigmaTij_body.block<3, 3>(3, 0) = gtsam::Matrix3::Zero(); // Cross-covariance of translation and rotation
      SigmaTij_body.block<3, 3>(0, 3) = SigmaTrRot.transpose(); // Cross-covariance of translation and rotation
      SigmaTij_body.block<3, 3>(3, 0) = SigmaTrRot; // Cross-covariance of translation and rotation
      SigmaTij_body.block<3, 3>(3, 3) = Sigmatij_body; // Covariance of the translation
      // Return the output as a tuple
      return std::make_tuple(deltaTimeij, Tij_body, SigmaTij_body);
}

// Dead reckoning from preintegrated state
std::pair<gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::deadReckonFromPreintegrate(
    const ros::Time& initial_time, const ros::Time& final_time, const gtsam::Pose3& Ti_w, const gtsam::Matrix6& SigmaTi_w) {
  // Iterate over the vel_w_buffer, integrating the velocities with respect to the initial time
  if (vel_w_buffer_.empty()) {
    ROS_WARN("Velocity buffer is empty, cannot dead reckon from preintegrated state");
    return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
  }
  // Create the preintegration parameters
  double deltaTimeij = 0.0;
  gtsam::Pose3 Tj_w = Ti_w; // Initialize the pose in world frame
  gtsam::Matrix6 SigmaTj_w = SigmaTi_w; // Initialize the covariance in world frame
  ros::Time last_vel_time = initial_time;
  bool initialized = false;
  while (!vel_w_buffer_.empty()) {
    const auto& vel_msg = vel_w_buffer_.front();
    ros::Time t = vel_msg.header.stamp;
    if (t < initial_time) {
      ROS_WARN("Velocity message at time %f is before the initial time %f, ignoring", t.toSec(), initial_time.toSec());
      vel_w_buffer_.pop_front(); // Remove this message
      continue; // Skip this message
    } else if (t > final_time) {
      break; // Stop integrating if we exceed the final time
    } else {
      if (!initialized) {
        last_vel_time = t;
        initialized = true;
        vel_w_buffer_.pop_front(); // Skip the first one (no dt)
        continue;
      }
      double dt = (t - last_vel_time).toSec();
      if (dt <= 0.0) {
        ROS_WARN("Non-positive dt (%f), skipping velocity message", dt);
        vel_w_buffer_.pop_front();
        continue;
      }
      gtsam::Vector3 v_enu(vel_msg.twist.twist.linear.x, vel_msg.twist.twist.linear.y, vel_msg.twist.twist.linear.z);
      // Get the delta position in the ENU frame
      gtsam::Point3 delta_pos = gtsam::Point3(v_enu.x() * dt, v_enu.y() * dt, v_enu.z() * dt);
      // Get a rotation matrix representing the orientation of the velocity vector in the world frame
      // gtsam::Vector3 forward = gtsam::Vector3(v_enu.x(), v_enu.y(), v_enu.z()).normalized(); // Normalize the velocity vector
      // double az = std::atan2(forward.y(), forward.x()); // Azimuth angle in radians
      // double el = std::atan2(forward.z(), std::sqrt(forward.x() * forward.x() + forward.y() * forward.y())); // Elevation angle in radians
      // gtsam::Rot3 R_yaw = gtsam::Rot3::Rz(az); // Rotation around Z axis (yaw)
      // gtsam::Rot3 R_pitch = gtsam::Rot3::Ry(el); // Rotation around Y axis (pitch)
      // gtsam::Rot3 R_vel_w = R_yaw * R_pitch; // Combined rotation for the velocity vector
      // Get the current orientation in the world frame
      gtsam::Rot3 R_ned_to_enu((gtsam::Matrix3() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished());
      // Convert the current orientation from NED to ENU
      gtsam::Rot3 R_enu_w = R_ned_to_enu * R_ned_;
      // Rotate the velocity vector
      // Compute the orientation of the
      // Create the rotation matrix from the
      //gtsam::Rot3 R_vel_w = gtsam::Rot3(x_positive_enu, y_positive, forward); // Rotation from ENU to velocity frame
      // Add the delta position to the total translation
      gtsam::Point3 tj_w = Tj_w.translation() + delta_pos; // Accumulate the translation
      gtsam::Pose3 current_nav_state = last_nav_report_.second; // Get the last reported state
      tj_w.z() = current_nav_state.translation().z(); // Reset the depth to the last reported:
      Tj_w = gtsam::Pose3(R_enu_w, tj_w); // Update the pose in world frame
      gtsam::Matrix3 vel_covariance = gtsam::Matrix3::Zero();
      // Upack the upper left 3x3 of the msg.twist.covariance into the vel_covariance matrix
      for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
          vel_covariance(i, j) = vel_msg.twist.covariance[i * 6 + j]; // Copy the covariance from the message
        }
      }
      gtsam::Matrix3 deltaTransCovij_enu = vel_covariance * dt * dt;
      gtsam::Matrix3 deltaRotCovij = 4 * gyro_noise_ * dt * dt; // Covariance of the rotation in ENU frame
      // Update the covariance of the pose rotation
      SigmaTj_w.block<3, 3>(0, 0) += deltaRotCovij; // Covariance of the rotation
      SigmaTj_w.block<3, 3>(0, 3) += gtsam::Matrix3::Zero(); // Cross-covariance of translation and rotation
      SigmaTj_w.block<3, 3>(3, 0) += gtsam::Matrix3::Zero(); // Cross-covariance of translation and rotation
      SigmaTj_w.block<3, 3>(3, 3) += deltaTransCovij_enu; // Covariance of the translation
      // Ensure the covariance is positive semi-definite
      // Get the covariance of the translation
      // Update the deltaTij
      deltaTimeij += dt; // Accumulate the time difference
      last_vel_time = t;
      vel_w_buffer_.pop_front(); // Remove the processed velocity message
    }
  }
  geometry_msgs::PoseWithCovarianceStamped dr_msg;
  dr_msg.header.stamp = final_time;
  dr_msg.header.frame_id = "world_enu"; // Use the same frame
  dr_msg.pose.pose.position.x = Tj_w.x();
  dr_msg.pose.pose.position.y = Tj_w.y();
  dr_msg.pose.pose.position.z = Tj_w.z();
  // Get the quaternion orientation from the NavState
  gtsam::Quaternion q = Tj_w.rotation().toQuaternion();
  dr_msg.pose.pose.orientation.w = q.w();
  dr_msg.pose.pose.orientation.x = q.x();
  dr_msg.pose.pose.orientation.y = q.y();
  dr_msg.pose.pose.orientation.z = q.z();
  // Fill the covariance
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      int src_i = (i < 3) ? i + 3 : i - 3;  // Swap translation (i<3) and rotation (i>=3)
      int src_j = (j < 3) ? j + 3 : j - 3;
      dr_msg.pose.covariance[i * 6 + j] = SigmaTj_w(src_i, src_j);
    }
  }
  // Publish the dead reckoned pose
  dr_state_pub_.publish(dr_msg);
  // Update DR state and covariance
  dr_state_and_cov_ = std::make_tuple(final_time, Tj_w, SigmaTj_w);
  dead_reckon_map_[final_time] = Tj_w;
  return std::make_pair(Tj_w, SigmaTj_w);
}

// handlePreintegrate
bool ImuPreintegratorNode::handlePreintegrate(
    spurdog_acomms::PreintegrateImu::Request &req,
    spurdog_acomms::PreintegrateImu::Response &res) {
  // Call getRelativePoseBetweenStates to get the relative pose and covariance
  auto preint_rotation = getPreintegratedRotation(req.initial_time, req.final_time);
  auto preint_translation = getPreintegratedTranslation(req.initial_time, req.final_time);
  auto preint_pose = getPreintegratedPose(preint_rotation, preint_translation);
  double deltaTimeij = std::get<0>(preint_pose);
  gtsam::Pose3 Tij_body = std::get<1>(preint_pose);
  gtsam::Matrix6 SigmaTij_body = std::get<2>(preint_pose);
  // Log the pose delta in terms of x,y,z and r,p,y (degrees)
  ROS_INFO("Preintegrated Pose Delta over %.2fs: [x: %.3f, y: %.3f, z: %.3f,]",
    deltaTimeij, Tij_body.x(), Tij_body.y(), Tij_body.z());
  //gtsam::Matrix6 Sigmaij_world = SigmaTij_body; //
  //gtsam::Matrix6 Sigmaj_world = Sigmai_world + Sigmaij_world; // Assuming no cross-coupling for now
  // Compute the Tj_world
  //gtsam::Pose3 Tj_world = Ti_world.compose(Tij_world, gtsam::OptionalJacobian<6, 6>(), gtsam::OptionalJacobian<6, 6>());
  // Compute the Tij_body (referenced to the body frame Ti)
  // Compute the relative pose in the body frame of Ti
  // gtsam::Matrix66 Ji, Jj;
  // gtsam::Pose3 Tij_body = Ti_world.transformPoseTo(Tj_world, Ji, Jj);
  // gtsam::Matrix6 Sigmaij_body = Ji * Sigmai_world * Ji.transpose() +
  //                             Jj * Sigmaj_world * Jj.transpose();
  // Log the covariance
  // std::stringstream ssCov;
  // ssCov << "Preintegrated Covariance:\n" << relativeCov;
  // ROS_INFO_STREAM(ssCov.str());
  // Fill the response message (a PoseWithCovarianceStamped)
  res.pose_delta.header.stamp = req.final_time;
  res.pose_delta.header.frame_id = "cv7_ahrs";
  res.pose_delta.pose.pose.position.x = Tij_body.x();
  res.pose_delta.pose.pose.position.y = Tij_body.y();
  res.pose_delta.pose.pose.position.z = Tij_body.z();
  // Get the quaternion orientation from the NavState
  gtsam::Quaternion q = Tij_body.rotation().toQuaternion();
  res.pose_delta.pose.pose.orientation.w = q.w();
  res.pose_delta.pose.pose.orientation.x = q.x();
  res.pose_delta.pose.pose.orientation.y = q.y();
  res.pose_delta.pose.pose.orientation.z = q.z();
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      int src_i = (i < 3) ? i + 3 : i - 3;  // Swap translation (i<3) and rotation (i>=3)
      int src_j = (j < 3) ? j + 3 : j - 3;
      res.pose_delta.pose.covariance[i * 6 + j] = SigmaTij_body(src_i, src_j);
    }
  }
  // Dead reckon from the preintegrated state
  gtsam::Pose3 Ti_w = std::get<1>(dr_state_and_cov_); // Get the initial pose in world frame
  gtsam::Matrix6 SigmaTi_w = std::get<2>(dr_state_and_cov_); // Get the initial covariance in world frame
  // Call deadReckonFrom
  auto dr_result = deadReckonFromPreintegrate(req.initial_time, req.final_time, Ti_w, SigmaTi_w);
  gtsam::Pose3 dr_pose = dr_result.first;
  gtsam::Matrix6 dr_cov = dr_result.second;
  ROS_INFO("Dead Reckoned Pose at time %f: [x: %.3f, y: %.3f, z: %.3f]",
           req.final_time.toSec(), dr_pose.x(), dr_pose.y(), dr_pose.z());
  return true;
}

// Write pose results to a TUM file
void ImuPreintegratorNode::writePoseResultsToTum(const std::string& filename) {
  std::ofstream file(filename + "dr_poses.tum");
  if (!file.is_open()) {
    ROS_ERROR("Failed to open file %s for writing", (filename + "dr_poses.tum").c_str());
    return;
  }
  for (const auto& entry : dead_reckon_map_) {
    const ros::Time& t = entry.first;
    const gtsam::Pose3& pose = entry.second;
    gtsam::Quaternion q = pose.rotation().toQuaternion();
    file << std::fixed << std::setprecision(6)
         << t.toSec() << " "
         << pose.x() << " "
         << pose.y() << " "
         << pose.z() << " "
         << q.w() << " "
         << q.x() << " "
         << q.y() << " "
         << q.z()
         << std::endl;
  }
  ROS_INFO("Pose results written to %s", (filename + "dr_poses.tum").c_str());
  file.close();
  // Also write the nav state map to a file
  std::ofstream nav_file(filename + "nav_states.tum");
  if (!nav_file.is_open()) {
    ROS_ERROR("Failed to open file %s for writing", (filename + "nav_states.tum").c_str());
    return;
  }
  for (const auto& entry : nav_state_map_) {
    const ros::Time& t = entry.first;
    const gtsam::Pose3& pose = entry.second;
    gtsam::Quaternion q = pose.rotation().toQuaternion();
    nav_file << std::fixed << std::setprecision(6)
             << t.toSec() << " "
             << pose.x() << " "
             << pose.y() << " "
             << pose.z() << " "
             << q.w() << " "
             << q.x() << " "
             << q.y() << " "
             << q.z()
             << std::endl;
  }
  ROS_INFO("Nav state results written to %s", (filename + "nav_states.tum").c_str());
  nav_file.close();
}

void onShutdownCallback(int sig) {
  ROS_INFO("Shutdown signal received.");

  if (g_node_ptr != nullptr) {
    std::string filename = "/home/morrisjp/bags/June/odom_test/";
    std::string rosname = ros::this_node::getName(); // Get the node name
    // Split the filename by slashes to avoid issues and replace with underscores
    std::replace(rosname.begin(), rosname.end(), '/', '_');
    // Remove the leading underscore if it exists
    if (!rosname.empty() && rosname[0] == '_') {
      rosname.erase(0, 1);
    }
    // Get the velocity source from the parameter server
    std::string velocity_source;
    ros::NodeHandle private_nh("~");
    private_nh.param("velocity_source", velocity_source, std::string("constant"));
    filename += rosname + "_" + velocity_source + "_";
    filename += std::to_string(ros::Time::now().toSec()) + "_";
    g_node_ptr->writePoseResultsToTum(filename);
    ROS_INFO("Wrote poses to TUM file on shutdown.");
  }

  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_preintegrator_node");
  ImuPreintegratorNode node;
  g_node_ptr = &node;
  //signal(SIGINT, onShutdownCallback);  // Catch Ctrl+C and cleanly shutdown
  ros::spin();
  return 0;
}