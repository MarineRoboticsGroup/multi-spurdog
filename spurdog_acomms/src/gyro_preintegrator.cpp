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
#include <std_msgs/Bool.h>
#include "gyro_preintegrator.hpp"
#include "spurdog_acomms/PreintegrateImu.h"

#include <deque>
#include <mutex>
#include <memory>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


ImuPreintegratorNode::ImuPreintegratorNode() {
  ros::NodeHandle nh_("~");
  imu_sub_ = nh_.subscribe("cv7_ahrs", 10, &ImuPreintegratorNode::imuCallback, this);
  dvl_vel_sub_ = nh_.subscribe("dvl_pdx", 10, &ImuPreintegratorNode::dvlVelCallback, this);
  nav_state_sub_ = nh_.subscribe("nav_state", 10, &ImuPreintegratorNode::navStateCallback, this);
  in_water_sub_ = nh_.subscribe("in_water", 10, &ImuPreintegratorNode::inWaterCallback, this);
  preint_srv_ = nh_.advertiseService("preintegrate_imu", &ImuPreintegratorNode::handlePreintegrate, this);
  // Load the noise parameters from the launch file
  double gyro_noise_sigma, gyro_bias_rw_sigma;
  nh_.param("gyro_noise_sigma", gyro_noise_sigma, 4.12e-5); // 4.12e-5
  nh_.param("gyro_bias_rw_sigma", gyro_bias_rw_sigma, 4.07e-5); // 4.07e-5
  // Set the noise covariance matrix for the gyroscope
  gyro_noise_ = gtsam::Matrix3::Identity() * std::pow(gyro_noise_sigma, 2);
  // print the gyro noise matrix
  // ROS_INFO("Gyro noise matrix: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
  //          gyro_noise_(0, 0), gyro_noise_(0, 1), gyro_noise_(0, 2),
  //          gyro_noise_(1, 0), gyro_noise_(1, 1), gyro_noise_(1, 2),
  //          gyro_noise_(2, 0), gyro_noise_(2, 1), gyro_noise_(2, 2));
  // Load the bias values from the launch file
  double gyro_bias_x, gyro_bias_y, gyro_bias_z;
  nh_.param("gyro_bias_x", gyro_bias_x, 0.0);
  nh_.param("gyro_bias_y", gyro_bias_y, 0.0);
  nh_.param("gyro_bias_z", gyro_bias_z, 0.0);
  bias_ = gtsam::Vector3(gyro_bias_x, gyro_bias_y, gyro_bias_z);
  ti_ = ros::Time(0); // Initialize the time to zero
  start_time_ = ros::Time::now(); // Initialize the start time
  last_nav_report_ = gtsam::Pose3::Identity(); // Initialize the last navigation report to identity
  in_water_ = false; // Initialize the in-water status to false
  dvl_vel_bw_ = gtsam::Vector3(0.0, 0.0, 0.0); // Initialize the DVL velocity to zero
  // Initialize a map of ros::Time to gtsam::NavState
  std::map<ros::Time, gtsam::NavState> nav_state_map_;
  std::map<ros::Time, gtsam::Matrix6> nav_cov_map_;
  // Initialize the IMU buffer
  imu_buffer_ = std::deque<sensor_msgs::Imu>();
  ROS_INFO("ImuPreintegratorNode initialized with gyro noise sigma: %f, gyro bias rw sigma: %f",
           gyro_noise_sigma, gyro_bias_rw_sigma);
  // After 10sec, handle the initial state
  ros::Duration(10.0).sleep(); // Wait for 10 seconds to allow for initial setup
}

// imuCallback
void ImuPreintegratorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push_back(*msg);
  if (ti_ == ros::Time(0)) {
    ti_ = msg->header.stamp;
    ROS_WARN("First IMU message received at time: %f", ti_.toSec());
  }
}
// dvlVelCallback
void ImuPreintegratorNode::dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // Update the DVL velocity in the body frame
  dvl_vel_bw_ = gtsam::Vector3(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  ROS_INFO("DVL velocity updated: [%f, %f, %f]", dvl_vel_bw_.x(), dvl_vel_bw_.y(), dvl_vel_bw_.z());
}
// navStateCallback
void ImuPreintegratorNode::navStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  //ROS_INFO("Received nav state at time: %f", msg->header.stamp.toSec());
  // Skip the first 4 messages to allow for initial setup
  ros::Time current_time = msg->header.stamp;
  last_nav_report_ = gtsam::Pose3(
      gtsam::Rot3(msg->pose.orientation.w, msg->pose.orientation.x,
                  msg->pose.orientation.y, msg->pose.orientation.z),
      gtsam::Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  // Check if the maps are empty
  if (nav_state_map_.empty() || nav_cov_map_.empty()) {
    // Add a new NavState and covariance to the maps
    gtsam::NavState initial_state(
        last_nav_report_.rotation(), // Default to identity rotation
        last_nav_report_.translation(), // Use the baro depth as the initial position
        gtsam::Vector3(0, 0, 0)); // Default to zero velocity
    gtsam::Vector6 initial_cov_sigmas(0.1, 0.1, 0.1, 1.5, 1.5, 0.1); // Set a covariance based on GPS and depth
    gtsam::Matrix6 initial_cov = initial_cov_sigmas.cwiseProduct(initial_cov_sigmas).asDiagonal();
    ti_ = msg->header.stamp; // Set the initial time to the message time
    nav_state_map_[ti_] = initial_state;
    nav_cov_map_[ti_] = initial_cov;
    gtsam::Vector init_rpy = last_nav_report_.rotation().rpy() * (180.0 / M_PI); // Convert to degrees
    ROS_INFO("Initialized NavState, Rotation: [%f, %f, %f], Translation: [%f, %f, %f]",
              init_rpy.x(), init_rpy.y(), init_rpy.z(),
             last_nav_report_.translation().x(),
             last_nav_report_.translation().y(),
             last_nav_report_.translation().z());
  } else {
    // Call propogateState with the current time
    ImuPreintegratorNode::propogateState(current_time);
  }
}
// inWaterCallback
void ImuPreintegratorNode::inWaterCallback(const std_msgs::Bool::ConstPtr& msg) {
  gtsam::Quaternion initial_orientation(1.0, 0.0, 0.0, 0.0); // Default to identity quaternion
  if (!in_water_) {
    // Do nothing
   // ROS_WARN("Not adding preintegration due to out-of-water status");
  } else if (!in_water_ && msg->data==true) {
    if (!imu_buffer_.empty()) {
      const sensor_msgs::Imu& last_msg = imu_buffer_.back();
      ti_ = last_msg.header.stamp; // Reset the start time to the last IMU message time
    } else {
      ROS_WARN("IMU buffer is empty, cannot reset preintegration start time");
    }
    // Reset the Nav State
    gtsam::NavState prevState(last_nav_report_.rotation(), last_nav_report_.translation(), gtsam::Vector3(0, 0, 0));
    gtsam::Vector6 prevCovSigmas(0.1, 0.1, 0.1, 1.5, 1.5, 0.1); // Set a covariance based on GPS and depth
    gtsam::Matrix6 prevCov = prevCovSigmas.cwiseProduct(prevCovSigmas).asDiagonal();
    // /gtsam::Matrix6 prevCov(gtsam::Matrix6::Identity() * gtsam::Vector6(1.5, 1.5, 0.1, 0.1, 0.1, 0.1)); // Set a cov based on GPS and depth
    nav_state_map_[ti_] = prevState;
    nav_cov_map_[ti_] = prevCov;
    start_time_ = ti_;
  }
  in_water_ = msg->data; // Update the in-water status
}
std::pair<gtsam::Vector3, gtsam::Matrix3> ImuPreintegratorNode::getVelocityModel() const {
  gtsam::Vector3 vel_model(0.0, 0.0, 0.0);
  gtsam::Matrix3 vel_noise_model(gtsam::Matrix3::Identity());
  if (!in_water_) {
    ROS_WARN("Not in water, returning zero velocity model");
    vel_noise_model = gtsam::Matrix3::Identity() * 0.1; // Set a small noise for the velocity model
    return std::make_pair(vel_model, vel_noise_model);
  } else if (!dvl_vel_bw_.isApprox(gtsam::Vector3(0.0, 0.0, 0.0))) {
    // If DVL velocity is available, use it
    vel_model = dvl_vel_bw_;
    vel_noise_model = gtsam::Matrix3::Identity() * 0.1; // Set a small noise for the velocity model
    return std::make_pair(vel_model, vel_noise_model);
  } else if (in_water_) {
    // Get the last predicted NavState depth and velocity
    auto it = nav_state_map_.find(ti_);
    if (it != nav_state_map_.end()) {
      ros::Time last_nav_state_time = it->first;
      double dt_rel_nav_state = (ros::Time::now() - last_nav_state_time).toSec();
      const gtsam::NavState& last_nav_state = it->second;
      double last_depth = last_nav_state.position().z();
      gtsam::Vector3 last_velocity = last_nav_state.velocity();
      // Calculate the model velocity by preserving the vx and vy, but setting the vz based on the change in depth
      double vz = (last_nav_report_.translation().z() - last_depth) / dt_rel_nav_state; // Change in depth over time
      vel_model = gtsam::Vector3(0.75, 0.0, vz);
      //vel_model = gtsam::Vector3(last_velocity.x(), last_velocity.y(), vz);
      gtsam::Vector3 vel_noise_sigmas(0.5, 0.5, 0.01) ; // Set a small noise for the velocity model
      // convert sigmas to variance, then reformat as a diagonal matrix
      gtsam::Matrix3 vel_noise_model = vel_noise_sigmas.cwiseProduct(vel_noise_sigmas).asDiagonal();
      return std::make_pair(vel_model, vel_noise_model);
    }else {
      ROS_WARN("No previous NavState found, returning unit velocity");
      return std::make_pair(gtsam::Vector3(1.0, 0.0, 0.0), gtsam::Matrix3::Identity() * 0.1); // Default to unit velocity
    }
  } else {
    return std::make_pair(gtsam::Vector3(0.0, 0.0, 0.0), gtsam::Matrix3::Identity() * 0.1); // Default to zero velocity
  }
}
// Preintegrate IMU buffer between ti_ and tj (specified as an argument)
// Output is a gtsam::Rot3 and a 3x3 covariance matrix
std::tuple<double, gtsam::Rot3, gtsam::Matrix3> ImuPreintegratorNode::preintegrateRotation(
    const ros::Time& tj) {
  if (imu_buffer_.empty()) {
    ROS_WARN("IMU buffer is empty, cannot preintegrate");
    return std::make_tuple(0.0, gtsam::Rot3::Identity(), gtsam::Matrix3::Zero());
  }
  // Create the preintegration parameters
  auto preint_rotation_params = boost::make_shared<gtsam::PreintegratedRotationParams>(
    gyro_noise_,
    gtsam::Vector3(0,0,0));
  // Create the preintegrated rotation object
  gtsam::PreintegratedRotation preint_rotation(preint_rotation_params);
  // Iterate through the IMU buffer and integrate the measurements
  ros::Time last_time = ti_;
  gtsam::Matrix3 deltaRotCovij = gtsam::Matrix3::Zero();
  // While imu buffer is not empty
  while (!imu_buffer_.empty()) {
    const auto& imu_msg = imu_buffer_.front();
    ros::Time t = imu_msg.header.stamp;
    if (t < ti_) {
      ROS_WARN("IMU message at time %f is before the initial time %f, ignoring", t.toSec(), ti_.toSec());
      imu_buffer_.pop_front(); // Remove this message
      continue; // Skip this message
    } else if (t > tj) {
      break; // Stop integrating if we exceed the final time
    } else {
      // Calculate the time difference
      double dt = (imu_msg.header.stamp - last_time).toSec();
      if (dt < 0) {
        ROS_WARN("Non-positive time difference: %f, skipping IMU message", dt);
        continue; // Skip this message if the time difference is non-positive
      }
      // Extract the gyro measurement
      gtsam::Vector3 gyro(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
      gtsam::Matrix3 J_k;
      // Integrate the rotation
      preint_rotation.integrateMeasurement(gyro, bias_, dt, J_k, boost::none);
      // Log the j_k
      // ROS_INFO("Integrating IMU measurement at time %f with dt %f, J_k: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
      //           imu_msg.header.stamp.toSec(), dt,
      //           J_k(0, 0), J_k(0, 1), J_k(0, 2),
      //           J_k(1, 0), J_k(1, 1), J_k(1, 2),
      //           J_k(2, 0), J_k(2, 1), J_k(2, 2));
      // Convert the gyro_noise by the dt
      // gtsam::Matrix3 gyro_noise_hz = gyro_noise_ * dt; // Convert to rad^2/s^2
      //deltaRotCovij += J_k * gyro_noise_hz * J_k.transpose();
      last_time = imu_msg.header.stamp; // Update the last time
      imu_buffer_.pop_front(); // Remove this message after processing
    }
  }
  // Get the final rotation and covariance
  double deltaTij = preint_rotation.deltaTij();
  //gtsam::Rot3 deltaRij = preint_rotation.deltaRij();
  gtsam::Matrix3 H; // H is dR/db
  gtsam::Rot3 deltaRij = preint_rotation.biascorrectedDeltaRij(bias_, &H);
  gtsam::Matrix3 gyro_noise_hz = gyro_noise_ * (1.0 / 80.0); // Convert to rad^2/s^2
  deltaRotCovij = H * gyro_noise_hz * H.transpose();
  // Log the H matrix
  ROS_INFO("H matrix: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
            H(0, 0), H(0, 1), H(0, 2),
            H(1, 0), H(1, 1), H(1, 2),
            H(2, 0), H(2, 1), H(2, 2));
  // Print the diagonals of the gyro noise hz matrix, with full precision
  // Extract the diagonal elements of the deltaRotCovij matrix
  // Print the diagonal elements with full precision

  // std::cout << std::fixed << std::setprecision(12) << gyro_noise_hz << std::endl;
  // Log the deltaTij
  // Reset the preintegration
  preint_rotation.resetIntegration();
  // Get the rotation in rpy
  gtsam::Vector3 deltaRij_rpy = deltaRij.rpy()*(180.0 / M_PI); // Convert to degrees
  // Get the sigmas of the rotation covariance
    gtsam::Vector3 rotSigmas = gtsam::Vector3(
        sqrt(deltaRotCovij(0, 0)),
        sqrt(deltaRotCovij(1, 1)),
        sqrt(deltaRotCovij(2, 2)));
  // // Log the preintegration results
  // ROS_INFO("Preintegrated rotation: deltaTij: %f, deltaRij: [%f, %f, %f], deltaRotCovij: [%f, %f, %f]",
  //          deltaTij, deltaRij_rpy.x(), deltaRij_rpy.y(), deltaRij_rpy.z(),
  //          rotSigmas.x(), rotSigmas.y(), rotSigmas.z());
  // Return the deltaTij, deltaRij, and deltaRotCovij
  return std::make_tuple(deltaTij, deltaRij, deltaRotCovij);
}
gtsam::NavState ImuPreintegratorNode::getPredictedNavState(
    const double deltaTij, const gtsam::Rot3& deltaRij, const gtsam::Vector3& model_velocity) {
  // Get the previous NavState  in the map
  auto it = nav_state_map_.find(ti_);
  if (it == nav_state_map_.end()) {
    ROS_WARN("No previous NavState found at time %f, returning default NavState", ti_.toSec());
    return gtsam::NavState(last_nav_report_.rotation(), last_nav_report_.translation(), gtsam::Vector3(0, 0, 0));
  }
  const gtsam::NavState& prevState = it->second;
  gtsam::Rot3 new_orientation = prevState.attitude().compose(deltaRij);
  gtsam::Point3 new_position = prevState.position() + model_velocity * deltaTij;
  gtsam::Vector3 new_velocity = model_velocity; // Assuming constant velocity for simplicity
  // Create a new NavState with the updated orientation, position, and velocity
  gtsam::NavState predState(new_orientation, new_position, new_velocity);
  return predState;
}
gtsam::Matrix6 ImuPreintegratorNode::getPredictedCovariance(
    const double deltaTij, const gtsam::NavState& predState, const gtsam::Matrix3& deltaRotCovij, const gtsam::Matrix3& vel_noise_model) {
  // Multiple each element of the velocity noise model by deltaTij^2
  gtsam::Matrix3 integrated_vel_noise_model = vel_noise_model * deltaTij;
  auto it_state = nav_state_map_.find(ti_);
  gtsam::NavState& prevState = it_state->second;

  // Get the previous covariance from the map
  auto it_cov = nav_cov_map_.find(ti_);
  if (it_cov == nav_cov_map_.end()) {
    ROS_WARN("No previous covariance found at time %f, returning default covariance", ti_.toSec());
    gtsam::Vector6 prevCovSigmas(1.5, 1.5, 0.1, 0.1, 0.1, 0.1); // Set a covariance based on GPS and depth
    gtsam::Matrix6 prevCov(prevCovSigmas.asDiagonal()); // Create a diagonal covariance matrix
    return prevCov;
  }
  gtsam::Matrix6 prevCov = it_cov->second;
  // Convetr the NavState to a Pose3
  gtsam::Pose3 prevPose = gtsam::Pose3(prevState.attitude(), prevState.position());
  gtsam::Pose3 predPose = gtsam::Pose3(predState.attitude(), predState.position());
  gtsam::Pose3 relative_pose = prevPose.between(predPose);
  // Compute the adjoint of the pose
  gtsam::Matrix6 rel_adjoint = relative_pose.AdjointMap();
  // Compute the covariance of the predicted state
  gtsam::Matrix6 Q = gtsam::Matrix6::Zero();
  Q.block<3, 3>(0, 0) = deltaRotCovij;
  Q.block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
  Q.block<3, 3>(3, 0) = gtsam::Matrix3::Zero();
  Q.block<3, 3>(3, 3) = integrated_vel_noise_model;
  // Log the Q matrix
  // ROS_INFO("Covariance Q: [%f, %f, %f, %f, %f, %f; %f, %f, %f, %f, %f, %f; "
  //          "%f, %f, %f, %f, %f, %f; "
  //          "%f, %f, %f, %f, %f, %f; "
  //          "%f, %f, %f, %f, %f, %f; "
  //          "%f, %f, %f, %f, %f, %f]",
  //          Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3), Q(0, 4), Q(0, 5),
  //          Q(1, 0), Q(1, 1), Q(1, 2), Q(1, 3), Q(1, 4), Q(1, 5),
  //          Q(2, 0), Q(2, 1), Q(2, 2), Q(2, 3), Q(2, 4), Q(2, 5),
  //          Q(3, 0), Q(3, 1), Q(3, 2), Q(3, 3), Q(3, 4), Q(3, 5),
  //          Q(4, 0), Q(4, 1), Q(4, 2), Q(4, 3), Q(4, 4), Q(4, 5),
  //          Q(5, 0), Q(5, 1), Q(5, 2), Q(5, 3), Q(5, 4), Q(5, 5));
  // Compute the predicted covariance
  gtsam::Matrix6 predCov = rel_adjoint * prevCov * rel_adjoint.transpose() + Q;
  // ROS_INFO("Predicted covariance: [%f, %f, %f, %f, %f, %f; %f, %f, %f, %f, %f, %f; %f, %f, %f, %f, %f, %f; "
  //         "%f, %f, %f, %f, %f, %f; %f, %f, %f, %f, %f, %f; "
  //         "%f, %f, %f, %f, %f, %f]",
  //          predCov(0, 0), predCov(0, 1), predCov(0, 2), predCov(0, 3), predCov(0, 4), predCov(0, 5),
  //          predCov(1, 0), predCov(1, 1), predCov(1, 2), predCov(1, 3), predCov(1, 4), predCov(1, 5),
  //          predCov(2, 0), predCov(2, 1), predCov(2, 2), predCov(2, 3), predCov(2, 4), predCov(2, 5),
  //          predCov(3, 0), predCov(3, 1), predCov(3, 2), predCov(3, 3), predCov(3, 4), predCov(3, 5),
  //          predCov(4, 0), predCov(4, 1), predCov(4, 2), predCov(4, 3), predCov(4, 4), predCov(4, 5),
  //          predCov(5, 0), predCov(5, 1), predCov(5, 2), predCov(5, 3), predCov(5, 4), predCov(5, 5));
  return predCov;
}
void ImuPreintegratorNode::propogateState(ros::Time final_time) {
  // Retrieve the latest state int he map for debugging
  gtsam::NavState prevState = nav_state_map_[ti_];
  // Print the previous NavState for debugging
  // ROS_INFO("Previous NavState at time %f: Position (%f, %f, %f)",
  //          ti_.toSec(),
  //          prevState.position().x(), prevState.position().y(), prevState.position().z());
  // Check if the final time is before the initial time
  // Preintegrate the IMU data between ti_ and tj
  auto preint_results = preintegrateRotation(final_time);
  double deltaTij = std::get<0>(preint_results);
  gtsam::Rot3 deltaRij = std::get<1>(preint_results);
  gtsam::Matrix3 deltaRotCovij = std::get<2>(preint_results);
  // Get the predicted NavState and covariance
  gtsam::Vector3 model_velocity;
  gtsam::Matrix3 vel_noise_model;
  std::tie(model_velocity, vel_noise_model) = getVelocityModel();
  gtsam::NavState predState = getPredictedNavState(deltaTij, deltaRij, model_velocity);
  gtsam::Matrix6 predCov = getPredictedCovariance(deltaTij, predState, deltaRotCovij, vel_noise_model);
  nav_state_map_[ti_ + ros::Duration(deltaTij)] = predState;
  nav_cov_map_[ti_ + ros::Duration(deltaTij)] = predCov;
  // Set ti_ to the final time for the next call
  ti_ = ti_ + ros::Duration(deltaTij);
  // Log the reported state
  gtsam::Vector3 last_nav_rpy = last_nav_report_.rotation().rpy();
  last_nav_rpy = gtsam::Vector3(last_nav_rpy.x() * 180.0 / M_PI,
                                               last_nav_rpy.y() * 180.0 / M_PI,
                                               last_nav_rpy.z() * 180.0 / M_PI);
  // ROS_INFO("Reported state at time %f: Position (%f, %f, %f), RPY (%f, %f, %f)",
  //          last_nav_report_.translation().z(),
  //          last_nav_report_.translation().x(), last_nav_report_.translation().y(), last_nav_report_.translation().z(),
  //          last_nav_rpy.x(), last_nav_rpy.y(), last_nav_rpy.z());
  // Log the propogated state and covariance
  // Convert attitude to rpy for logging in degrees
  gtsam::Vector3 rpy = predState.attitude().rpy();
  rpy = gtsam::Vector3(rpy.x() * 180.0 / M_PI,
                      rpy.y() * 180.0 / M_PI,
                      rpy.z() * 180.0 / M_PI);
  // Log the propagated state
  ROS_INFO("Propagated state at time %f: Position (%f, %f, %f), RPY (%f, %f, %f)",
           ti_.toSec(),
           predState.position().x(), predState.position().y(), predState.position().z(),
            rpy.x(), rpy.y(), rpy.z());
  // Convert the covariance to 6 sigmas for logging
  gtsam::Vector6 covSigmas;
  covSigmas << sqrt(predCov(0, 0)), sqrt(predCov(1, 1)), sqrt(predCov(2, 2)),
               sqrt(predCov(3, 3)), sqrt(predCov(4, 4)), sqrt(predCov(5, 5));
  // Log the propagated covariance
  ROS_INFO("Propagated covariance at time %f: Sigmas (X: %f, Y: %f, Z: %f, R: %f, P: %f, Y: %f)",
          ti_.toSec(),
          covSigmas(3), covSigmas(4), covSigmas(5),
          covSigmas(0), covSigmas(1), covSigmas(2));
}
std::pair<gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::getRelativePoseBetweenStates(const ros::Time& initial_time, const ros::Time& final_time) {
  std::map<ros::Time, gtsam::NavState>::iterator it_start;
  std::map<ros::Time, gtsam::NavState>::iterator it_end;

  if (initial_time == ros::Time(0)) {  // When called for normal between factor generation
    it_start = nav_state_map_.find(start_time_);
    if (it_start == nav_state_map_.end()) {
      ROS_WARN("No NavState found at start time %f, cannot compute relative pose", start_time_.toSec());
      return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
    }
  } else {  // When called to get the relative pose from some time to now
    it_start = nav_state_map_.lower_bound(initial_time);
    if (it_start == nav_state_map_.begin()) {
      ROS_WARN("Initial time %f is before the first NavState, cannot compute relative pose", initial_time.toSec());
      return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
    } else if (it_start == nav_state_map_.end()) {
      ROS_WARN("Initial time %f occurs after last NavState, using last NavState", initial_time.toSec());
      --it_start;  // Backtrack to last available
    }
  }

  if (initial_time == ros::Time(0)) { // When called for normal between factor generation
    // start_time_ is unique and held in the nav_state_map_ and nav_cov_map_ at the same indices
    it_start = nav_state_map_.find(start_time_);
    if (it_start == nav_state_map_.end()) {
      ROS_WARN("No NavState found at start time %f, cannot compute relative pose",
                start_time_.toSec());
      return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
    };
  } else { // When called to get the relative pose from some time to now
    // Find the NavState closest to the initial_time
    it_start = nav_state_map_.lower_bound(initial_time);
    if (it_start == nav_state_map_.begin()) {
      ROS_WARN("Initial time %f is before the first NavState, cannot compute relative pose",
             initial_time.toSec());
      return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
    } else if (it_start == nav_state_map_.end()) {
    // If the final time is after the states in the map, return the latest nav state
    ROS_WARN("Initial time %f occurs after last NavState, using last NavState",
             initial_time.toSec());
    };
  }
  // Find the closest time in the map that is less than final_time
  it_end = nav_state_map_.lower_bound(final_time);
  if (it_end == nav_state_map_.begin()) {
    ROS_WARN("Final time %f is before the first NavState, cannot compute relative pose",
             final_time.toSec());
    return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
  } else if (it_end == nav_state_map_.end()) {
    // If the final time is after the states in the map, return the latest nav state
    ROS_WARN("Final time %f occurs after last NavState, using last NavState",
             final_time.toSec());
  };
  // Get the Nav States and covariances
  gtsam::NavState startState = it_start->second;
  gtsam::NavState endState = (--it_end)->second;
  gtsam::Matrix6 startCov = nav_cov_map_[it_start->first];
  gtsam::Matrix6 endCov = nav_cov_map_[it_end->first];
  // Compute the relative pose and covariance
  gtsam::Pose3 startPose = gtsam::Pose3(startState.attitude(), startState.position());
  gtsam::Pose3 endPose = gtsam::Pose3(endState.attitude(), endState.position());
  //gtsam::Pose3 relativePose = startPose.between(endPose);
  // Get the relative pose jacobians
  gtsam::Matrix6 H1, H2;
  gtsam::Pose3 relativePose = gtsam::traits<gtsam::Pose3>::Between(startPose, endPose, H1, H2);
  // Compute the relative covariance
  gtsam::Matrix6 relativeCov = H1 * startCov * H1.transpose() + H2 * endCov * H2.transpose();
  // gtsam::Matrix6 relativeCov = gtsam::Matrix6::Zero();
  // // Get the change in covariance between the two poses (the between factor noise model)
  // gtsam::Matrix6 adjointStart = startPose.AdjointMap();
  // gtsam::Matrix6 adjointEnd = endPose.AdjointMap();
  // relativeCov = adjointStart * startCov * adjointStart.transpose() +
  //               adjointEnd * endCov * adjointEnd.transpose();
  // Update the start time to the final time for the next call
  start_time_ = final_time;
  return std::make_pair(relativePose, relativeCov);
}
bool ImuPreintegratorNode::handlePreintegrate(
    spurdog_acomms::PreintegrateImu::Request &req,
    spurdog_acomms::PreintegrateImu::Response &res) {
  // Call getRelativePoseBetweenStates to get the relative pose and covariance
  auto relativePoseCov = getRelativePoseBetweenStates(req.initial_time, req.final_time);
  gtsam::Pose3 relativePose = relativePoseCov.first;
  gtsam::Matrix6 relativeCov = relativePoseCov.second;
  // Fill the response message (a PoseWithCovarianceStamped)
  res.pose_delta.header.stamp = req.final_time;
  res.pose_delta.header.frame_id = "cv7_ahrs";
  res.pose_delta.pose.pose.position.x = relativePose.x();
  res.pose_delta.pose.pose.position.y = relativePose.y();
  res.pose_delta.pose.pose.position.z = relativePose.z();
  // Get the quaternion orientation from the NavState
  gtsam::Quaternion q = relativePose.rotation().toQuaternion();
  res.pose_delta.pose.pose.orientation.w = q.w();
  res.pose_delta.pose.pose.orientation.x = q.x();
  res.pose_delta.pose.pose.orientation.y = q.y();
  res.pose_delta.pose.pose.orientation.z = q.z();
  // Covariance is block diagonal with the rotation in the upper left 3x3 and the position in the lower right 3x3
  // Fill the upper-left 3x3 block with the position covariance (from relativeCov's bottom-right block)
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      // Position covariance goes in top-left: rows 0-2, cols 0-2
      res.pose_delta.pose.covariance[i * 6 + j] = relativeCov(3 + i, 3 + j);
    }
  }

  // Fill the lower-right 3x3 block with the rotation covariance (from relativeCov's top-left block)
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      // Rotation covariance goes in bottom-right: rows 3-5, cols 3-5
      res.pose_delta.pose.covariance[(i + 3) * 6 + (j + 3)] = relativeCov(i, j);
    }
  }

  return true;
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_preintegrator_node");
  ImuPreintegratorNode node;
  ros::spin();
  return 0;
}