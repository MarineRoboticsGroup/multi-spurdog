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
#include <fstream>

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
  ros::NodeHandle private_nh_("~");
  std::string resolved_ns = ros::this_node::getNamespace();  // e.g., "/actor_0"
  ros::NodeHandle actor_ns(resolved_ns);  // Explicit NodeHandle for /actor_0
  imu_sub_ = actor_ns.subscribe("cv7_ahrs", 10, &ImuPreintegratorNode::imuCallback, this);
  //nav_state_sub_ = actor_ns.subscribe("nav_state", 10, &ImuPreintegratorNode::navStateCallback, this);
  in_water_sub_ = actor_ns.subscribe("in_water", 10, &ImuPreintegratorNode::inWaterCallback, this);
  preint_srv_ = actor_ns.advertiseService("preintegrate_imu", &ImuPreintegratorNode::handlePreintegrate, this);

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
  bias_ = gtsam::Vector3(gyro_bias_x, gyro_bias_y, gyro_bias_z);

  // Set the velocity source
  std::string velocity_source;
  private_nh_.param("velocity_source", velocity_source, std::string("constant"));
  if (velocity_source == "dvl") {
    dvl_vel_sub_ = actor_ns.subscribe("dvl_pdx", 10, &ImuPreintegratorNode::dvlVelCallback, this);
    gtsam::Vector3 dvl_noise_sigmas(0.1, 0.05, 0.02); // For single measurement
    vel_noise_model_ = dvl_noise_sigmas.cwiseProduct(dvl_noise_sigmas).asDiagonal();
  } else {
    ROS_WARN("Velocity source not set to DVL, using default constant velocity");
    vel_model_ = gtsam::Vector3(0.8, 0.0, 0.0); // Default to typical velocity
    gtsam::Vector3 constant_vel_noise_sigmas(0.5, 0.25, 0.1); // Cautiously set to 5x DVL
    vel_noise_model_ = constant_vel_noise_sigmas.cwiseProduct(constant_vel_noise_sigmas).asDiagonal();
  }

  // Initialize the buffers
  ti_ = ros::Time(0); // Initialize the time to zero
  imu_buffer_ = std::deque<sensor_msgs::Imu>();
  dvl_buffer_ = std::deque<geometry_msgs::TwistStamped>();

  // After 10sec, handle the initial state
  ros::Duration(1.0).sleep(); // Wait for 10 seconds to allow for initial setup
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
  dvl_buffer_.push_back(*msg);
}

// inWaterCallback
void ImuPreintegratorNode::inWaterCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!in_water_) {
    // Do nothing if already out of water
  } else if (!in_water_ && msg->data==true) { // Entering the water
    if (!imu_buffer_.empty()) {
      const sensor_msgs::Imu& last_msg = imu_buffer_.back();
      ti_ = last_msg.header.stamp;
      imu_buffer_.clear(); // Clear the IMU buffer to start fresh
      dvl_buffer_.clear(); // Clear the DVL velocity buffer as well
    } else {
      ROS_WARN("IMU buffer is empty, cannot reset preintegration start time");
    }
  }
  in_water_ = msg->data; // Update the in-water status
}

// initStateAndCovariance
std::pair<gtsam::NavState, gtsam::Matrix6> ImuPreintegratorNode::initStateAndCovariance()
{
  // Initialize the NavState and covariance maps
    gtsam::NavState initial_state(
        gtsam::Rot3(), // Default to identity rotation
        gtsam::Point3(0, 0, 0), // Default to zero position
        gtsam::Vector3(0, 0, 0)); // Default to zero velocity
    gtsam::Vector6 initial_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2).finished();
    gtsam::Matrix6 initial_cov = initial_cov_sigmas.cwiseProduct(initial_cov_sigmas).asDiagonal();
    //Return the initial state and covariance
    return std::make_pair(initial_state, initial_cov);
}

// getAverageVelocityBetweenTimes
std::pair<gtsam::Vector3, gtsam::Matrix3> ImuPreintegratorNode::getAverageVelocityBetweenTimes(
    const ros::Time& initial_time, const ros::Time& final_time) {
  // Check the velocity source
  if (!in_water_){
    ROS_WARN("Not in water, returning zero velocity model");
    return std::make_pair(gtsam::Vector3(0.0, 0.0, 0.0), gtsam::Matrix3::Identity() * 0.1);
  } else if (dvl_buffer_.empty()) {
    return std::make_pair(vel_model_, vel_noise_model_); // Use the constant velocity model
  } else {
    // Take the average velocity from the DVL buffer
    gtsam::Vector3 avg_velocity(0.0, 0.0, 0.0);
    gtsam::Matrix3 avg_velocity_cov = gtsam::Matrix3::Zero();
    int count = 0;
    while (!dvl_buffer_.empty()) {
      const auto& dvl_msg = dvl_buffer_.front();
      if (dvl_msg.header.stamp < initial_time) {
        dvl_buffer_.pop_front(); // Remove old messages
        continue; // Skip this message
      } else if (dvl_msg.header.stamp > final_time) {
        break; // Stop processing if we exceed the final time
      } else {
        // Accumulate the velocity
        avg_velocity += gtsam::Vector3(dvl_msg.twist.linear.x, dvl_msg.twist.linear.y, dvl_msg.twist.linear.z);
        count++;
        dvl_buffer_.pop_front(); // Remove this message after processing
      }
    }
    // Get the average velocity
    if (count > 0) {
      avg_velocity /= count; // Average the velocity
      avg_velocity_cov = vel_noise_model_ / count; // Average the covariance
      return std::make_pair(avg_velocity, avg_velocity_cov);
    } else {
      ROS_WARN("No valid DVL messages found between %f and %f, returning zero velocity model",
               initial_time.toSec(), final_time.toSec());
      return std::make_pair(gtsam::Vector3(0.0, 0.0, 0.0), gtsam::Matrix3::Identity() * 0.1);
    }
  }
}

// getPreintegratedRotation
std::tuple<double, gtsam::Rot3, gtsam::Matrix3> ImuPreintegratorNode::getPreintegratedRotation(
    const ros::Time& initial_time, const ros::Time& final_time) {
  // Check if the IMU buffer is empty
  if (imu_buffer_.empty()) {
    ROS_WARN("IMU buffer is empty, cannot preintegrate rotation");
    return std::make_tuple(0.0, gtsam::Rot3::Identity(), gtsam::Matrix3::Zero());
  }
  // Create the preintegration parameters
  auto preint_rotation_params = boost::make_shared<gtsam::PreintegratedRotationParams>(
      gyro_noise_, bias_);
  // Create the preintegrated rotation object
  gtsam::PreintegratedRotation preint_rotation(preint_rotation_params);
  // Iterate through the IMU buffer and integrate the measurements
  ros::Time last_time = initial_time;
  double count = 0.0; // Initialize mean dt
  gtsam::Matrix3 deltaRotCovij = gtsam::Matrix3::Zero();
  // While imu buffer is not empty
  while (!imu_buffer_.empty()) {
    const auto& imu_msg = imu_buffer_.front();
    ros::Time t = imu_msg.header.stamp;
    if (t < initial_time) {
      ROS_WARN("IMU message at time %f is before the initial time %f, ignoring", t.toSec(), initial_time.toSec());
      imu_buffer_.pop_front(); // Remove this message
      continue; // Skip this message
    } else if (t > final_time) {
      break; // Stop integrating if we exceed the final time
    } else {
      // Calculate the time difference
      double dt = (imu_msg.header.stamp - last_time).toSec();
      if (dt < 0) {
        ROS_WARN("Non-positive time difference: %f, skipping IMU message", dt);
        imu_buffer_.pop_front(); // Remove this message
        continue; // Skip this message
      }
      count += 1;
      // Extract the gyro measurement
      gtsam::Vector3 gyro(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
      gtsam::Matrix3 J_k;
      preint_rotation.integrateMeasurement(gyro, bias_, dt, J_k, boost::none); // Returns a Jacobian if need for debugging
      last_time = imu_msg.header.stamp; // Update the last time
      imu_buffer_.pop_front(); // Remove this message after processing
    }
  }
  // Get the final rotation and covariance
  double deltaTij = preint_rotation.deltaTij();
  double mean_dt = count > 0 ? deltaTij / count : 0.0; // Calculate the mean dt
  gtsam::Matrix3 H; // H is dR/db
  gtsam::Rot3 deltaRij = preint_rotation.biascorrectedDeltaRij(bias_, &H);
  gtsam::Matrix3 gyro_noise_hz = gyro_noise_ * mean_dt; // Convert to rad^2/s^2
  deltaRotCovij = H * gyro_noise_hz * H.transpose();
  preint_rotation.resetIntegration();
  // Log the preintegration results
  return std::make_tuple(deltaTij, deltaRij, deltaRotCovij);
}

// propagateState
std::pair<gtsam::NavState, gtsam::Matrix6> ImuPreintegratorNode::propagateState(
    const std::pair<gtsam::NavState, gtsam::Matrix6>& initial_state_and_cov,
    double deltaTij,
    const gtsam::Rot3& deltaRij,
    const gtsam::Matrix3& deltaRotCovij,
    const gtsam::Vector3& model_velocity,
    const gtsam::Matrix3& vel_noise_model) {
  // Extract the initial state and covariance
  gtsam::NavState initial_state = initial_state_and_cov.first;
  gtsam::Matrix6 initial_cov = initial_state_and_cov.second;
  // Calculate the new pose, given the initial state
  gtsam::Pose3 initial_pose = gtsam::Pose3(initial_state.attitude(), initial_state.position());
  // Calculate the new pose after applying the delta rotation and model velocity
  gtsam::Pose3 predicted_pose = initial_pose.compose(gtsam::Pose3(deltaRij, gtsam::Point3(model_velocity * deltaTij)));
  gtsam::NavState predicted_state(predicted_pose.rotation(), predicted_pose.translation(), model_velocity);
  // Calculate the relative pose
  gtsam::Pose3 relative_pose = initial_pose.between(predicted_pose);
  gtsam::Matrix6 rel_adjoint = relative_pose.AdjointMap();
  // Compute the covariance of the predicted state
  gtsam::Matrix6 Q = gtsam::Matrix6::Zero();
  Q.block<3, 3>(0, 0) = deltaRotCovij;
  Q.block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
  Q.block<3, 3>(3, 0) = gtsam::Matrix3::Zero();
  Q.block<3, 3>(3, 3) = vel_noise_model;
  gtsam::Matrix6 predicted_cov = rel_adjoint * initial_cov * rel_adjoint.transpose() + Q;
  return std::make_pair(predicted_state, predicted_cov);
}

// getRelativePoseBetweenStates
std::pair<gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::getRelativePoseBetweenStates(
    const ros::Time& initial_time, const ros::Time& final_time) {
  // Check if the initial and final times are valid
  if (initial_time.isZero() || final_time.isZero()) {
    ROS_ERROR("Invalid initial or final time for preintegration");
    return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
  } else if (initial_time >= final_time) {
    ROS_ERROR("Initial time must be less than final time for preintegration");
    return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
  } else if (imu_buffer_.empty()) {
    ROS_ERROR("IMU buffer is empty, cannot preintegrate");
    return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
  } else if (dvl_buffer_.empty()) {
    //ROS_WARN("DVL buffer is empty, using constant velocity model");
  } else {
    ROS_INFO("Preintegrating IMU data from %f to %f", initial_time.toSec(), final_time.toSec());
    // Get the initial NavState and covariance
    auto initial_state_and_cov = initStateAndCovariance();
    // gtsam::NavState initial_state = initial_state_and_cov.first;
    // gtsam::Matrix6 initial_cov = initial_state_and_cov.second;
    // Get the average velocity between the initial and final times from the DVL buffer
    auto avg_velocity_and_cov = getAverageVelocityBetweenTimes(initial_time, final_time);
    gtsam::Vector3 avg_velocity = avg_velocity_and_cov.first;
    gtsam::Matrix3 avg_velocity_cov = avg_velocity_and_cov.second;
    // Get the preintegrated rotation and covariance
    auto preint_results = getPreintegratedRotation(initial_time, final_time);
    double deltaTij = std::get<0>(preint_results);
    gtsam::Rot3 deltaRij = std::get<1>(preint_results);
    gtsam::Matrix3 deltaRotCovij = std::get<2>(preint_results);
    // Get the predicted NavState and covariance
    std::pair<gtsam::NavState, gtsam::Matrix6> predicted_state_and_cov;
    predicted_state_and_cov = propagateState(
        initial_state_and_cov, deltaTij, deltaRij, deltaRotCovij, avg_velocity, avg_velocity_cov);
    return std::make_pair(
      gtsam::Pose3(predicted_state_and_cov.first.attitude(), predicted_state_and_cov.first.position()),
      predicted_state_and_cov.second);
  }
  // If we reach here, something went wrong
  ROS_ERROR("Failed to compute relative pose between states");
  return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
}

// handlePreintegrate
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
  // If the node shuts down, write the maps to a file
  // node.writePosesToTumFile("/ros/logs/"); // Change this to your desired directory
  ROS_INFO("ImuPreintegratorNode shutting down, maps written to /tmp");
  return 0;
}