#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include "spurdog_acomms/PreintegrateImu.h"
#include "manifold_preintegrator.hpp"

#include <deque>
#include <mutex>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;
using symbol_shorthand::X;  // pose key
using symbol_shorthand::V;  // velocity key
using symbol_shorthand::B;  // bias key

ImuPreintegratorNode::ImuPreintegratorNode() {
  imu_sub_ = nh_.subscribe("cv7_ahrs", 10000, &ImuPreintegratorNode::imuCallback, this);
  nav_state_sub_ = nh_.subscribe("nav_state", 10, &ImuPreintegratorNode::navStateCallback, this);
  dvl_vel_sub_ = nh_.subscribe("dvl_pdx", 10, &ImuPreintegratorNode::dvlVelCallback, this);
  in_water_sub = nh_.subscribe("in_water", 10, &ImuPreintegratorNode::inWaterCallback, this);
  preint_srv_ = nh_.advertiseService("preintegrate_imu", &ImuPreintegratorNode::handlePreintegration, this);

  // Create variabeles to track the measured data
  imu_buffer_ = std::deque<sensor_msgs::Imu>();
  in_water_ = false;
  ti_ = ros::Time(0);
  imu_q_bw_ = Quaternion(1.0, 0.0, 0.0, 0.0);
  baro_depth_w_ = 0.0;
  dvl_vel_bw_ = Vector3(0.0, 0.0, 0.0);

  // Load the imu noise parameters from the launch file
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
  bias_ = imuBias::ConstantBias(
      Vector3(accel_bias_x, accel_bias_y, accel_bias_z),
      Vector3(gyro_bias_x, gyro_bias_y, gyro_bias_z));
  bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << accel_bias_rw_sigma, accel_bias_rw_sigma, accel_bias_rw_sigma,
                          gyro_bias_rw_sigma, gyro_bias_rw_sigma, gyro_bias_rw_sigma).finished());
  // Initialize for NED using MakeSharedD with gravity pointing down
  pim_params_= PreintegrationCombinedParams::MakeSharedD(9.81); // Gravity in m/s^2
  pim_params_->accelerometerCovariance = I_3x3 * accel_noise_sigma * accel_noise_sigma;
  pim_params_->gyroscopeCovariance = I_3x3 * gyro_noise_sigma * gyro_noise_sigma;
  pim_params_->integrationCovariance = I_3x3 * 1e-8;
  pim_params_->biasAccCovariance = gtsam::I_3x3 * accel_bias_rw_sigma * accel_bias_rw_sigma;
  pim_params_->biasOmegaCovariance = gtsam::I_3x3 * gyro_bias_rw_sigma * gyro_bias_rw_sigma;
  pim_params_->biasAccOmegaInt = gtsam::I_6x6 * 1e-8; // Initial bias covariance
  pim_ = boost::make_shared<PreintegratedCombinedMeasurements>(pim_params_, bias_);

  // Create the ros::Time to factor index mapping
  time_to_factor_index_ = std::map<ros::Time, size_t>();

  // Initialize the graph and initial values
  graph_ = gtsam::NonlinearFactorGraph();
  initial_ = gtsam::Values();
  preint_states_ = std::vector<gtsam::NavState>();
}

// navStateCallback
void ImuPreintegratorNode::navStateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  ros::Time timestamp = msg->header.stamp;
  // Update the previous state with the latest nav state
  gtsam::Quaternion q(msg->pose.pose.orientation.w,
                      msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z);
  // Update the stored depth
  baro_depth_w_ = msg->pose.pose.position.z; // Assuming z is depth in NED frame
  // Generate sequential factors
  ImuPreintegratorNode::generateSequentialFactors(timestamp);
}

// imuCallback
void ImuPreintegratorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (ti_ == ros::Time(0)) {
    ti_ = msg->header.stamp;
    time_to_factor_index_[ti_] = 0; // Initialize the first factor index
    ROS_WARN("First IMU message received at time: %f", ti_.toSec());
  }
  // Check that the IMU Mesage is after ti_
  if (msg->header.stamp < ti_) {
    ROS_WARN("IMU message at time %f is before the initial time %f, ignoring", msg->header.stamp.toSec(), ti_.toSec());
    return;
  }
  // Add the IMU message to the buffer
  imu_buffer_.push_back(*msg);
}

// dvlVelCallback
void ImuPreintegratorNode::dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // Update the DVL velocity in the body frame
  dvl_vel_bw_ = gtsam::Vector3(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

// inWaterCallback
void ImuPreintegratorNode::inWaterCallback(const std_msgs::Bool::ConstPtr& msg) {
  in_water_ = msg->data; // Update the in-water status
  // If manifolfd preintegration is not in water, reset the preintegration
  if (!in_water_) {
    ROS_WARN("Not adding preintegration due to out-of-water status");
  // If we transition from in_water == false to true, we need to reset the preintegration
  } else if (in_water_ && ti_ == ros::Time(0)) {
    // Reset the start time to the current time
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    ti_ = ros::Time(0);
    pim_->resetIntegration();
    preint_states_.clear();
    graph_.resize(0);
    initial_.clear();
    time_to_factor_index_.clear();
    ROS_WARN("Transitioned to in-water, resetting preintegration start time to %f", ti_.toSec());
    ImuPreintegratorNode::generateInitialFactors();
  } else {
    ROS_INFO("In-water status is true, continuing with preintegration");
  }
}

// create the pim object given some final timestamp
void ImuPreintegratorNode::addImuMeasurementsToPIM(const ros::Time& final_time) {
  // Reset the preintegration object
  pim_->resetIntegration();
  // Iterate through the IMU buffer and integrate measurements that fall within the time window [ti_, final_time]
  ros::Time last_msg_time = ti_;
  while (!imu_buffer_.empty()) {
    const auto& imu_msg = imu_buffer_.front();
    ros::Time t = imu_msg.header.stamp;

    if (t < ti_) {
      ROS_WARN("IMU message at time %f is before the initial time %f, ignoring", t.toSec(), ti_.toSec());
      imu_buffer_.pop_front(); // Remove this message
      continue;
    }

    if (t > final_time) {
      break; // Stop integrating if we exceed the final time
    }

    // Convert IMU data to GTSAM format
    Vector3 accel(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    Vector3 omega(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

    // Integrate the measurement
    pim_->integrateMeasurement(accel, omega, (t - last_msg_time).toSec());

    // Update the last message time
    last_msg_time = t;
    imu_buffer_.pop_front(); // Remove this message after processing
  }
  // Update the ti_ to the last message time
  ti_ = last_msg_time;
}

// generate initial set of factors
void ImuPreintegratorNode::generateInitialFactors() {
  // Reset the preintegration object
  pim_->resetIntegration();
  // Clear the graph and initial values
  graph_.resize(0);
  initial_.clear();
  preint_states_.clear();
  time_to_factor_index_.clear();
  // Set the initial time to the first IMU message time
  if (imu_buffer_.empty()) {
    ROS_WARN("IMU buffer is empty, cannot generate initial factors");
    return;
  }
  ti_ = imu_buffer_.front().header.stamp;
  time_to_factor_index_[ti_] = 0; // Initialize the first factor index
  // Create initial set of X, V, B keys
  gtsam::Key pose_key = X(0); // Initial pose key
  gtsam::Key vel_key = V(0);  // Initial velocity key
  gtsam::Key bias_key = B(0); // Initial bias key
  // Generate the pose, velocity and bias prior factors
  gtsam::Pose3 initial_pose(
    gtsam::Rot3(imu_q_bw_.w(), imu_q_bw_.x(), imu_q_bw_.y(), imu_q_bw_.z()),
    gtsam::Point3(0.0, 0.0, baro_depth_w_) // Assuming z is depth in NED frame
  );
  gtsam::Vector3 initial_velocity(0.0, 0.0, 0.0); // Assuming stationary at start
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished());
  gtsam::noiseModel::Diagonal::shared_ptr vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(3) << 0.01, 0.01, 0.01).finished());
  // Bias noise is initialized on startup
  // Add prior factors to the graph
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(pose_key, initial_pose, pose_noise));
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(vel_key, initial_velocity, vel_noise));
  graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bias_key, bias_, bias_noise_));
  // Add the initial state to the initial values
  initial_.insert(pose_key, initial_pose);
  initial_.insert(vel_key, initial_velocity);
  initial_.insert(bias_key, bias_);
  // Create the initial NavState
  gtsam::NavState initial_state(initial_pose, initial_velocity);
  // Add the initial NavState to the preintegrated states
  preint_states_.push_back(initial_state);
  // Log the initial state
  ROS_INFO("Generated Initial Factors at time %f with NavState [%f, %f, %f], [%f, %f, %f]",
           ti_.toSec(),
           initial_pose.x(), initial_pose.y(), initial_pose.z(),
           initial_velocity.x(), initial_velocity.y(), initial_velocity.z());
}

// generate sequential factors
void ImuPreintegratorNode::generateSequentialFactors(const ros::Time& final_time) {
  // Ensure we have a valid start time
  if (ti_ == ros::Time(0)) {
    ROS_WARN("Initial time is not set, cannot generate sequential factors");
    return;
  }
  // Add IMU measurements to the preintegration object
  ImuPreintegratorNode::addImuMeasurementsToPIM(final_time);
  // Check if we have any measurements to process
  if (pim_->deltaTij() <= 0) {
    ROS_WARN("No valid IMU measurements found for preintegration");
    return;
  }
  // Create a new factor for the preintegrated measurements
  double next_idx = time_to_factor_index_.size();
  // Create keys for the next pose, velocity, and bias
  gtsam::Key xi = X(next_idx-1); // Previous pose key
  gtsam::Key vi = V(next_idx-1); // Previous velocity key
  gtsam::Key bi = B(next_idx-1); // Previous bias key
  gtsam::Key xj = X(next_idx);
  gtsam::Key vj = V(next_idx);
  gtsam::Key bj = B(next_idx);
  // Create the preintegrated factor
  gtsam::CombinedImuFactor factor(xi, vi, bi, xj, vj, bj,*pim_);
  // Add the factor to the graph
  graph_.add(factor);
  // Update the initial values with the new pose and velocity
  gtsam::Pose3 predicted_pose = pim_->predict(preint_states_.back(), bias_).pose();
  // overide the previous pose orientation with the latest IMU orientation and baro depth
  predicted_pose = gtsam::Pose3(gtsam::Rot3(imu_q_bw_.w(), imu_q_bw_.x(), imu_q_bw_.y(), imu_q_bw_.z()),
                                gtsam::Point3(predicted_pose.x(), predicted_pose.y(), baro_depth_w_));
  gtsam::Vector3 predicted_velocity = pim_->predict(preint_states_.back(), bias_).velocity();
  initial_.insert(xj, predicted_pose);
  initial_.insert(vj, predicted_velocity);
  initial_.insert(bj, bias_);
  // Add priors
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 1, 1, 0.05, 0.1, 0.1, 0.1).finished()); // Loose prior on X,Y, strict prior on Z, and orientation
  gtsam::noiseModel::Diagonal::shared_ptr vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(3) << 0.5, 0.1, 0.05).finished()); // Loose prior on VX, progressively stricter priors on VY, VZ
  // Bias noise is initialized on startup
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(xj, predicted_pose, pose_noise));
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(vj, predicted_velocity, vel_noise));
  graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bj, bias_, bias_noise_));
  // Update the preintegrated states
  gtsam::NavState new_state(predicted_pose, predicted_velocity);
  // Log the nav state
  ROS_INFO("Generated New Factors at time %f with NavState: [%f, %f, %f], [%f, %f, %f]",
           final_time.toSec(),
           predicted_pose.x(), predicted_pose.y(), predicted_pose.z(),
           predicted_velocity.x(), predicted_velocity.y(), predicted_velocity.z());
  preint_states_.push_back(new_state);
  // Update the time to factor index mapping
  time_to_factor_index_[final_time] = time_to_factor_index_.size();
}

// Solve the factor graph between two integer indices (return gtsam::values)
gtsam::Values ImuPreintegratorNode::solveFactorGraph(const int start_idx, const int end_idx) {
  // Check if the factor graph is empty
  if (graph_.size() == 0) {
    ROS_WARN("Factor graph is empty, cannot solve");
    return gtsam::Values();
  }
  // TODO: Add checks that the start and end are in the graph (check X(start_idx), X(end_idx), etc.)

  // Create an optimizer
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_, params);
  // Optimize the factor graph
  gtsam::Values result = optimizer.optimize();
  // Return the optimized values
  return result;
}

// build preintegration response
geometry_msgs::PoseWithCovarianceStamped ImuPreintegratorNode::buildPoseResponse(
    const gtsam::Pose3& pose, const gtsam::Matrix& covariance, const int end_idx) {
  ros::Time tj;
  for (const auto& entry : time_to_factor_index_) {
    if (entry.second == end_idx) {
      tj = entry.first;
      break;
    }
  }
  geometry_msgs::PoseWithCovarianceStamped response;
  response.header.stamp = tj;
  response.header.frame_id = "imu";
  response.pose.pose.position.x = pose.x();
  response.pose.pose.position.y = pose.y();
  response.pose.pose.position.z = pose.z();
  gtsam::Quaternion q = pose.rotation().toQuaternion();
  response.pose.pose.orientation.w = q.w();
  response.pose.pose.orientation.x = q.x();
  response.pose.pose.orientation.y = q.y();
  response.pose.pose.orientation.z = q.z();
  // Fill the covariance matrix
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      response.pose.covariance[i * 6 + j] = covariance(i, j);
    }
  }
  return response;
}

void ImuPreintegratorNode::pruneGraph(const int end_idx, const gtsam::Values& results) {
  // Remove any keys and factors prior to the end index
  for (int i = 0; i < end_idx; ++i) {
    // TODO: Check this syntax
    graph_.remove((X(i)));
    graph_.remove((V(i)));
    graph_.remove((B(i)));
    initial_.erase(X(i));
    initial_.erase(V(i));
    initial_.erase(B(i));
  }
  // For any indices greter than or equal to the end index, initialize to the optimized values
  for (int i = end_idx; i < time_to_factor_index_.size(); ++i) {
    gtsam::Key pose_key = X(i);
    gtsam::Key vel_key = V(i);
    gtsam::Key bias_key = B(i);
    if (results.exists(pose_key)) {
      initial_.insert(pose_key, results.at<gtsam::Pose3>(pose_key));
    }
    if (results.exists(vel_key)) {
      initial_.insert(vel_key, results.at<gtsam::Vector3>(vel_key));
    }
    if (results.exists(bias_key)) {
      initial_.insert(bias_key, results.at<gtsam::imuBias::ConstantBias>(bias_key));
    }
  }
}

geometry_msgs::PoseWithCovarianceStamped ImuPreintegratorNode::getResults(
  const gtsam::Values result, const int start_idx, const int end_idx) {
  // Get the optimized values
  gtsam::Pose3 optimized_pose_i = result.at<gtsam::Pose3>(X(start_idx));
  gtsam::Vector3 optimized_velocity_i = result.at<gtsam::Vector3>(V(start_idx));
  gtsam::imuBias::ConstantBias optimized_bias_i = result.at<gtsam::imuBias::ConstantBias>(B(start_idx));
  gtsam::Pose3 optimized_pose_j = result.at<gtsam::Pose3>(X(end_idx));
  gtsam::Vector3 optimized_velocity_j = result.at<gtsam::Vector3>(V(end_idx));
  gtsam::imuBias::ConstantBias optimized_bias_j = result.at<gtsam::imuBias::ConstantBias>(B(end_idx));
  gtsam::Marginals marginals = gtsam::Marginals(graph_, result);
  // Get the marginal covariance of all variables
  gtsam::Matrix pose_covariance_i = marginals.marginalCovariance(X(start_idx));
  gtsam::Matrix pose_covariance_j = marginals.marginalCovariance(X(end_idx));
  gtsam::Matrix velocity_covariance_i = marginals.marginalCovariance(V(start_idx));
  gtsam::Matrix velocity_covariance_j = marginals.marginalCovariance(V(end_idx));
  gtsam::Matrix bias_covariance_i = marginals.marginalCovariance(B(start_idx));
  gtsam::Matrix bias_covariance_j = marginals.marginalCovariance(B(end_idx));
  // Find the relative covariance of the relative pose
  gtsam::Matrix Hi, Hj;
  gtsam::Pose3 Tij = optimized_pose_i.between(optimized_pose_j, Hi, Hj);
  gtsam::Matrix Covij = Hi * pose_covariance_i * Hi.transpose() +
                                         Hj * pose_covariance_j * Hj.transpose();
  // Log the solved pose difference
  ROS_INFO("Solved delta index %d to %d: [%.2f, %.2f, %.2f], [%.2f, %.2f, %.2f]",
           start_idx, end_idx,
           Tij.x(), Tij.y(), Tij.z(),
           optimized_velocity_i.x(), optimized_velocity_i.y(), optimized_velocity_i.z());
  // Build the response message
  geometry_msgs::PoseWithCovarianceStamped response_msg = ImuPreintegratorNode::buildPoseResponse(Tij,Covij, end_idx);
  // Remove any keys and factors prior to the end index
  ImuPreintegratorNode::pruneGraph(end_idx, result);
  // Return the response
  return response_msg;
}

bool ImuPreintegratorNode::handlePreintegration(
    spurdog_acomms::PreintegrateImu::Request& req,
    spurdog_acomms::PreintegrateImu::Response& res) {
  ros::Time end_time = req.final_time;
    // Check that there is IMU data available
  if (time_to_factor_index_.empty()) {
    ROS_ERROR("No IMU data available for preintegration");
    return false;
  //Check that the graph is not empty
  } else if (graph_.size() == 0) {
    ROS_ERROR("Factor graph is empty, cannot preintegrate");
    return false;
  } else if (end_time < ti_) {
    ROS_ERROR("Invalid time range: start_time must be less than end_time");
    return false;
  }
  // Find the indices based on the ti_ and final time
  int start_idx = -1;
  int end_idx = -1;
  for (const auto& entry : time_to_factor_index_) {
    if (entry.first == ti_) {
      start_idx = entry.second;
    }
    if (entry.first == end_time) {
      end_idx = entry.second;
    }
  }
  // Check if the start and end indices are valid
  if (start_idx == -1 || end_idx == -1 || start_idx >= end_idx) {
    ROS_ERROR("Invalid start or end index for preintegration");
    return false;
  }
  // Solve the factor graph
  gtsam::Values result = ImuPreintegratorNode::solveFactorGraph(start_idx, end_idx);
  // Get the results and build the response
  res.pose_delta = ImuPreintegratorNode::getResults(result, start_idx, end_idx);
  res.success = true; // Indicate that the preintegration was successful
  ROS_INFO("Preintegration successful from time %f to %f", ti_.toSec(), end_time.toSec());
  return true; // Indicate that the service was handled successfully
}

// Main function to initialize the ROS node
int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_preintegrator_node");
  ImuPreintegratorNode node;
  ros::spin();
  return 0;
}
// This code is a ROS node that preintegrates IMU measurements using GTSAM's manifold preintegration.
// It subscribes to IMU, navigation state, DVL velocity, and in-water status topics,
// and provides a service to preintegrate IMU data over a specified time range.
// The node maintains a buffer of IMU messages, integrates them into a preintegration object,