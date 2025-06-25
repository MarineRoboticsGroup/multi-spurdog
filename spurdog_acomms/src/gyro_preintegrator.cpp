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
#include <csignal>

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

ImuPreintegratorNode* g_node_ptr = nullptr;

ImuPreintegratorNode::ImuPreintegratorNode() {
  ros::NodeHandle private_nh_("~");
  std::string resolved_ns = ros::this_node::getNamespace();  // e.g., "/actor_0"
  ros::NodeHandle actor_ns(resolved_ns);  // Explicit NodeHandle for /actor_0
  imu_sub_ = actor_ns.subscribe("cv7_ahrs", 10, &ImuPreintegratorNode::imuCallback, this);
  nav_state_sub_ = actor_ns.subscribe("nav_state", 10, &ImuPreintegratorNode::navStateCallback, this);
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
    gtsam::Vector3 dvl_noise_sigmas(0.1, 0.1, 0.1); // m/s, in body frame For single measurement
    vel_noise_model_ = dvl_noise_sigmas.cwiseProduct(dvl_noise_sigmas).asDiagonal();
  } else {
    ROS_WARN("Velocity source not set to DVL, using default constant velocity");
    vel_model_ = gtsam::Vector3(0.8, 0.0, 0.0); // Default to typical velocity
    gtsam::Vector3 constant_vel_noise_sigmas(0.5, 0.25, 0.1); // Cautiously set to 5x DVL
    vel_noise_model_ = constant_vel_noise_sigmas.cwiseProduct(constant_vel_noise_sigmas).asDiagonal();
  }

  // Initialize the buffers
  ti_ = ros::Time(0); // Initialize the time to zero
  in_water_ = false; // Initialize the in-water status
  current_orientation_ = gtsam::Rot3::Identity(); // Initialize the current orientation
  R_wb_i_ = gtsam::Rot3::Identity(); // Initialize the rotation from body to world frame
  imu_buffer_ = std::deque<sensor_msgs::Imu>();
  dvl_buffer_ = std::deque<geometry_msgs::TwistStamped>();
  dr_state_and_cov_ = std::make_tuple(ros::Time(0), gtsam::Pose3(), gtsam::Matrix6::Identity().eval());
  last_nav_report_ = std::make_pair(ros::Time(0), gtsam::Pose3());

  // After 10sec, handle the initial state
  ros::Duration(1.0).sleep(); // Wait for 10 seconds to allow for initial setup

}

// imuCallback
void ImuPreintegratorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push_back(*msg);
  if (ti_ == ros::Time(0)) {
    ti_ = msg->header.stamp;
    ROS_WARN("First IMU message received at time: %f", ti_.toSec());
    R_wb_i_ = gtsam::Rot3::Quaternion(
        msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  }
  // Convert the quaternion to RPY angles (degrees)
  // gtsam::Vector3 rpy = gtsam::Rot3(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z).rpy();
  // ROS_INFO("IMU Orientation (RPY): [%f, %f, %f]", rpy.x() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI, rpy.z() * 180.0 / M_PI);
  // Update the current orientation based on the IMU message
  current_orientation_ = gtsam::Rot3::Quaternion(
      msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

// navStateCallback
void ImuPreintegratorNode::navStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Update the initial state and covariance based on the nav state
  ros::Time t = std::get<0>(dr_state_and_cov_);
  gtsam::Pose3 pose = std::get<1>(dr_state_and_cov_);
  gtsam::Matrix6 cov = std::get<2>(dr_state_and_cov_);
  if (t == ros::Time(0) &&
      pose.equals(gtsam::Pose3()) &&  // Pose3 provides an equals() method
      cov.isApprox(gtsam::Matrix6::Identity())) {
    // Initialize the initial state and covariance
    gtsam::Pose3 initial_pose(
        gtsam::Rot3::Quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
        gtsam::Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    gtsam::Vector6 initial_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
    gtsam::Matrix6 initial_cov = initial_cov_sigmas.cwiseProduct(initial_cov_sigmas).asDiagonal();
    dr_state_and_cov_ = std::make_tuple(msg->header.stamp, initial_pose, initial_cov);
    ROS_INFO("Initial state set at time: %f", msg->header.stamp.toSec());
  }
  // Update the last nav report
  last_nav_report_ = std::make_pair(msg->header.stamp,
      gtsam::Pose3(gtsam::Rot3::Quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
                    gtsam::Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)));
}

// dvlVelCallback
void ImuPreintegratorNode::dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // Rotate the DVL from body to world frame using the current orientation
  gtsam::Rot3 R_wb = current_orientation_;
  gtsam::Vector3 dvl_velocity(
      msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  // Transform the DVL velocity to the world frame
  gtsam::Vector3 dvl_velocity_world = R_wb.rotate(dvl_velocity);
  // Create a new TwistStamped message with the transformed velocity
  geometry_msgs::TwistStamped transformed_msg;
  transformed_msg.header = msg->header; // Copy the header
  transformed_msg.twist.linear.x = dvl_velocity_world.x();
  transformed_msg.twist.linear.y = dvl_velocity_world.y();
  transformed_msg.twist.linear.z = dvl_velocity_world.z();
  dvl_buffer_.push_back(transformed_msg);
}

// inWaterCallback
void ImuPreintegratorNode::inWaterCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!in_water_) {
    // Set the dr_pose and covariance to the last nav report
    if (last_nav_report_.first != ros::Time(0)) {
      gtsam::Pose3 last_pose = last_nav_report_.second;
      gtsam::Vector6 last_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
      gtsam::Matrix6 last_cov = last_cov_sigmas.cwiseProduct(last_cov_sigmas).asDiagonal();
      dr_state_and_cov_ = std::make_tuple(ti_, last_pose, last_cov);
      ROS_INFO("Reset preintegration start time to %f with pose [%f, %f, %f]",
                ti_.toSec(), last_pose.translation().x(), last_pose.translation().y(), last_pose.translation().z());
    } else {
      ROS_WARN("Last nav report is not set, cannot reset preintegration start time");
    }
  } else if (!in_water_ && msg->data==true) { // Entering the water
    if (!imu_buffer_.empty()) {
      const sensor_msgs::Imu& last_msg = imu_buffer_.back();
      ti_ = last_msg.header.stamp;
      imu_buffer_.clear(); // Clear the IMU buffer to start fresh
      dvl_buffer_.clear(); // Clear the DVL velocity buffer as well
    }else {
      ROS_WARN("IMU buffer is empty, cannot reset preintegration start time");
    }
    // Set the dr_pose and covariance to the last nav report
    if (last_nav_report_.first != ros::Time(0)) {
      gtsam::Pose3 last_pose = last_nav_report_.second;
      gtsam::Vector6 last_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
      gtsam::Matrix6 last_cov = last_cov_sigmas.cwiseProduct(last_cov_sigmas).asDiagonal();
      dr_state_and_cov_ = std::make_tuple(ti_, last_pose, last_cov);
      ROS_INFO("Reset preintegration start time to %f with pose [%f, %f, %f]",
                ti_.toSec(), last_pose.translation().x(), last_pose.translation().y(), last_pose.translation().z());
    } else {
      ROS_WARN("Last nav report is not set, cannot reset preintegration start time");
    }
  } else {
    // IN the water, do nothing
  }
  in_water_ = msg->data; // Update the in-water status
}

// initStateAndCovariance
std::pair<gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::initStateAndCovariance()
{
  // Initialize the NavState and covariance maps
    gtsam::Pose3 initial_pose(
        gtsam::Rot3::Identity(), gtsam::Point3(0.0, 0.0, 0.0)); // Initial pose at origin
    gtsam::Vector6 initial_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-2).finished();
    gtsam::Matrix6 initial_cov = initial_cov_sigmas.cwiseProduct(initial_cov_sigmas).asDiagonal();
    //Return the initial state and covariance
    return std::make_pair(initial_pose, initial_cov);
}

// getAverageVelocityBetweenTimes
std::pair<gtsam::Vector3, gtsam::Matrix3> ImuPreintegratorNode::getPreintegratedTranslation(
    const ros::Time& initial_time, const ros::Time& final_time) {
  // Applies the desried velocity model over tij to return the preintegrated translation (world frame) and its covariance
  // Check the velocity source
  if (!in_water_){
    ROS_WARN("Not in water, returning zero velocity model");
    return std::make_pair(gtsam::Vector3(0.0, 0.0, 0.0), gtsam::Matrix3::Identity() * 0.1);
  } else if (dvl_buffer_.empty()) {
    //ROS_WARN("DVL buffer is empty, returning constant velocity model");
    ros::Duration duration = final_time - initial_time;
    // Rotate the constant velocity model from body to world frame using the orientation
    gtsam::Rot3 R_wb = current_orientation_;
    gtsam::Vector3 constant_velocity_world = R_wb.rotate(vel_model_);
    // Get Translation and scale the covariance
    gtsam::Vector3 translation = constant_velocity_world * duration.toSec();
    gtsam::Matrix3 scaled_cov = vel_noise_model_ * duration.toSec() * duration.toSec();
    // ROS_INFO("Tij between %f: [%f, %f, %f]",
    //     ros::Duration(final_time.toSec()-initial_time.toSec()).toSec(),
    //     translation.x(), translation.y(), translation.z());
    return std::make_pair(translation, scaled_cov); // Use the constant velocity model
  } else {
    // Take the average velocity from the DVL buffer
    gtsam::Vector3 delta_position = gtsam::Vector3::Zero();
    gtsam::Matrix3 delta_position_cov = gtsam::Matrix3::Zero();
    ros::Time last_measurement_time = initial_time;
    bool initialized = false;
    while (!dvl_buffer_.empty()) {
      const auto& dvl_msg = dvl_buffer_.front();
      ros::Time t = dvl_msg.header.stamp;
      if (t < initial_time) {
        dvl_buffer_.pop_front(); // Remove old messages
        continue;
      } else if (t > final_time) {
        break; // Stop processing if we exceed the final time
      } else {
        if (!initialized) {
          last_measurement_time = t;
          initialized = true;
          dvl_buffer_.pop_front(); // Skip the first one (no dt)
          continue;
      }
      double dt = (t - last_measurement_time).toSec();
      if (dt <= 0.0) {
        ROS_WARN("Non-positive dt (%f), skipping DVL message", dt);
        dvl_buffer_.pop_front();
        continue;
      }
      gtsam::Vector3 v(dvl_msg.twist.linear.x,
                       dvl_msg.twist.linear.y,
                       dvl_msg.twist.linear.z);
      delta_position += v * dt;
      delta_position_cov += vel_noise_model_ * dt * dt;
      last_measurement_time = t;
      dvl_buffer_.pop_front();
    }
  }
  if (initialized) {
      // Return the position delta and covariance
      // ROS_INFO("Tij between %f: [%f, %f, %f]",
      //         ros::Duration(final_time.toSec()-initial_time.toSec()).toSec(),
      //         delta_position.x(), delta_position.y(), delta_position.z());
      return std::make_pair(delta_position, delta_position_cov);
    } else {
      ROS_WARN("No valid DVL messages found between %f and %f, returning zero delta position",
              initial_time.toSec(), final_time.toSec());
      return std::make_pair(gtsam::Vector3::Zero(), gtsam::Matrix3::Identity() * 0.1);
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
  gtsam::Vector3 rpy = deltaRij.rpy();
  // ROS_INFO("Preintegrated rotation: deltaTij = %f, deltaRij (RPY) = [%f, %f, %f]",
  //          deltaTij,
  //          rpy.x() * 180.0 / M_PI,
  //          rpy.y() * 180.0 / M_PI,
  //          rpy.z() * 180.0 / M_PI);
  return std::make_tuple(deltaTij, deltaRij, deltaRotCovij);
}

// propagateState
std::pair<gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::propagateState(
    const std::pair<gtsam::Pose3, gtsam::Matrix6>& initial_state_and_cov,
    double deltaTij,                      // Time difference between the initial and final states
    const gtsam::Rot3& deltaRij,          // Rotation, in body frame
    const gtsam::Matrix3& deltaRotCovij,  // Covariance of the rotation
    const gtsam::Vector3& deltaTransij,   // Translation, in world frame
    const gtsam::Matrix3& deltaTransCovij // Covariance of the translation
  ) {
  //gtsam::Rot3 R_ij_w = deltaRij; // Rotation in world frame
  gtsam::Rot3 R_ij_w = R_wb_i_ * deltaRij * R_wb_i_.inverse(); // Rotate the delta rotation into the world frame
  // Log the delta rotation in world frame
  gtsam::Vector3 rpy = R_ij_w.rpy();
  // ROS_INFO("Delta rotation in world frame (RPY): [%f, %f, %f]",
  //          rpy.x() * 180.0 / M_PI,
  //          rpy.y() * 180.0 / M_PI,
  //          rpy.z() * 180.0 / M_PI);
  gtsam::Pose3 T_ij_w(R_ij_w, deltaTransij); // Create the pose in world frame
  gtsam::Matrix3 RotCovij_w = R_wb_i_.matrix() * deltaRotCovij * R_wb_i_.matrix().transpose(); // Rotate the covariance into the world frame

  // Extract the initial state and covariance
  // gtsam::Pose3 initial_pose= initial_state_and_cov.first;
  // gtsam::Matrix6 initial_cov = initial_state_and_cov.second;
  // Calculate the new pose, given the initial state
  // gtsam::Pose3 initial_pose = gtsam::Pose3(initial_state.attitude(), initial_state.position());
  // Calculate the new pose after applying the delta rotation and model velocity

  // Rotate the translation into the rotation frame
  // gtsam::Rot3 R_ij = initial_pose.rotation().between(deltaRij);
  // gtsam::Vector3 deltaTransij_rotated = R_ij.rotate(deltaTransij);
  // gtsam::Pose3 predicted_pose = initial_pose.compose(gtsam::Pose3(deltaRij, deltaTransij_rotated));

  // Log the predicted pose
  // ROS_INFO("Predicted pose: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
  //          predicted_pose.translation().x(), predicted_pose.translation().y(), predicted_pose.translation().z(),
  //          predicted_pose.rotation().toQuaternion().w(), predicted_pose.rotation().toQuaternion().x(),
  //          predicted_pose  .rotation().toQuaternion().y(), predicted_pose.rotation().toQuaternion().z());
  // gtsam::Pose3 predicted_state(predicted_pose.rotation(), predicted_pose.translation());
  // Calculate the relative pose
  // gtsam::Pose3 rel_pose = initial_pose.between(predicted_pose);
  gtsam::Matrix6 Ad_T_ij_w = T_ij_w.AdjointMap();
  // Compute the covariance of the predicted state
  gtsam::Matrix6 initial_cov = gtsam::Matrix6::Zero();
  gtsam::Matrix6 Q = gtsam::Matrix6::Zero();
  Q.block<3, 3>(0, 0) = deltaRotCovij;
  Q.block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
  Q.block<3, 3>(3, 0) = gtsam::Matrix3::Zero();
  Q.block<3, 3>(3, 3) = deltaTransCovij;
  gtsam::Matrix6 predicted_cov = Ad_T_ij_w * initial_cov * Ad_T_ij_w.transpose() + Q;
  // Then set the R_wb_i_ to the current orientation
  R_wb_i_ = current_orientation_;
  // std::stringstream ss5;
  // ss5 << "Covariance Q:\n" << Q;
  // ROS_INFO_STREAM(ss5.str());
  // gtsam::Matrix6 predicted_cov = rel_adjoint * initial_cov * rel_adjoint.transpose() + Q;
  // std::stringstream ss6;
  // ss6 << "Predicted Covariance:\n" << predicted_cov;
  // ROS_INFO_STREAM(ss6.str());
  return std::make_pair(T_ij_w, predicted_cov);
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
  } else {
    //ROS_INFO("Preintegrating IMU data from %f to %f", initial_time.toSec(), final_time.toSec());
    // Get the initial NavState and covariance
    auto initial_state_and_cov = initStateAndCovariance();
    gtsam::Pose3 initial_state = initial_state_and_cov.first;
    gtsam::Matrix6 initial_cov = initial_state_and_cov.second;
    // ROS_INFO("Initial state: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
    //          initial_state.position().x(), initial_state.position().y(), initial_state.position().z(),
    //          initial_state.attitude().toQuaternion().w(), initial_state.attitude().toQuaternion().x(),
    //          initial_state.attitude().toQuaternion().y(), initial_state.attitude().toQuaternion().z());
    // std::stringstream ss1;
    // ss1 << "Initial Covariance:\n" << initial_cov;
    // ROS_INFO_STREAM(ss1.str());
    // Get the average velocity between the initial and final times from the DVL buffer
    auto preint_translation_and_cov = getPreintegratedTranslation(initial_time, final_time);
    gtsam::Vector3 deltaTransij = preint_translation_and_cov.first;
    gtsam::Matrix3 deltaTransCovij = preint_translation_and_cov.second;
    // ROS_INFO("Average velocity: [%f, %f, %f]",
    //          avg_velocity.x(), avg_velocity.y(), avg_velocity.z());
    // std::stringstream ss2;
    // ss2 << "Velocity Covariance:\n" << avg_velocity_cov;
    // ROS_INFO_STREAM(ss2.str());
    // Get the preintegrated rotation and covariance
    auto preint_results = getPreintegratedRotation(initial_time, final_time);
    double deltaTij = std::get<0>(preint_results);
    gtsam::Rot3 deltaRij = std::get<1>(preint_results);
    // std::stringstream ss3;
    // ss3 << "Preintegrated Rotation:\n" << deltaRij.matrix();
    // ROS_INFO_STREAM(ss3.str());
    gtsam::Matrix3 deltaRotCovij = std::get<2>(preint_results);
    // ROS_INFO("Preintegrated rotation: deltaTij = %f", deltaTij);
    // std::stringstream ss4;
    // ss4 << "Delta Rotation Covariance:\n" << deltaRotCovij;
    // ROS_INFO_STREAM(ss4.str());
    // Get the predicted NavState and covariance
    std::pair<gtsam::Pose3, gtsam::Matrix6> predicted_state_and_cov;
    predicted_state_and_cov = propagateState(
        initial_state_and_cov, deltaTij, deltaRij, deltaRotCovij, deltaTransij, deltaTransCovij);
    return std::make_pair(
        predicted_state_and_cov.first, predicted_state_and_cov.second);
  }
  // // If we reach here, something went wrong
  // ROS_ERROR("Failed to compute relative pose between states");
  // return std::make_pair(gtsam::Pose3(), gtsam::Matrix6::Zero());
}

// handlePreintegrate
bool ImuPreintegratorNode::handlePreintegrate(
    spurdog_acomms::PreintegrateImu::Request &req,
    spurdog_acomms::PreintegrateImu::Response &res) {
  // Call getRelativePoseBetweenStates to get the relative pose and covariance
  auto relativePoseCov = getRelativePoseBetweenStates(req.initial_time, req.final_time);
  gtsam::Pose3 relativePose = relativePoseCov.first;
  gtsam::Matrix6 relativeCov = relativePoseCov.second;
  // get the relative rotation in the world frame in rpy angles
  gtsam::Vector3 rpy = relativePose.rotation().rpy();
  // Log the relative pose and covariance
  // ROS_INFO("Relative pose:  position [%f, %f, %f], orientation [%f, %f, %f]",
  //          relativePose.translation().x(), relativePose.translation().y(), relativePose.translation().z(),
  //          rpy.x() * 180.0 / M_PI,
  //          rpy.y() * 180.0 / M_PI,
  //          rpy.z() * 180.0 / M_PI);
  // Get the dead reckoning state and covariance
  gtsam::Pose3 initialPose = std::get<1>(dr_state_and_cov_);
  gtsam::Matrix6 initialCov = std::get<2>(dr_state_and_cov_);
  // ROS_INFO("Initial dead reckoning pose: position [%f, %f, %
  auto predicted_state_and_cov = deadReckonFromPreintegrate(req.final_time, relativePose, relativeCov, initialPose, initialCov);
  dr_state_and_cov_ = std::make_tuple(req.final_time, predicted_state_and_cov.first, predicted_state_and_cov.second);
  gtsam::Pose3 finalPose = std::get<1>(dr_state_and_cov_);
  gtsam::Matrix6 finalCov = std::get<2>(dr_state_and_cov_);
  // ROS_INFO("Last MOOSNav report: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
  //          last_nav_report_.second.translation().x(), last_nav_report_.second.translation().y(),
  //          last_nav_report_.second.translation().z(),
  //          last_nav_report_.second.rotation().toQuaternion().w(), last_nav_report_.second.rotation().toQuaternion().x(),
  //          last_nav_report_.second.rotation().toQuaternion().y(), last_nav_report_.second.rotation().toQuaternion().z());
  // ROS_INFO("Initial reckoned pose: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
  //          initialPose.translation().x(), initialPose.translation().y(), initialPose.translation().z(),
  //          initialPose.rotation().toQuaternion().w(), initialPose.rotation().toQuaternion().x(),
  //          initialPose.rotation().toQuaternion().y(), initialPose.rotation().toQuaternion().z());
  ROS_INFO("Final reckoned pose: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
           finalPose.translation().x(), finalPose.translation().y(), finalPose.translation().z(),
           finalPose.rotation().toQuaternion().w(), finalPose.rotation().toQuaternion().x(),
           finalPose.rotation().toQuaternion().y(), finalPose.rotation().toQuaternion().z());
  // Log the relative pose and covariance
  // ROS_INFO("Relative pose between last nav report and dead reckoned pose: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
  //          relposeNavDR.translation().x(), relposeNavDR.translation().y(), relposeNavDR.translation().z(),
  //          relposeNavDR.rotation().toQuaternion().w(), relposeNavDR.rotation().toQuaternion().x(),
  //          relposeNavDR.rotation().toQuaternion().y(), relposeNavDR.rotation().toQuaternion().z());
  // Calculate the relative pose between the dead reckoned pose and the last nav report
  // gtsam::Pose3 relativePoseToNav = last_nav_report_.second.between(dr_pose);
  //   ROS_INFO("Pose Error: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
  //          relativePoseToNav.translation().x(), relativePoseToNav.translation().y(), relativePoseToNav.translation().z(),
  //          relativePoseToNav.rotation().toQuaternion().w(), relativePoseToNav.rotation().toQuaternion().x(),
  //          relativePoseToNav.rotation().toQuaternion().y(), relativePoseToNav.rotation().toQuaternion().z());
  // Log the relative pose and covariance
  // std::stringstream ss5;
  // ss5 << "Dead Reckoned Covariance:\n" << dr_cov;
  // ROS_INFO_STREAM(ss5.str());

  // Add pose results to the map
  dead_reckon_map_[req.final_time] = finalPose;
  nav_state_map_[last_nav_report_.first] = last_nav_report_.second;
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

// Dead reckoning from preintegrated state
std::pair<gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::deadReckonFromPreintegrate(
    const ros::Time& final_time, const gtsam::Pose3& preint_pose, const gtsam::Matrix6& preint_cov,
    const gtsam::Pose3& initial_pose, const gtsam::Matrix6& initial_cov) {

  // Compose the preintegrated pose with the dr_state_and_cov_ pose
  gtsam::Pose3 new_pose = initial_pose.compose(preint_pose);
  // Propogate the covariance
  gtsam::Matrix6 adjoint = preint_pose.AdjointMap();
  gtsam::Matrix6 new_cov = adjoint * initial_cov * adjoint.transpose() + preint_cov;
  // Log the final time
  return std::make_pair(new_pose, new_cov);
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
  signal(SIGINT, onShutdownCallback);  // Catch Ctrl+C and cleanly shutdown
  ros::spin();
  return 0;
}