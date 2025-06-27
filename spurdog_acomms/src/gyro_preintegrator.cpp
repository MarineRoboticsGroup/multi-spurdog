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
    gtsam::Vector3 dvl_noise_sigmas(0.1, 0.1, 0.1); // Per Ray
    dvl_noise_model_ = dvl_noise_sigmas.cwiseProduct(dvl_noise_sigmas).asDiagonal();
  } else {
    ROS_WARN("Velocity source not set to DVL, using default constant velocity");
    constant_vel_model_ = gtsam::Vector3(0.8, 0.0, 0.0); // Default to typical velocity
    gtsam::Vector3 constant_vel_noise_sigmas(0.5, 0.25, 0.1); // Cautiously set to 5x DVL
    constant_vel_noise_model_ = constant_vel_noise_sigmas.cwiseProduct(constant_vel_noise_sigmas).asDiagonal();
  }

  // Initialize the buffers
  ti_ = ros::Time(0); // Initialize the time to zero
  in_water_ = false; // Initialize the in-water status
  imu_buffer_ = std::deque<sensor_msgs::Imu>();
  dvl_buffer_ = std::deque<geometry_msgs::TwistStamped>();
  dr_state_and_cov_ = std::make_tuple(ros::Time(0), gtsam::Pose3(), gtsam::Matrix6::Identity().eval());

  ros::Duration(1.0).sleep(); // Wait  to allow for initial setup
}

// imuCallback
void ImuPreintegratorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_buffer_.push_back(*msg);
  if (ti_ == ros::Time(0)) {
    ti_ = msg->header.stamp;
    ROS_WARN("First IMU message received at time: %f", ti_.toSec());
  }
}

// navStateCallback
void ImuPreintegratorNode::navStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Update the initial state and covariance based on the nav state
  ros::Time t = std::get<0>(dr_state_and_cov_);
  gtsam::Pose3 pose = std::get<1>(dr_state_and_cov_);
  gtsam::Matrix6 cov = std::get<2>(dr_state_and_cov_);
  // Get the pose from the message
  gtsam::Pose3 reported_pose = gtsam::Pose3(
      gtsam::Rot3::Quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
      gtsam::Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  // Rotate the reported pose from NED to ENU
  gtsam::Rot3 R_enu = convertVehicleNEDToWorldENU(reported_pose.rotation());
  reported_pose = gtsam::Pose3(R_enu, reported_pose.translation()); // Translation is in ENU, but rotation is NED?
  if (t == ros::Time(0) &&
      pose.equals(gtsam::Pose3()) &&  // Pose3 provides an equals() method
      cov.isApprox(gtsam::Matrix6::Identity())) {
    // Initialize the initial state and covariance
    gtsam::Vector6 initial_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
    gtsam::Matrix6 initial_cov = initial_cov_sigmas.cwiseProduct(initial_cov_sigmas).asDiagonal();
    dr_state_and_cov_ = std::make_tuple(msg->header.stamp, reported_pose, initial_cov);
    ROS_INFO("Initial state set at time: %f", msg->header.stamp.toSec());
  }
  last_nav_report_ = std::make_pair(msg->header.stamp, reported_pose);
}

// dvlVelCallback
void ImuPreintegratorNode::dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // Rotate the DVL from body to world frame using the current orientation
  gtsam::Vector3 dvl_velocity(
      msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  dvl_buffer_.push_back(*msg);
}

// inWaterCallback
void ImuPreintegratorNode::inWaterCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!in_water_) {
    // Set the dr_pose and covariance to the last nav report
    gtsam::Vector6 last_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
    gtsam::Matrix6 last_cov = last_cov_sigmas.cwiseProduct(last_cov_sigmas).asDiagonal();
    dr_state_and_cov_ = std::make_tuple(last_nav_report_.first, last_nav_report_.second, last_cov);
  } else if (!in_water_ && msg->data==true) { // Entering the water
    if (!imu_buffer_.empty()) {
      const sensor_msgs::Imu& last_msg = imu_buffer_.back();
      ti_ = last_msg.header.stamp;
      imu_buffer_.clear(); // Clear the IMU buffer to start fresh
      dvl_buffer_.clear(); // Clear the DVL velocity buffer as well
      ROS_WARN("Entering water, reset preintegration start time to %f", ti_.toSec());
    }else {
      ROS_WARN("IMU buffer is empty, cannot reset preintegration start time");
    }
  } else {
    // IN the water, do nothing
  }
  in_water_ = msg->data; // Update the in-water status
}

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

gtsam::Rot3 ImuPreintegratorNode::convertVehicleNEDToWorldENU(const gtsam::Rot3& R_ahrs) {
    gtsam::Rot3 R_decl = gtsam::Rot3::Rz((-M_PI * 14.1 / 180.0)); // declination correction 14.1degW
    // Get the vehicle rotation in NED frame for debugging
    gtsam::Rot3 R_ned_to_body = R_decl * R_ahrs; // Rotation from NED to Body
    gtsam::Vector3 vehicle_rpy_ned = R_ned_to_body.rpy() * 180.0 / M_PI; // Convert to degrees for logigng
    gtsam::Rot3 R_body_to_ned = R_ned_to_body.inverse(); // Rotation from Body to NED
    gtsam::Rot3 R_flip_z = gtsam::Rot3::Rx(M_PI); // Flip Z axis
    gtsam::Rot3 R_north_to_east = gtsam::Rot3::Rz(-M_PI / 2); // North to East rotation
    gtsam::Rot3 R_ned_enu = R_north_to_east * R_flip_z; // NED to ENU rotation
    gtsam::Rot3 Ri_wb_enu = R_ned_enu * R_body_to_ned; // Convert from NED to ENU
    // Log the conversion
    gtsam::Vector3 rpy = Ri_wb_enu.rpy() * 180.0 / M_PI; // Convert to degrees
    // ROS_INFO("Converted rotation from NED [%f, %f, %f] to ENU [%f, %f, %f]",
    //          vehicle_rpy_ned.x(), vehicle_rpy_ned.y(), vehicle_rpy_ned.z(),
    //          rpy.x(), rpy.y(), rpy.z());
    return Ri_wb_enu;
}

//getPreintegratedRotation
std::tuple<double, gtsam::Rot3, gtsam::Matrix3, gtsam::Rot3> ImuPreintegratorNode::getPreintegratedRotation(
  const ros::Time& imu_ti, const ros::Time& imu_tj) {
  // Check if the IMU buffer is empty
  if (imu_buffer_.empty()) {
    ROS_WARN("IMU buffer is empty, cannot preintegrate rotation");
    return std::make_tuple(0.0, gtsam::Rot3::Identity(), gtsam::Matrix3::Zero(), gtsam::Rot3::Identity());
  }
  // Create the preintegration parameters
  auto preint_rotation_params = boost::make_shared<gtsam::PreintegratedRotationParams>(
      gyro_noise_, gyro_bias_);
  // Create the preintegrated rotation object
  gtsam::PreintegratedRotation preint_rotation(preint_rotation_params);
  // Iterate through the IMU buffer and integrate the measurements
  ros::Time last_imu_time = imu_ti;
  gtsam::Matrix3 deltaRotCovij = gtsam::Matrix3::Zero();
  gtsam::Rot3 Ri_wb; // Initial rotation in world frame
  bool initialized = false;
  double count = 0.0; // Initialize mean dt
  // While imu buffer is not empty
  while (!imu_buffer_.empty()) {
    const auto& imu_msg = imu_buffer_.front();
    ros::Time t = imu_msg.header.stamp;
    // if (t < imu_ti) {
    //   ROS_WARN("IMU message at time %f is before the initial time %f, ignoring", t.toSec(), imu_ti.toSec());
    //   imu_buffer_.pop_front(); // Remove this message
    //   continue; // Skip this message
    if (t > imu_tj){
    //} else if (t > imu_tj) {
      break; // Stop integrating if we exceed the final time
    } else {
      if (!initialized) {
        last_imu_time = t;
        initialized = true;
        // Set the Rwb to the initial orientation
        Ri_wb = gtsam::Rot3::Quaternion(imu_msg.orientation.w,
                                       imu_msg.orientation.x,
                                       imu_msg.orientation.y,
                                       imu_msg.orientation.z);
        // Convert the vehicle NED to world ENU
        Ri_wb = convertVehicleNEDToWorldENU(Ri_wb); // Convert from NED to ENU
        // Notes: -decl is good, -z, +x looks good for first 200sec, then breaks down
        imu_buffer_.pop_front(); // Skip the first one (no dt)
        continue;
      }
      double dt = (t - last_imu_time).toSec();
      if (dt <= 0.0) {
        ROS_WARN("Non-positive dt (%f), skipping IMU message", dt);
        imu_buffer_.pop_front();
        continue;
      }
      gtsam::Vector3 omega(imu_msg.angular_velocity.x,
                           imu_msg.angular_velocity.y,
                           imu_msg.angular_velocity.z);
      preint_rotation.integrateMeasurement(omega, gyro_bias_, dt);
      count += 1;
      last_imu_time = t;
      imu_buffer_.pop_front();
    }
  }
  // Get the integrated rotation and covariance
  double deltaTij = preint_rotation.deltaTij();
  double mean_dt = count > 0 ? deltaTij / count : 0.0; // Calculate the mean dt
  gtsam::Matrix3 H; // H is dR/db
  gtsam::Rot3 deltaRij = preint_rotation.biascorrectedDeltaRij(gyro_bias_, &H);
  gtsam::Matrix3 gyro_noise_hz = gyro_noise_ * mean_dt; // Convert to rad^2/s^2
  deltaRotCovij = H * gyro_noise_hz * H.transpose();
  preint_rotation.resetIntegration();
  return std::make_tuple(deltaTij, deltaRij, deltaRotCovij, Ri_wb);
}

std::tuple<double, gtsam::Pose3, gtsam::Matrix6, gtsam::Rot3> ImuPreintegratorNode::getPreintegratedPose(
  const gtsam::Vector3& velocity, const gtsam::Matrix3& vel_noise_model,
  const ros::Time& vel_ti, const ros::Time& vel_tj) {
  // Get the preintegrated rotation/cov in the body frame:
  std::tuple<double, gtsam::Rot3, gtsam::Matrix3, gtsam::Rot3> preintegrated_rotation;
  preintegrated_rotation = getPreintegratedRotation(vel_ti, vel_tj);
  double deltaTij = std::get<0>(preintegrated_rotation);
  gtsam::Rot3 deltaRij = std::get<1>(preintegrated_rotation);
  gtsam::Matrix3 deltaRotCovij = std::get<2>(preintegrated_rotation);
  gtsam::Rot3 Ri_wb = std::get<3>(preintegrated_rotation); // Initial rotation in world frame
  // Get the preintegrated translation/cov in the world frame:
  gtsam::Vector3 deltaTransij;
  gtsam::Matrix3 deltaTransCovij;
  deltaTransij = gtsam::Vector3(velocity.x() * deltaTij,
                                velocity.y() * deltaTij,
                                velocity.z() * deltaTij);
  deltaTransCovij = vel_noise_model * deltaTij;
  // Convert the translation into ENU
  // deltaTransij = Ri_wb.rotate(deltaTransij); // Rotate the translation into the world frame
  // Build the complete preintegrated pose
  //gtsam::Pose3 Tij_b = gtsam::Pose3(Ri_wb, deltaTransij); // Pose in body frame
  gtsam::Pose3 Tij_b = gtsam::Pose3(deltaRij, deltaTransij); // Pose in body frame
  // Build the complete preintegrated covariance
  gtsam::Matrix6 Covij_b = gtsam::Matrix6::Zero();
  Covij_b.block<3, 3>(0, 0) = deltaRotCovij; // Covariance of the rotation
  Covij_b.block<3, 3>(0, 3) = gtsam::Matrix3::Zero(); // Cross-covariance
  Covij_b.block<3, 3>(3, 0) = gtsam::Matrix3::Zero(); // Cross-covariance
  Covij_b.block<3, 3>(3, 3) = deltaTransCovij; // Covariance of the translation
  return std::make_tuple(deltaTij, Tij_b, Covij_b, Ri_wb);
}
// getRelativePoseBetweenStates
std::tuple<double, gtsam::Pose3, gtsam::Matrix6> ImuPreintegratorNode::getRelativePoseBetween(
    const ros::Time& initial_time, const ros::Time& final_time) {
  double requested_deltaTij = ros::Duration(final_time - initial_time).toSec();
  double actual_deltaTij = 0.0;
  gtsam::Pose3 relPose = gtsam::Pose3();
  gtsam::Matrix6 relCov = gtsam::Matrix6::Zero();
  gtsam::Rot3 Ri_wb = gtsam::Rot3::Identity(); // Initial orientation in world frame
  // Check the velocity source
  if (!in_water_) {
    gtsam::Vector6 surface_cov_sigmas = (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1.5, 1.5, 0.1).finished();
    gtsam::Matrix6 surface_cov = surface_cov_sigmas.cwiseProduct(surface_cov_sigmas).asDiagonal();
    // Not in water, return zero velocity model
    actual_deltaTij = requested_deltaTij;
    relPose = gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.0, 0.0, 0.0));
    relCov = surface_cov; // Use the surface covariance
  } else if (dvl_buffer_.empty()) {
    // Integrate between the initial and final times using the constant velocity model
    auto preintegrated_pose = getPreintegratedPose(
        constant_vel_model_, constant_vel_noise_model_, initial_time, final_time);
    actual_deltaTij = std::get<0>(preintegrated_pose);
    relPose = std::get<1>(preintegrated_pose);
    relCov = std::get<2>(preintegrated_pose);
    Ri_wb = std::get<3>(preintegrated_pose);
  } else {
    // Take the average velocity from the DVL buffer
    ros::Time last_dvl_time = initial_time;
    bool initialized = false;
    bool Rwb_initialized = false;
    while (!dvl_buffer_.empty()) {
      const auto& dvl_msg = dvl_buffer_.front();
      ros::Time t = dvl_msg.header.stamp;
      if (t < initial_time) {
        ROS_WARN("DVL message at time %f is before the initial time %f, ignoring", t.toSec(), initial_time.toSec());
        dvl_buffer_.pop_front(); // Remove old messages
        continue;
      } else if (t > final_time) {
        break; // Stop processing if we exceed the final time
      } else {
        if (!initialized) {
          last_dvl_time = t;
          initialized = true;
          dvl_buffer_.pop_front(); // Skip the first one (no dt)
          continue;
        }
        double dt = (t - last_dvl_time).toSec();
        if (dt <= 0.0) {
          ROS_WARN("Non-positive dt (%f), skipping DVL message", dt);
          dvl_buffer_.pop_front();
          continue;
        }
        // Integrate the Rotation between the last_dvl_time and t
        gtsam::Vector3 v(dvl_msg.twist.linear.x,
                          dvl_msg.twist.linear.y,
                          dvl_msg.twist.linear.z);
        auto preintegrated_pose = getPreintegratedPose(
            v, dvl_noise_model_, last_dvl_time, t);
        if (!Rwb_initialized) {
          Ri_wb = std::get<3>(preintegrated_pose); // Set the initial orientation in world frame
          Rwb_initialized = true; // Mark the initial orientation as set
        }
        actual_deltaTij += std::get<0>(preintegrated_pose);
        // Compose the new preintegrated pose with the previous one
        gtsam::Pose3 new_relPose;
        gtsam::Matrix6 new_relCov;
        //gtsam::Matrix3 Ji, Jj; // Jacobians for composing the poses
        //new_relPose = relPose.compose(std::get<1>(preintegrated_pose), Ji, Jj);
        gtsam::Matrix66 Ji, Jj;
        gtsam::Pose3 new_pose = relPose.compose(std::get<1>(preintegrated_pose),
                                             gtsam::OptionalJacobian<6,6>(Ji),
                                             gtsam::OptionalJacobian<6,6>(Jj));
        new_relCov = Ji * relCov * Ji.transpose() +
                                    Jj * std::get<2>(preintegrated_pose) * Jj.transpose();
        // Re-write the relative pose and covariance
        relPose = new_pose;
        relCov = new_relCov;
        last_dvl_time = t;
        dvl_buffer_.pop_front();
      }
    }
  }
  // Rotate the relative pose and covariance from body to world frame
  gtsam::Rot3 Rij_b = relPose.rotation();
  gtsam::Point3 tij_b = relPose.translation();
  // ROS_INFO("Relative pose in body frame: rotation [%f, %f, %f], translation [%f, %f, %f]",
  //          Rij_b.rpy().x()*(180/M_PI), Rij_b.rpy().y()*(180/M_PI), Rij_b.rpy().z()*(180/M_PI),
  //          tij_b.x(), tij_b.y(), tij_b.z());
  gtsam::Pose3 T_wb(Ri_wb, gtsam::Point3(0.0, 0.0, 0.0)); // Pose from world to body
  gtsam::Pose3 Tij_w = T_wb * relPose * T_wb.inverse(); // Relative pose in world frame
  ROS_INFO("Relative pose in world frame: rotation [%f, %f, %f], translation [%f, %f, %f]",
           Tij_w.rotation().rpy().x()*(180/M_PI), Tij_w.rotation().rpy().y()*(180/M_PI), Tij_w.rotation().rpy().z()*(180/M_PI),
           Tij_w.translation().x(), Tij_w.translation().y(), Tij_w.translation().z());
  // gtsam::Rot3 Rij_w = relPose.rotation();
  // gtsam::Point3 tij_w = relPose.translation();
  gtsam::Matrix6 Covij_b = relCov; // Covariance in body frame
  //gtsam::Rot3 Rij_w = Ri_wb * Rij_b * Ri_wb.inverse(); // Rotate the delta rotation into the world frame
  //gtsam::Point3 tij_w = Ri_wb.rotate(tij_b); // Rotate the translation into the world frame
  //gtsam::Pose3 Tij_w(Rij_w, tij_w); // Create the pose in world frame
  // Rotate the covariance from body to world frame
  //gtsam::Matrix6 Ad_wb = gtsam::Pose3(Ri_wb, gtsam::Point3()).AdjointMap(); // Adjoint map for the rotation
  gtsam::Matrix6 Ad_wb = T_wb.AdjointMap(); // Adjoint map for the rotation
  gtsam::Matrix6 Covij_w = Ad_wb * Covij_b * Ad_wb.transpose(); // Rotate the covariance
  // Ensure the covariance is positive semi-definite
  Covij_w = makePSD(Covij_w);
  return std::make_tuple(actual_deltaTij, Tij_w, Covij_w);
}

// handlePreintegrate
bool ImuPreintegratorNode::handlePreintegrate(
    spurdog_acomms::PreintegrateImu::Request &req,
    spurdog_acomms::PreintegrateImu::Response &res) {
  // Call getRelativePoseBetweenStates to get the relative pose and covariance
  auto relativePoseCov = getRelativePoseBetween(req.initial_time, req.final_time);
  double deltaTij = std::get<0>(relativePoseCov);
  gtsam::Pose3 relativePose = std::get<1>(relativePoseCov);
  gtsam::Matrix6 relativeCov = std::get<2>(relativePoseCov);
  // Log the preintegration results
  gtsam::Vector3 rpy = relativePose.rotation().rpy();
  // Attempt to DR
  auto predicted_state_and_cov = deadReckonFromPreintegrate(req.final_time, relativePose, relativeCov);
  // Log the new state
  gtsam::Pose3 finalPose = std::get<0>(predicted_state_and_cov);
  gtsam::Matrix6 dr_cov = std::get<1>(predicted_state_and_cov);
  gtsam::Vector3 final_rpy = finalPose.rotation().rpy();
  // ROS_INFO("Final reckoned pose: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
  //          finalPose.translation().x(), finalPose.translation().y(), finalPose.translation().z(),
  //          finalPose.rotation().toQuaternion().w(), finalPose.rotation().toQuaternion().x(),
  //          finalPose.rotation().toQuaternion().y(), finalPose.rotation().toQuaternion().z());
  // ROS_INFO("Last Nav Report: position [%f, %f, %f], orientation [%f, %f, %f, %f]",
  //          last_nav_report_.second.translation().x(), last_nav_report_.second.translation().y(),
  //          last_nav_report_.second.translation().z(),
  //          last_nav_report_.second.rotation().toQuaternion().w(),
  //          last_nav_report_.second.rotation().toQuaternion().x(),
  //          last_nav_report_.second.rotation().toQuaternion().y(),
  //          last_nav_report_.second.rotation().toQuaternion().z());
  // Log the covariance
  // std::stringstream ss5;
  // ss5 << "Dead Reckoned Covariance:\n" << dr_cov;
  // ROS_INFO_STREAM(ss5.str());

  // Update the last nav report with the final pose
  // Get the final pose and covariance
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
    const ros::Time& final_time, const gtsam::Pose3& preint_pose, const gtsam::Matrix6& preint_cov) {
  // Get the intial pose and covariance from the dr_state_and_cov_
  gtsam::Pose3 initial_pose = std::get<1>(dr_state_and_cov_);
  gtsam::Matrix6 initial_cov = std::get<2>(dr_state_and_cov_);
  // Compose the preintegrated pose with the dr_state_and_cov_ pose
  gtsam::Matrix66 Ji, Jj;
  gtsam::Pose3 new_pose = initial_pose.compose(preint_pose,
                                        gtsam::OptionalJacobian<6,6>(Ji),
                                        gtsam::OptionalJacobian<6,6>(Jj));
  gtsam::Matrix6 new_cov = Ji * initial_cov * Ji.transpose() +
                                  Jj * preint_cov * Jj.transpose();
  // std::stringstream ssInitCov;
  // ssInitCov << "Initial Covariance:\n" << initial_cov;
  // ROS_INFO_STREAM(ssInitCov.str());
  // std::stringstream ssPreintCov;
  // ssPreintCov << "Preintegrated Covariance:\n" << preint_cov;
  // ROS_INFO_STREAM(ssPreintCov.str());
  // std::stringstream ssNewCov;
  // ssNewCov << "New Covariance:\n" << new_cov;
  // ROS_INFO_STREAM(ssNewCov.str());

  // Update DR state and covariance
  dr_state_and_cov_ = std::make_tuple(final_time, new_pose, new_cov);
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