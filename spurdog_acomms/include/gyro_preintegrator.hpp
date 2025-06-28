#ifndef GYRO_PREINTEGRATOR_HPP
#define GYRO_PREINTEGRATOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <spurdog_acomms/PreintegrateImu.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <memory>
#include <deque>
#include <mutex>

// GTSAM headers
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <Eigen/Eigenvalues>

class ImuPreintegratorNode {
private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber dvl_vel_sub_;
  ros::Subscriber nav_state_sub_;
  ros::Subscriber in_water_sub_;
  ros::ServiceServer preint_srv_;

  ros::Time ti_;
  gtsam::Matrix3 gyro_noise_;
  gtsam::Vector3 gyro_bias_;
  gtsam::Vector3 constant_vel_model_;
  gtsam::Matrix3 constant_vel_noise_model_;
  gtsam::Matrix3 dvl_noise_model_;
  bool in_water_;
  std::deque<sensor_msgs::Imu> imu_buffer_;
  std::deque<geometry_msgs::TwistStamped> dvl_buffer_;
  std::tuple<ros::Time, gtsam::Pose3, gtsam::Matrix6> dr_state_and_cov_;
  std::pair<ros::Time, gtsam::Pose3> last_nav_report_;
  std::map<ros::Time, gtsam::Pose3> dead_reckon_map_;
  std::map<ros::Time, gtsam::Pose3> nav_state_map_;

public:
  ImuPreintegratorNode();
  // Callbacks
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void navStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void inWaterCallback(const std_msgs::Bool::ConstPtr& msg);
  void onShutdownCallback();

  // Integration methods
  std::tuple<double, gtsam::Rot3, gtsam::Matrix3, gtsam::Rot3> getPreintegratedRotation(
    const ros::Time& imu_ti, const ros::Time& imu_tj);
  std::tuple<double, gtsam::Pose3, gtsam::Matrix6, gtsam::Rot3> getPreintegratedPose(
      const gtsam::Vector3& velocity, const gtsam::Matrix3& vel_noise_model,
      const ros::Time& vel_ti, const ros::Time& vel_tj);
  std::tuple<double, gtsam::Pose3, gtsam::Matrix6> getRelativePoseBetween(
      const ros::Time& initial_time, const ros::Time& final_time);
  // State propagation methods
  bool handlePreintegrate(
      spurdog_acomms::PreintegrateImu::Request &req,
      spurdog_acomms::PreintegrateImu::Response &res);
  std::pair<gtsam::Pose3, gtsam::Matrix6> deadReckonFromPreintegrate(
      const ros::Time& final_time, const gtsam::Pose3& preint_pose, const gtsam::Matrix6& preint_cov);
  void writePoseResultsToTum(const std::string& filename);
  gtsam::Matrix6 makePSD(const gtsam::Matrix6& input);
  gtsam::Rot3 convertVehicleNEDToWorldENU(const gtsam::Rot3& input);
};

#endif  // GYRO_PREINTEGRATOR_HPP
