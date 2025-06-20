#ifndef IMU_PREINTEGRATOR_HPP
#define IMU_PREINTEGRATOR_HPP

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
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/NavState.h>

class ImuPreintegratorNode {
private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::ServiceServer preint_srv_;
  ros::Subscriber dvl_vel_sub_;
  ros::Subscriber nav_state_sub_;
  ros::Subscriber in_water_sub_;

  ros::Time ti_;
  ros::Time start_time_;
  gtsam::Matrix3 gyro_noise_;
  gtsam::Vector3 bias_;
  gtsam::Vector3 dvl_vel_bw_;
  gtsam::Pose3 last_nav_report_;
  bool in_water_;
  std::map<ros::Time, gtsam::NavState> nav_state_map_;
  std::map<ros::Time, gtsam::Matrix6> nav_cov_map_;
  std::deque<sensor_msgs::Imu> imu_buffer_;
public:
  ImuPreintegratorNode();

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void navStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void inWaterCallback(const std_msgs::Bool::ConstPtr& msg);
  std::pair<gtsam::Vector3, gtsam::Matrix3> getVelocityModel() const;
  std::tuple<double, gtsam::Rot3, gtsam::Matrix3> preintegrateRotation(const ros::Time& last_time);
  gtsam::NavState getPredictedNavState(
      const double deltaTij, const gtsam::Rot3& deltaRij, const gtsam::Vector3& model_velocity);
  gtsam::Matrix6 getPredictedCovariance(
      const double deltaTij, const gtsam::NavState& predState,
      const gtsam::Matrix3& deltaRotCovij, const gtsam::Matrix3& vel_noise_model);
  void propogateState(ros::Time final_time);
  std::pair<gtsam::Pose3, gtsam::Matrix6> getRelativePoseBetweenStates(ros::Time final_time);
  bool handlePreintegrate(
      spurdog_acomms::PreintegrateImu::Request &req,
      spurdog_acomms::PreintegrateImu::Response &res);
};

#endif  // IMU_PREINTEGRATOR_HPP
