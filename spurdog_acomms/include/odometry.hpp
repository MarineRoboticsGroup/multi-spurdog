#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <spurdog_acomms/PreintegrateImu.h>
#include <spurdog_acomms/Bar30SoundSpeed.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
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
  ros::Publisher dr_state_pub_;
  ros::ServiceServer preint_srv_;

  ros::Time ti_;
  gtsam::Matrix3 gyro_noise_;
  gtsam::Vector3 gyro_bias_;
  gtsam::Vector3 constant_vel_model_;
  gtsam::Matrix3 constant_vel_noise_model_;
  gtsam::Matrix3 dvl_noise_model_;
  bool in_water_;
  std::deque<sensor_msgs::Imu> imu_buffer_;
  std::deque<geometry_msgs::TwistWithCovarianceStamped> vel_b_buffer_;
  std::deque<geometry_msgs::TwistWithCovarianceStamped> vel_w_buffer_;
  std::tuple<ros::Time, gtsam::Pose3, gtsam::Matrix6> dr_state_and_cov_;
  gtsam::Rot3 R_ned_;
  gtsam::Rot3 R_enu_;
  gtsam::Vector3 Omega_ned_;
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
  std::tuple<double, gtsam::Rot3, gtsam::Matrix3> getPreintegratedRotation(
    const ros::Time& ti, const ros::Time& tj);
  std::tuple<double, gtsam::Point3, gtsam::Matrix3, gtsam::Matrix3> getPreintegratedTranslation(
      const ros::Time& imu_ti, const ros::Time& imu_tj);
  std::tuple<double, gtsam::Pose3, gtsam::Matrix6> getPreintegratedPose(
      const std::tuple<double, gtsam::Rot3, gtsam::Matrix3>& preint_rotation,
      const std::tuple<double, gtsam::Point3, gtsam::Matrix3, gtsam::Matrix3>& preint_translation);
  std::tuple<double, gtsam::Pose3, gtsam::Matrix6> getRelativePoseBetween(
      const ros::Time& initial_time, const ros::Time& final_time);
  // State propagation methods
  bool handlePreintegrate(
      spurdog_acomms::PreintegrateImu::Request &req,
      spurdog_acomms::PreintegrateImu::Response &res);
  std::pair<gtsam::Pose3, gtsam::Matrix6> deadReckonFromPreintegrate(
      const ros::Time& initial_time, const ros::Time& final_time,
      const gtsam::Pose3& Ti_w, const gtsam::Matrix6& Sigmai_w,
      const gtsam::Pose3& Tij_b, const gtsam::Matrix6& SigmaTij_b);
  void writePoseResultsToTum(const std::string& filename);
  gtsam::Matrix6 makePSD(const gtsam::Matrix6& input);
  gtsam::Rot3 convertRotNEDtoENU(const gtsam::Rot3& R_ned);
  gtsam::Matrix3 convertCovNEDToENU(const gtsam::Rot3& R_ned, gtsam::Matrix3& cov_ned);
  gtsam::Pose3 convertVehicleNEDToVehicleENU(const gtsam::Pose3& T_ned);
};

#endif  // ODOMETRY_HPP
