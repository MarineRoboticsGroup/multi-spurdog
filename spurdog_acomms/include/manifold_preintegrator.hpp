#ifndef MANIFOLD_PREINTEGRATOR_HPP
#define MANIFOLD_PREINTEGRATOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <spurdog_acomms/PreintegrateImu.h>

#include <memory>
#include <deque>
#include <mutex>

// GTSAM headers
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

class ImuPreintegratorNode {
private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber nav_state_sub_;
  ros::Subscriber dvl_vel_sub_;
  ros::Subscriber in_water_sub;
  ros::ServiceServer preint_srv_;
  std::mutex buffer_mutex_;
  std::deque<sensor_msgs::Imu> imu_buffer_;
  ros::Time ti_;
  bool in_water_;
  gtsam::Quaternion imu_q_bw_;
  double baro_depth_w_;
  gtsam::Vector3 dvl_vel_bw_;
  gtsam::imuBias::ConstantBias bias_;
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise_;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> pim_params_;
  // Define a preintegration object
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
  std::map<ros::Time, size_t> time_to_factor_index_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_;
  std::vector<gtsam::NavState> preint_states_;

public:
  ImuPreintegratorNode();

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void navStateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void dvlVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void inWaterCallback(const std_msgs::Bool::ConstPtr& msg);
  void addImuMeasurementsToPIM(const ros::Time& final_time);
  void generateInitialFactors();
  void generateSequentialFactors(const ros::Time& timestamp);
  gtsam::Values solveFactorGraph(const int start_idx, const int end_idx);
  geometry_msgs::PoseWithCovarianceStamped buildPoseResponse(const gtsam::Pose3& pose, const gtsam::Matrix& covariance, const int factor_index);
  void pruneGraph(const int end_idx, const gtsam::Values& results);
  geometry_msgs::PoseWithCovarianceStamped getResults(const gtsam::Values result, const int start_idx, const int end_idx);
  bool handlePreintegration(spurdog_acomms::PreintegrateImu::Request &req,
                          spurdog_acomms::PreintegrateImu::Response &res);
};

#endif  // MANIFOLD_PREINTEGRATOR_HPP
