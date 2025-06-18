#ifndef IMU_PREINTEGRATOR_HPP
#define IMU_PREINTEGRATOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

class ImuPreintegratorNode {
private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::ServiceServer preint_srv_;

  std::deque<sensor_msgs::Imu> imu_buffer_;
  std::mutex buffer_mutex_;
  ros::Time ti_;
  // Add a sequence number for the IMU messages
  int req_seq_num_;
  // Preintegration parameters
  boost::shared_ptr<gtsam::PreintegrationParams> preint_params_;
  gtsam::imuBias::ConstantBias bias;

  // Preintegration NavState
  gtsam::NavState prevState;
  gtsam::NavState predState;

public:
  ImuPreintegratorNode();

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  bool handlePreintegrate(spurdog_acomms::PreintegrateImu::Request &req,
                          spurdog_acomms::PreintegrateImu::Response &res);
};

#endif  // IMU_PREINTEGRATOR_HPP
