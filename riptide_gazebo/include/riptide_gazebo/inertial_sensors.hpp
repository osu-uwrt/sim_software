#ifndef INERTIAL_SENSORS
#define INERTIAL_SENSORS

#include <tf/tf.h>
#include "auv_core/math_lib.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen_conversions/eigen_msg.h"
#include "math.h"

#include "auv_msgs/SixDoF.h"
#include "gazebo_msgs/ModelStates.h"
#include "nortek_dvl/Dvl.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/Imu.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

namespace riptide_gazebo
{
class InertialSensors
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber modelStatesSub_, uuvImuSub_;
  ros::Publisher sixdofPub_, imuPub_, dvlPub_, depthPub_;
  std::string modelName_, modelStatesTopic_;
  bool modelFound_;
  int modelIndex_;

  riptide_msgs::Imu imuMsg_;
  riptide_msgs::Depth depthMsg_;
  nortek_dvl::Dvl dvlMsg_;
  auv_msgs::SixDoF sixdofMsg_;

  Eigen::Vector3d gravityNED_, linearVel_, linearAccel_, linearAccelIn_, angularVel_, rpy_;
  Eigen::Quaterniond quatNED_, quatENU_, quatBodyFixedENU2NED_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  InertialSensors(ros::NodeHandle &nh);
  void modelStatesCB(const gazebo_msgs::ModelStates::ConstPtr &msg);
  void uuvImuCB(const sensor_msgs::Imu::ConstPtr &msg);
  Eigen::Quaterniond getNEDQuaterionFromENU(Eigen::Quaterniond quatENU);
};
}

#endif