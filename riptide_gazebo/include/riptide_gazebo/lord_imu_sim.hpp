#ifndef LORD_IMU_SIM
#define LORD_IMU_SIM

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_core/math_lib.hpp"
#include "auv_core/constants.hpp"
#include <tf/tf.h>
#include "eigen_conversions/eigen_msg.h"
#include "math.h"

#include "ros/ros.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "sensor_msgs/Imu.h"

namespace riptide_gazebo
{
class LordImu
{
 private:
   ros::NodeHandle nh_;
   ros::Subscriber uuvImuSub_;
   ros::Publisher lordImuPub_;
   imu_3dm_gx4::FilterOutput imuMsg_;
   Eigen::Quaterniond quatNED_, quatENU_;
   Eigen::Vector3d linearAccel_, linearAccelIn_, angularVel_, angularVelIn_, rpy_, gravityNED_;

 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   LordImu(ros::NodeHandle &nh);
   void uuvImuCB(const sensor_msgs::Imu::ConstPtr &msg);
   Eigen::Vector3d getGravityInBodyFrame(Eigen::Quaterniond quatENU);
   Eigen::Quaterniond getNEDQuaterionFromENU(Eigen::Quaterniond quatENU);
};
}

#endif