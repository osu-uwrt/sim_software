#ifndef INERTIAL_SENSORS
#define INERTIAL_SENSORS

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_core/math_lib.hpp"
#include "auv_core/constants.hpp"
#include <tf/tf.h>
#include "eigen_conversions/eigen_msg.h"
#include "math.h"

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "nortek_dvl/Dvl.h"
#include "auv_msgs/SixDoF.h"

namespace riptide_gazebo
{
class InertialSensors
{
 private:
   ros::NodeHandle nh_;
   ros::Subscriber modelStatesSub_, clockSub_;
   ros::Publisher sixdofPub_, imuPub_, dvlPub_, depthPub_;
   std::string modelName_, modelStatesTopic_;
   bool modelFound_;
   int modelIndex_;
   double dt;
   ros::Time lastClockTime_;

   riptide_msgs::Imu imuMsg_;
   riptide_msgs::Depth depthMsg_;
   nortek_dvl::Dvl dvlMsg_;
   auv_msgs::SixDoF sixdofMsg_;
   
   Eigen::Vector3d linearVel_, lastLinearVel_, linearAccel_, angularVel_, rpy_;
   Eigen::Quaterniond quatNED_, quatENU_;
   bool firstIter_;

 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   InertialSensors(ros::NodeHandle &nh);
   void modelStatesCB(const gazebo_msgs::ModelStates::ConstPtr &msg);
   Eigen::Quaterniond getNEDQuaterionFromENU(Eigen::Quaterniond quatENU);
};
}

#endif