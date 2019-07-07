#include "riptide_gazebo/inertial_sensors.hpp"

namespace riptide_gazebo
{
InertialSensors::InertialSensors(ros::NodeHandle &nh)
{
   nh_ = nh;

   std::string sixdofTopic, imuTopic, dvlTopic, depthTopic;
   nh_.param("model", modelName_, std::string("maelstrom"));
   nh_.param("model_states_topic", modelStatesTopic_, std::string("/gazebo/model_states"));
   nh_.param("six_dof_topic", sixdofTopic, std::string("/auv_gnc/trans_ekf/six_dof"));
   nh_.param("imu_topic", imuTopic, std::string("/state/imu"));
   nh_.param("dvl_topic", dvlTopic, std::string("/state/dvl"));
   nh_.param("depth_topic", depthTopic, std::string("/state/depth"));

   modelStatesSub_ = nh_.subscribe<gazebo_msgs::ModelStates>(modelStatesTopic_, 1, &InertialSensors::modelStatesCB, this);
   sixdofPub_ = nh_.advertise<auv_msgs::SixDoF>(sixdofTopic, 1);
   imuPub_ = nh_.advertise<riptide_msgs::Imu>(imuTopic, 1);
   dvlPub_ = nh_.advertise<nortek_dvl::Dvl>(dvlTopic, 1);
   depthPub_ = nh_.advertise<riptide_msgs::Depth>(depthTopic, 1);

   linearVel_.setZero();
   lastLinearVel_.setZero();
   linearAccel_.setZero();
   angularVel_.setZero();
   rpy_.setZero();

   quatNED_.w() = 1;
   quatNED_.x() = 0;
   quatNED_.y() = 0;
   quatNED_.z() = 0;

   quatENU_.w() = 1;
   quatENU_.x() = 0;
   quatENU_.y() = 0;
   quatENU_.z() = 0;

   modelFound_ = false;
   modelIndex_ = 0;
   firstIter_ = true;

   ROS_INFO("Inertial Sensors initialized");
}

/**
 * This callback reads in IMU data from gazebo configured with the ENU frame, and then expresses all
 * of the values using an NED-measuring IMU (just like the LORD MicroStrain IMU)
 * NOTE: This code will ONLY work if the IMU used in gazebo is set to the 'world' reference frame. 
 * DO NOT use 'world_ned'
 */
void InertialSensors::modelStatesCB(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
   // Find index for modelName (look up every time in case Gazebo is shutdown then)
   modelFound_ = false;
   for (int i = 0; i < msg->name.size(); i++)
   {
      if (msg->name[i] == modelName_)
      {
         modelFound_ = true;
         modelIndex_ = i;
      }
   }

   if (!modelFound_)
   {
      ROS_WARN("InertialSensors: Model Name %s not found in %s", modelName_.c_str(), modelStatesTopic_.c_str());
      return;
   }
   
   ros::Time clockTime = ros::Time::now();

   // Imu and DVL Math
   tf::quaternionMsgToEigen(msg->pose[modelIndex_].orientation, quatENU_);
   quatNED_ = InertialSensors::getNEDQuaterionFromENU(quatENU_);
   rpy_ = auv_core::math_lib::toEulerAngle(quatNED_);

   tf::vectorMsgToEigen(msg->twist[modelIndex_].linear, linearVel_);
   tf::vectorMsgToEigen(msg->twist[modelIndex_].angular, angularVel_);
   if (firstIter_)
   {
      firstIter_ = false;
   }
   else
   {
      dt = clockTime.toSec() - lastClockTime_.toSec();
      linearAccel_ = (linearVel_ = lastLinearVel_) / dt;
   }

   linearAccel_ = quatENU_.conjugate() * linearAccel_;
   linearVel_ = quatENU_.conjugate() * linearVel_;
   angularVel_ = quatENU_.conjugate() * angularVel_;

   // SixDof Message
   sixdofMsg_.header.stamp = clockTime;
   sixdofMsg_.pose.position.x = msg->pose[modelIndex_].position.y;
   sixdofMsg_.pose.position.y = msg->pose[modelIndex_].position.x;
   sixdofMsg_.pose.position.z = -msg->pose[modelIndex_].position.z;
   tf::quaternionEigenToMsg(quatNED_, sixdofMsg_.pose.orientation);
   tf::vectorEigenToMsg(linearVel_, sixdofMsg_.velocity.linear);
   tf::vectorEigenToMsg(angularVel_, sixdofMsg_.velocity.angular);
   tf::vectorEigenToMsg(linearAccel_, sixdofMsg_.linear_accel);
   
   // IMU Message
   imuMsg_.header.stamp = clockTime;
   tf::vectorEigenToMsg(rpy_, imuMsg_.rpy_rad);
   tf::vectorEigenToMsg(angularVel_, imuMsg_.ang_vel_rad);

   rpy_ = rpy_* 180 / M_PI;
   angularVel_ = angularVel_* 180 / M_PI;

   tf::vectorEigenToMsg(rpy_, imuMsg_.rpy_deg);
   tf::vectorEigenToMsg(angularVel_, imuMsg_.ang_vel_deg);
   tf::vectorEigenToMsg(linearAccel_, imuMsg_.linear_accel);
   imuMsg_.heading_alt = rpy_(2);
   imuMsg_.heading_LORD = rpy_(2);

   // DVL Message
   dvlMsg_.header.stamp = clockTime;
   tf::vectorEigenToMsg(linearVel_, dvlMsg_.velocity);

   // Depth Message
   depthMsg_.header.stamp = clockTime;
   depthMsg_.depth = -msg->pose[modelIndex_].position.z;

   sixdofPub_.publish(sixdofMsg_);
   imuPub_.publish(imuMsg_);
   dvlPub_.publish(dvlMsg_);
   depthPub_.publish(depthMsg_);

   lastLinearVel_ = linearVel_;
   lastClockTime_ = clockTime;
}

/**
 * @param quatENU quaternion from Local ENU frame to the body-fixed ENU frame
 * \brief Returns the quaternion from local NED frame to the body-fixed NED frame
 */
Eigen::Quaterniond InertialSensors::getNEDQuaterionFromENU(Eigen::Quaterniond quatENU)
{
   // Quaternion rotated 90 deg in yaw
   Eigen::Quaterniond quat1 = auv_core::math_lib::toQuaternion(M_PI/2.0, 0, 0);

   // Quaternion difference from quat1 to quatENU
   Eigen::Quaterniond qDiff = quat1.conjugate() * quatENU;
   Eigen::Vector4d angleAxis1 = auv_core::math_lib::quaternion2AngleAxis(qDiff); // Angle Axis format

   Eigen::Vector4d angleAxis2 = Eigen::Vector4d::Zero();
   angleAxis2(0) = angleAxis1(0); // Angle
   angleAxis2(1) = angleAxis1(1); // X component remains the same
   angleAxis2(2) = -angleAxis1(2); // Negate y compoennt
   angleAxis2(3) = -angleAxis1(3); // Negate z component

   return auv_core::math_lib::angleAxis2Quaternion(angleAxis2); // Convert to quaternion
}
}