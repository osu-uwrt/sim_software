#include "riptide_gazebo/lord_imu_sim.hpp"

namespace riptide_gazebo
{
LordImu::LordImu(ros::NodeHandle &nh)
{
   nh_ = nh;

   std::string sim_topic, real_topic;
   nh_.param("sim_topic", sim_topic, std::string("/sensors/imu"));
   nh_.param("real_topic", real_topic, std::string("/imu/filter"));

   uuvImuSub_ = nh_.subscribe<sensor_msgs::Imu>(sim_topic, 1, &LordImu::uuvImuCB, this);
   lordImuPub_ = nh_.advertise<imu_3dm_gx4::FilterOutput>(real_topic, 1);

   linearAccel_.setZero();
   linearAccelIn_.setZero();
   rpy_.setZero();

   quatNED_.w() = 1;
   quatNED_.x() = 0;
   quatNED_.y() = 0;
   quatNED_.z() = 0;

   quatENU_.w() = 1;
   quatENU_.x() = 0;
   quatENU_.y() = 0;
   quatENU_.z() = 0;

   gravityNED_.setZero();
   gravityNED_(2) = 9.81;

   ROS_INFO("LORD Microstrain IMU initialized");
}

/**
 * This callback reads in IMU data from gazebo configured with the ENU frame, and then expresses all
 * of the values using an NED-measuring IMU (just like the LORD MicroStrain IMU)
 * NOTE: This code will ONLY work if the IMU used in gazebo is set to the 'world' reference frame. 
 * DO NOT use 'world_ned'
 */
void LordImu::uuvImuCB(const sensor_msgs::Imu::ConstPtr &msg)
{
   // Get body-frame orientation from an NED-measuring IMU
   tf::quaternionMsgToEigen(msg->orientation, quatENU_);
   Eigen::Quaterniond quat1 = auv_core::math_lib::toQuaternion(M_PI/2.0, 0, -M_PI);
   quatENU_ = quatENU_ * quat1;
   quatNED_ = LordImu::getNEDQuaterionFromENU(quatENU_);
   tf::quaternionEigenToMsg(quatNED_, imuMsg_.quaternion);
   
   // Euler Angles (Roll, Pitch, Yaw)
   rpy_ = auv_core::math_lib::toEulerAngle(quatNED_);
   tf::vectorEigenToMsg(rpy_, imuMsg_.euler_rpy);
   imuMsg_.heading_update_alt = rpy_(2);
   imuMsg_.heading_update_LORD = rpy_(2);

   // Linear Acceleration
   tf::vectorMsgToEigen(msg->linear_acceleration, linearAccelIn_);
   linearAccel_ = linearAccelIn_ - quatNED_.conjugate() * gravityNED_;
   tf::vectorEigenToMsg(linearAccel_, imuMsg_.linear_acceleration);
   std::cout << "gravity in body frame:" << std::endl << quatNED_.conjugate() * gravityNED_ << std::endl;
   std::cout << "accel in:" << std::endl << linearAccelIn_ << std::endl;

   // Angular Velocity
   tf::vectorMsgToEigen(msg->angular_velocity, angularVelIn_);
   imuMsg_.angular_velocity = msg->angular_velocity;

   lordImuPub_.publish(imuMsg_);

   /*Eigen::Quaterniond quat_to_ned = auv_core::math_lib::toQuaternion(M_PI/2, 0, M_PI);
   tf::quaternionMsgToEigen(msg->orientation, quat_enu_);
   quat_ned_ = quat_to_ned.conjugate() * quat_enu_; // Compute quat wrt NED axis
   tf::quaternionEigenToMsg(quat_ned_, imu_msg_.quaternion);
   std::cout << "Quat NED:" << std::endl << quat_ned_.w() << std::endl << quat_ned_.vec() << std::endl;

   Eigen::Vector3d euler = auv_core::math_lib::toEulerAngle(quat_ned_);
   imu_msg_.euler_rpy.x = euler(0);
   imu_msg_.euler_rpy.y = euler(1);
   imu_msg_.euler_rpy.z = euler(2);
   imu_msg_.heading_update_alt = euler(2);
   imu_msg_.heading_update_LORD = euler(2);

   tf::vectorMsgToEigen(msg->linear_acceleration, linear_accel_in_);
   linear_accel_ = linear_accel_in_ + quat_ned_ * gravity_;

   imu_msg_.linear_acceleration.x = linear_accel_(0);
   imu_msg_.linear_acceleration.y = linear_accel_(1);
   imu_msg_.linear_acceleration.z = linear_accel_(2);

   imu_msg_.angular_velocity = msg->angular_velocity;

   lord_imu_pub_.publish(imu_msg_);
   std::cout << "euler:" << std::endl << euler << std::endl;
   std::cout << "linear accel:" << std::endl << linear_accel_ << std::endl;*/

}

/**
 * @param quatENU quaternion from Local ENU frame to the body-fixed ENU frame
 * \brief Returns the quaternion from local NED frame to the body-fixed NED frame
 */
Eigen::Quaterniond LordImu::getNEDQuaterionFromENU(Eigen::Quaterniond quatENU)
{
   // Quaternion rotated 90 deg in yaw
   Eigen::Quaterniond quat1 = auv_core::math_lib::toQuaternion(M_PI/2.0, 0, 0);

   // Quaternion difference from quat1 to quatENU
   Eigen::Quaterniond qDiff = quat1.conjugate() * quatENU;
   Eigen::Vector4d angleAxis1 = auv_core::math_lib::quaternion2AngleAxis(qDiff); // Angle Axis format

   Eigen::Vector4d angleAxis2 = Eigen::Vector4d::Zero();
   angleAxis2(0) = angleAxis1(0); // Angle
   angleAxis2(1) = -angleAxis1(2); // X component remains the same
   angleAxis2(2) = angleAxis1(1); // Negate y compoennt
   angleAxis2(3) = -angleAxis1(3); // Negate z component

   return auv_core::math_lib::angleAxis2Quaternion(angleAxis2); // Convert to quaternion
}

Eigen::Vector3d getGravityInBodyFrame()
{
   
}

}