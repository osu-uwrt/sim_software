#include "riptide_gazebo/inertial_sensors.hpp"

namespace riptide_gazebo
{
InertialSensors::InertialSensors(ros::NodeHandle &nh)
{
   nh_ = nh;

   std::string uuvImuTopic, sixdofTopic, imuTopic, dvlTopic, depthTopic;
   nh_.param("model", modelName_, std::string("maelstrom"));
   nh_.param("model_states_topic", modelStatesTopic_, std::string("/gazebo/model_states"));
   nh_.param("uuv_imu_topic", uuvImuTopic, std::string("/sensors/imu"));
   nh_.param("six_dof_topic", sixdofTopic, std::string("/auv_gnc/trans_ekf/six_dof"));
   nh_.param("imu_topic", imuTopic, std::string("/state/imu"));
   nh_.param("dvl_topic", dvlTopic, std::string("/state/dvl"));
   nh_.param("depth_topic", depthTopic, std::string("/state/depth"));

   modelStatesSub_ = nh_.subscribe<gazebo_msgs::ModelStates>(modelStatesTopic_, 1, &InertialSensors::modelStatesCB, this);
   uuvImuSub_ = nh_.subscribe<sensor_msgs::Imu>(uuvImuTopic, 1, &InertialSensors::uuvImuCB, this);

   sixdofPub_ = nh_.advertise<auv_msgs::SixDoF>(sixdofTopic, 1);
   imuPub_ = nh_.advertise<riptide_msgs::Imu>(imuTopic, 1);
   dvlPub_ = nh_.advertise<nortek_dvl::Dvl>(dvlTopic, 1);
   depthPub_ = nh_.advertise<riptide_msgs::Depth>(depthTopic, 1);

   linearVel_.setZero();
   linearAccel_.setZero();
   linearAccelIn_.setZero();
   angularVel_.setZero();
   rpy_.setZero();

   quatNED_.setIdentity();
   quatENU_.setIdentity();

   quatBodyFixedENU2NED_ = auv_core::rot3d::rpy2Quat(M_PI, 0, 0); // Roll by 180 deg

   gravityNED_.setZero();
   gravityNED_(2) = 9.81;

   modelFound_ = false;
   modelIndex_ = 0;

   ROS_INFO("Inertial Sensors initialized");
}

/**
 * \brief This callback reads in the model state from gazebo and publishes the appropriate sensor data on their topics
 */
void InertialSensors::modelStatesCB(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
   // Find index for modelName
   if (!modelFound_)
   {
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
         ROS_WARN("InertialSensors: Model name %s not found in topic %s. Check if model is loaded.", modelName_.c_str(), modelStatesTopic_.c_str());
         return;
      }
   }

   ros::Time clockTime = ros::Time::now();

   // Get NED quaternion and Euler Angles
   tf::quaternionMsgToEigen(msg->pose[modelIndex_].orientation, quatENU_);
   quatNED_ = InertialSensors::getNEDQuaterionFromENU(quatENU_);
   rpy_ = auv_core::rot3d::quat2RPY(quatNED_);

   // Express velocity vectors in body-fixed NED frame
   tf::vectorMsgToEigen(msg->twist[modelIndex_].linear, linearVel_);
   tf::vectorMsgToEigen(msg->twist[modelIndex_].angular, angularVel_);
   linearVel_ = quatENU_.conjugate() * linearVel_;    // Express in body-fixed ENU frame
   angularVel_ = quatENU_.conjugate() * angularVel_;  // Express in body-fixed ENU frame
   linearVel_ = quatBodyFixedENU2NED_ * linearVel_;   // Express in body-fixed NED frame
   angularVel_ = quatBodyFixedENU2NED_ * angularVel_; // Express in body-fixed NED frame

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
   tf::quaternionEigenToMsg(quatNED_, imuMsg_.quaternion);
   tf::vectorEigenToMsg(rpy_, imuMsg_.rpy_rad);
   tf::vectorEigenToMsg(rpy_ * 180 / M_PI, imuMsg_.rpy_deg);
   tf::vectorEigenToMsg(angularVel_, imuMsg_.ang_vel_rad);
   tf::vectorEigenToMsg(angularVel_ * 180 / M_PI, imuMsg_.ang_vel_deg);
   tf::vectorEigenToMsg(linearAccel_, imuMsg_.linear_accel);
   imuMsg_.heading_alt = rpy_(2) * 180 / M_PI;
   imuMsg_.heading_LORD = rpy_(2) * 180 / M_PI;

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
}

/**
 * \brief Express the vehicle's linear accel in the body-fixed NED frame and remove the gravitational components.
 * NOTE: This will only work if the IMU is set to use the 'world' reference frame. Do NOT use 'world_ned'
 */
void InertialSensors::uuvImuCB(const sensor_msgs::Imu::ConstPtr &msg)
{
   if (!modelFound_)
      return;

   // Express linear accel in body-fixed NED frame
   linearAccelIn_(0) = -msg->linear_acceleration.x; // For some reason with UUV Sim, have to negate the x-component
   linearAccelIn_(1) = msg->linear_acceleration.y;
   linearAccelIn_(2) = msg->linear_acceleration.z;

   linearAccel_ = linearAccelIn_ - quatNED_.conjugate() * gravityNED_;
   tf::vectorEigenToMsg(linearAccel_, imuMsg_.linear_accel);
}

/**
 * @param quatENU Quaternion from Local ENU frame to the body-fixed ENU frame
 * \brief Returns the quaternion from local NED frame to the body-fixed NED frame
 */
Eigen::Quaterniond InertialSensors::getNEDQuaterionFromENU(Eigen::Quaterniond quatENU)
{
   // Quaternion rotated 90 deg in yaw
   Eigen::Quaterniond quat1 = auv_core::rot3d::rpy2Quat(0, 0, M_PI / 2.0);

   // Quaternion difference from quat1 to quatENU
   Eigen::Quaterniond qDiff = quat1.conjugate() * quatENU; // qDiff = q1.conjugate() * q2
   Eigen::Vector4d angleAxis1 = auv_core::rot3d::quat2AngleAxis(qDiff); // Angle Axis format

   Eigen::Vector4d angleAxis2 = Eigen::Vector4d::Zero();
   angleAxis2(0) = angleAxis1(0);  // Angle remains unchanged
   angleAxis2(1) = angleAxis1(1);  // X component remains unchanged
   angleAxis2(2) = -angleAxis1(2); // Negate y compoennt
   angleAxis2(3) = -angleAxis1(3); // Negate z component

   return auv_core::rot3d::angleAxis2Quat(angleAxis2); // Convert to quaternion
}
} // namespace riptide_gazebo
