#include "riptide_gazebo/inertial_sensors.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lord_imu_sim");
  ros::NodeHandle nh("~");
  riptide_gazebo::InertialSensors inertialSensors(nh);
  ros::spin();
  return 0;
}