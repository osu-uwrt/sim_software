#include "riptide_gazebo/lord_imu_sim.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lord_imu_sim");
  ros::NodeHandle nh("~");
  riptide_gazebo::LordImu imu(nh);
  ros::spin();
  return 0;
}