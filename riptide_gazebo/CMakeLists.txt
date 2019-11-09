cmake_minimum_required(VERSION 2.8.3)
project(riptide_gazebo)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   nortek_dvl
   sensor_msgs
   riptide_msgs
   imu_3dm_gx4
   roscpp
   tf
)


catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS
      uuv_sensor_ros_plugins_msgs
      nortek_dvl
      geometry_msgs
      sensor_msgs
      std_msgs
      riptide_msgs
      rospy
)

include_directories(${catkin_INCLUDE_DIRS})
