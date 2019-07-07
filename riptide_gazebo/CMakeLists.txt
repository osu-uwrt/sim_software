cmake_minimum_required(VERSION 2.8.3)
project(riptide_gazebo)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   auv_core
   auv_msgs
   eigen_conversions
   nortek_dvl
   sensor_msgs
   riptide_msgs
   imu_3dm_gx4
   roscpp
   tf
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
   CATKIN_DEPENDS
      uuv_sensor_ros_plugins_msgs
      nortek_dvl
      geometry_msgs
      sensor_msgs
      std_msgs
      riptide_msgs
      rospy
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(inertial_sensors src/inertial_sensors_node.cpp src/inertial_sensors.cpp)
target_link_libraries(inertial_sensors ${catkin_LIBRARIES})
add_dependencies(inertial_sensors ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
   scripts/depth_sensor_node
   scripts/nortek_dvl_node
   # scripts/lord_imu_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)