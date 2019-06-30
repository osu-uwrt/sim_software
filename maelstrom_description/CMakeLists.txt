cmake_minimum_required(VERSION 2.8.3)
project(maelstrom_description)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
   common_robot_description
   xacro
   uuv_gazebo_ros_plugins
   uuv_world_ros_plugins
)

catkin_package(
   CATKIN_DEPENDS xacro
)

# Xacro ctest_read_custom_files()
set(model_xacro_files
   model/robots/maelstrom.urdf.xacro
)

# Generate urdf's and put them in model/urdf
foreach(xacro_file ${model_xacro_files})
   xacro_add_xacro_file(${xacro_file} INORDER)
   list(APPEND xacro_outputs ${XACRO_OUTPUT_FILE})
endforeach()
xacro_install(xacro_target ${xacro_outputs} DESTINATION model/urdf)