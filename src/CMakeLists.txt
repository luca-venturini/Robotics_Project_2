cmake_minimum_required(VERSION 3.0.2)
project(demo)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
  tf2_ros
)

catkin_package(

)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(
  odom_tf src/odom_tf.cpp
)

add_executable(
  show_path src/show_path.cpp
)

add_dependencies(odom_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf
  ${catkin_LIBRARIES}
)

add_dependencies(show_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_path
  ${catkin_LIBRARIES}
)
