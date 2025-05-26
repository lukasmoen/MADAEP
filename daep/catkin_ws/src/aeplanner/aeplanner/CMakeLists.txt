cmake_minimum_required(VERSION 2.8.3)
project(aeplanner)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  tf
  tf2
  genmsg 
  actionlib_msgs 
  actionlib
  nav_msgs
  kdtree
  pigain
  dynamic_reconfigure
  plan_env  
  sensor_msgs
  pcl_conversions
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)


add_message_files(
  DIRECTORY msg 
  FILES Node.msg Goal.msg Goals.msg
)

add_action_files(
  DIRECTORY action 
  FILES aeplanner.action
)

add_service_files(
  DIRECTORY srv
  FILES Reevaluate.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
  actionlib_msgs
  sensor_msgs

)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES aeplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs tf kdtree sensor_msgs plan_env
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS} 
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

  ${plan_env_INCLUDE_DIRS} 
)

add_library(aeplanner
  src/aeplanner_nodelet.cpp
  src/aeplanner.cpp
  src/aeplanner_viz.cpp
  src/param.cpp
  src/kalman.cpp
)

target_link_libraries(aeplanner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(aeplanner
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

# Create an executable for aeplanner_node
add_executable(aeplanner_node
  src/aeplanner_node.cpp
  src/aeplanner.cpp
  src/aeplanner_viz.cpp
  src/param.cpp
  src/kalman.cpp
)

target_link_libraries(aeplanner_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_dependencies(aeplanner_node
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  aeplanner_ref_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
