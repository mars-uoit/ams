cmake_minimum_required(VERSION 2.8.3)
project(ams_uav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs tf genmsg dynamic_reconfigure
             actionlib actionlib_msgs mavros roslaunch roslint)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ModeChange.srv
   DesiredPoseChange.srv
 )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES FlyToTarget.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
 )

 generate_dynamic_reconfigure_options(
  cfg/control.cfg
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs tf dynamic_reconfigure mavros sensor_msgs geometry_msgs actionlib_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## Quad controller node
add_library(quad_controller src/quad_controller.cpp)
add_dependencies(quad_controller ${PROJECT_NAME}_gencpp)
target_link_libraries(quad_controller quad_controller ${catkin_LIBRARIES})
add_executable(quad_controller_node src/quad_controller_node.cpp)
add_dependencies(quad_controller_node ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(quad_controller_node quad_controller ${catkin_LIBRARIES})

## Quad teleop node
add_library(quad_teleop src/quad_teleop.cpp)
add_dependencies(quad_teleop ${PROJECT_NAME}_gencpp)
target_link_libraries(quad_teleop quad_teleop ${catkin_LIBRARIES})
add_executable(quad_teleop_node src/quad_teleop_node.cpp)
add_dependencies(quad_teleop_node ${PROJECT_NAME}_gencpp)
target_link_libraries(quad_teleop_node quad_teleop ${catkin_LIBRARIES})

## Quad manager node
add_library(quad_manager src/quad_manager.cpp)
add_dependencies(quad_manager ${PROJECT_NAME}_gencpp)
target_link_libraries(quad_manager quad_manager ${catkin_LIBRARIES})
add_executable(quad_manager_node src/quad_manager_node.cpp)
add_dependencies(quad_manager_node ${PROJECT_NAME}_gencpp)
target_link_libraries(quad_manager_node quad_manager ${catkin_LIBRARIES})

## Quad trajectory planner
add_executable(trajectory_planner src/trajectory_planner.cpp)
add_dependencies(trajectory_planner ${PROJECT_NAME}_gencpp)
target_link_libraries(trajectory_planner ${catkin_LIBRARIES})


add_executable(fly_to_target_action_server src/fly_to_target_action_server.cpp)
target_link_libraries(fly_to_target_action_server ${catkin_LIBRARIES})
add_dependencies(fly_to_target_action_server ${PROJECT_NAME}_gencpp)

add_executable(action_test src/action_test.cpp)
target_link_libraries(action_test ${catkin_LIBRARIES})
add_dependencies(action_test ${PROJECT_NAME}_gencpp)
