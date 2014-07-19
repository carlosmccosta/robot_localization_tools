cmake_minimum_required(VERSION 2.8.3)
project(robot_localization_tools)

##############
## Packages ##
##############

set(${PROJECT_NAME}_CATKIN_COMPONENTS 
    roscpp
    tf2
    angles
    std_msgs
    geometry_msgs
    gazebo_msgs
    laserscan_to_pointcloud
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})



###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_COMPONENTS}    
)



###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(robot_localization_error src/robot_localization_error.cpp)

add_executable(robot_localization_error_node src/robot_localization_error_node.cpp)

target_link_libraries(robot_localization_error ${catkin_LIBRARIES})
target_link_libraries(robot_localization_error_node robot_localization_error ${catkin_LIBRARIES})

