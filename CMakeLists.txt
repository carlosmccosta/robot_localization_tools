cmake_minimum_required(VERSION 2.8.3)
project(robot_localization_tools)



#######################################################################################################################
## packages
#######################################################################################################################

set(${PROJECT_NAME}_CATKIN_COMPONENTS 
    roscpp
    tf2
    angles
    std_msgs
    geometry_msgs
    rosconsole
    laserscan_to_pointcloud
    message_generation
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})



#######################################################################################################################
## catkin specific configuration
#######################################################################################################################


add_message_files(
  FILES
  LocalizationError.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        rlt_robot_localization_error
        rlt_twist_publisher
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
        message_runtime
)



#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(rlt_robot_localization_error
    src/robot_localization_error.cpp
)

add_library(rlt_twist_publisher
    src/twist_publisher.cpp
)



#==================
# executables
#==================

add_executable(rlt_robot_localization_error_node
    src/robot_localization_error_node.cpp
)

add_executable(rlt_twist_publisher_node
    src/twist_publisher_node.cpp
)



#==================
# dependencies
#==================

add_dependencies(rlt_robot_localization_error
    ${PROJECT_NAME}_generate_messages_cpp
)



#==================
# library link
#==================

target_link_libraries(rlt_robot_localization_error
    ${catkin_LIBRARIES}
)

target_link_libraries(rlt_twist_publisher
    ${catkin_LIBRARIES}
)

target_link_libraries(rlt_robot_localization_error_node
    rlt_robot_localization_error
    ${catkin_LIBRARIES}
)

target_link_libraries(rlt_twist_publisher_node
    rlt_twist_publisher
    ${catkin_LIBRARIES}
)
