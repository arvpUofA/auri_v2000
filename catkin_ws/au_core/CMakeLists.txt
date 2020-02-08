cmake_minimum_required(VERSION 3.10.2)

project(au_core)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Werror)

find_package(ament_cmake REQUIRED)
#find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rcllib REQUIRED
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(message_generation REQUIRED)
find_package(tf2 REQUIRED)
find_package(angles REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
find_package(yaml-cpp)
include_directories(${Eigen_INCLUDE_DIRS})


#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

#find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(
  FILES
  msg/LedCommand.msg
  msg/Map.msg
  msg/MapObject.msg
  msg/MCRaw.msg
  msg/MCDiff.msg
  msg/MCBaseSpeed.msg
  msg/MCStatus.msg
  msg/MCFeedback.msg
  msg/KillSig.msg
  msg/Depth.msg
  msg/Roi.msg
  msg/RoiArray.msg
  msg/ServoAngle.msg
  msg/DynamicsState.msg
  msg/DynamicsStability.msg
  msg/MotorDisable.msg
  msg/Dvl.msg
  msg/DvlStatus.msg
  msg/PingStatus.msg
  srv/ActuatorCommand.srv
  srv/Detection.srv
  srv/Tracking.srv
)


#rosidl_generate_interfaces(
#  DEPENDENCIES
  #std_msgs
  #geometry_msgs
#)



ament_export_dependencies(message_runtime roslib)
ament_export_include_directories(include)
ament_export_libraries(utils)
#catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES utils
#    CATKIN_DEPENDS message_runtime roslib
#)

###########
## Build ##
###########

#include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#)

#Build for utils library
add_library(utils
    src/camera_info.cpp
    src/loader_util.cpp
    src/rolling_stats.cpp
    src/vision_util.cpp
    src/sigint_handler.cpp
    src/timeout_guard.cpp
    src/math_util.cpp)

#add_dependencies(utils
#    ${catkin_EXPORTED_TARGETS}
#)
target_link_libraries(utils
    yaml-cpp
)

#############
##  Tests  ##
#############

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(rolling_stats_tests
            test/rolling_stats_tests.cpp)
    ament_target_dependencies(rolling_stats_tests
            "rclcpp"
            "std_msgs")
    target_link_libraries(rolling_stats_tests utils)

    ament_add_gtest(math_utils_tests
            test/math_utils_tests.cpp)
    target_link_libraries(math_utils_tests utils)

    ament_add_gtest(loader_util_tests
            test/loader_util_tests.cpp)
    target_link_libraries(loader_util_tests utils)

    #ament_add_gtest(test/loader_util_tests.py)
endif()

#############
## Install ##
#############

install(TARGETS utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h|*.hpp"
  PATTERN ".svn" EXCLUDE
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()
