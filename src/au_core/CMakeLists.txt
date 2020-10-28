cmake_minimum_required(VERSION 3.10.2)

project(au_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#set(CMAKE_CXX_EXTENSIONS OFF)
#add_compile_options(-Wall -Wextra -Werror)\
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(message_generation REQUIRED)
find_package(tf2 REQUIRED)
find_package(angles REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include_directories(${Eigen_INCLUDE_DIRS})
find_package(Eigen REQUIRED)
find_package(yaml-cpp)



#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################




#rosidl_generate_interfaces(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)



#ament_export_dependencies(message_runtime roslib)
#ament_export_include_directories(include)
#ament_export_libraries(utils)
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
#)

#Build for utils library
add_library(utils
    src/camera_info.cpp
    src/loader_util.cpp
    src/rolling_stats.cpp
    src/vision_util.cpp
    src/timeout_guard.cpp
    src/math_util.cpp)
ament_target_dependencies(utils
    "rclcpp"
    "rclcpp_components")

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
