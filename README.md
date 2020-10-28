# auri_v2000
In progress:
au_core

TODO:
au_control
au_geometry
au_localization
au_mapping
au_missioncontrol
au_motionplanner
au_planner
au_sensors
au_sonar
au_vision

# Installation
Second link is for debian packages but come back to first link for complete installation
https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Binary/
https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/

# Building packages
https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/
Colcon is used instead of catkin for ROS2. Run colcon build in the root directory to build it.

# Migrating ROS1 source code
https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
this article gives a general look into how CMakeLists and the build system are different and how to change them accordingly. It also gives a good intro into source code conversion with pubsub equivalents.

All packages used in au_core are currently supported in ROS2.
