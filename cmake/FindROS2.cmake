# Note that this does not find any ros2 package in particular, but determines whether a distro is sourced properly
# Can be extended to handle supported / unsupported distros
unset(ROS2_FOUND)
message(STATUS "ROS 2 distro is \"$ENV{ROS_DISTRO}\"")
if (NOT DEFINED ENV{ROS_DISTRO} OR NOT DEFINED ENV{AMENT_PREFIX_PATH})
    message(WARNING, "To build a module that depends on ROS 2, a ROS distribution needs to be sourced, but none detected.")
    set(ROS2_FOUND FALSE)
    return()
endif()
set(ROS2_FOUND TRUE)