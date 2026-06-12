SET(TESTS
  utest_ROS_bridge_roundtrip
)

# The test binary acts as the "real ROS" peer, so unlike Chrono_ros itself it
# links rclcpp and the message packages it instantiates as typed messages.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

SET(LIBS
  Chrono_core
  Chrono_ros
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

# Force C++17, required by rclcpp
set(CMAKE_CXX_STANDARD 17)

MESSAGE(STATUS "Add unit test programs for ROS module")
build_utests(YES "${TESTS}" "ros" "${LIBS}")
