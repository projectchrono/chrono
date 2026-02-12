SET(TESTS
  utest_ROS_simple_publisher_subscriber
)

SET(LIBS Chrono_core Chrono_ros)

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

MESSAGE(STATUS "Add unit test programs for ROS module")
build_utests(YES "${TESTS}" "${LIBS}")
