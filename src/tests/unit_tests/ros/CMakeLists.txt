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

# Core unit tests: the dependency-free CDR / schema / message / transport suites
# from chrono_ros/tests_dev, registered with CTest here. They link the internal
# chrono_ros_core static library and use a minimal return-code harness (not
# googletest), so each is registered directly with add_test. The same sources
# also build under Address/UB sanitizers via chrono_ros/tests_dev/run_dev_tests.sh
# for the fast dev loop.
if(TARGET chrono_ros_core)
  set(CH_ROS_TESTS_DEV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../chrono_ros/tests_dev)
  foreach(t cdr message schema transport)
    add_executable(utest_ROS_${t} ${CH_ROS_TESTS_DEV_DIR}/test_${t}.cpp)
    target_link_libraries(utest_ROS_${t} chrono_ros_core)
    target_include_directories(utest_ROS_${t} PRIVATE ${CH_ROS_TESTS_DEV_DIR})
    set_property(TARGET utest_ROS_${t} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:utest_ROS_${t}>")
    add_test(NAME utest_ROS_${t} COMMAND utest_ROS_${t})
  endforeach()
endif()
