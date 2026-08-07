set(TESTS "")

if(CH_USE_URDF)
  SET(TESTS ${TESTS}
    utest_PARSERS_URDF_simple
    utest_PARSERS_URDF_package
  )
endif()

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_parsers)
if(CH_USE_ROS)
  list(APPEND LIBS Chrono_ros)
endif()

message(STATUS "Add unit test programs for PARSERS module")
build_utests(YES "${TESTS}" "parsers" "${LIBS}")

# utest_PARSERS_URDF_package uses ament_index_cpp directly (guarded by
# CHRONO_HAS_ROS). Chrono_parsers links ament PRIVATE, so the test target does
# not inherit its include dirs / link - provide them to the test target itself.
if(CH_USE_ROS)
  find_package(ament_index_cpp QUIET)
  set(_ament_prefix_path "$ENV{AMENT_PREFIX_PATH}")
  foreach(PROGRAM ${TESTS})
    foreach(_ros2_packages_path IN LISTS _ament_prefix_path)
      string(REPLACE ":" ";" _ros2_packages_path ${_ros2_packages_path})
      target_include_directories(${PROGRAM} PRIVATE "${_ros2_packages_path}/include")
    endforeach()
    # plain signature to match build_utests' target_link_libraries (no mixing)
    target_link_libraries(${PROGRAM} ament_index_cpp::ament_index_cpp)
  endforeach()
endif()
