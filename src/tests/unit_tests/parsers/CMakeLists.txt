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
