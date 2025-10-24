set(TESTS "")

if(HAVE_URDF)
  SET(TESTS ${TESTS}
    utest_PARSERS_URDF_simple
    utest_PARSERS_URDF_package
  )
endif()

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_parsers)
if(HAVE_ROS)
  list(APPEND LIBS Chrono_ros)
endif()

message(STATUS "Add unit test programs for PARSERS module")
build_utests(YES "${TESTS}" "${LIBS}")
foreach(PROGRAM ${TESTS})
    if(HAVE_ROS)
      target_compile_definitions(${PROGRAM} PRIVATE "HAVE_ROS")
    endif()
endforeach()
