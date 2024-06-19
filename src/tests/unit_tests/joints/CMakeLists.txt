# Unit tests for miscellaneous Chrono joints
# ==================================================================

message(STATUS "Unit tests for Chrono joints...")

set(TEST_PROGRAMS
    utest_JOINT_revolute
    utest_JOINT_spherical
    utest_JOINT_universal
    utest_JOINT_prismatic
    utest_JOINT_cylindrical
    utest_JOINT_distance
    utest_JOINT_distance_unilateral
    utest_JOINT_rackpinion
    utest_JOINT_revsph
    utest_JOINT_linactuator
    utest_JOINT_transpring
    utest_JOINT_tsda
    utest_JOINT_rotspring
)

#--------------------------------------------------------------

# A hack to set the working directory in which to execute the CTest
# runs.  This is needed for tests that need to access the Chrono data
# directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
set(LIBRARIES ChronoEngine)

#--------------------------------------------------------------
# Add executables

foreach(PROGRAM ${TEST_PROGRAMS})
  message(STATUS "...${PROGRAM}")
  
  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM}  PROPERTIES
    FOLDER tests
    COMPILE_FLAGS "${COMPILER_FLAGS}"
    LINK_FLAGS "${LINKER_FLAGS}")
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
  add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

  set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
endforeach()
