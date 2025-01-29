set(TESTS "")

if(HAVE_URDF)
  SET(TESTS
    ${TESTS}
    utest_PARSERS_URDF_simple
    utest_PARSERS_URDF_package
  )
endif()

if(HAVE_ROS)
  add_definitions(-DHAVE_ROS)
endif()

#--------------------------------------------------------------

# A hack to set the working directory in which to execute the CTest
# runs.  This is needed for tests that need to access the Chrono data
# directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

#--------------------------------------------------------------

set(LIBRARIES ChronoEngine ChronoEngine_parsers)

#--------------------------------------------------------------
# Add executables

MESSAGE(STATUS "Add unit test programs for PARSERS module")

foreach(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES FOLDER tests)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

    set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
endforeach(PROGRAM)
