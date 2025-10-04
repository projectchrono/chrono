set(LIBRARIES Chrono_core Chrono_sensor)

set(TESTS
    utest_SEN_gps
    utest_SEN_interface
    utest_SEN_optixengine
    utest_SEN_optixgeometry
    utest_SEN_optixpipeline
    utest_SEN_threadsafety    
    utest_SEN_radar
)

#--------------------------------------------------------------

# A hack to set the working directory in which to execute the CTest runs.
# This is needed for tests that need to access the Chrono sensor_shaders directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

#--------------------------------------------------------------
# Add executables

message(STATUS "Add unit test programs for SENSOR module")

foreach(PROGRAM ${TESTS})
    #message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES FOLDER tests)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

    add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
    
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
ENDFOREACH(PROGRAM)
