message(STATUS "Unit test programs for Vehicle module...")

set(TESTS
    utest_VEH_destructors
)

#--------------------------------------------------------------

# A hack to set the working directory in which to execute the CTest runs.
# This is needed for tests that need to access the Chrono data directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

#--------------------------------------------------------------

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_vehicle")

#--------------------------------------------------------------
# Add executables

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )
    
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS} gtest_main)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ##add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
    ##set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
endforeach(PROGRAM)
