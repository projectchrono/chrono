set(TESTS
    utest_FSI_Poiseuille_flow
)
#--------------------------------------------------------------

include_directories(${CH_FSI_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_FSI_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_fsi")

if(ENABLE_MODULE_VSG)
    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
endif()

# A hack to set the working directory in which to execute the CTest runs.
# This is needed for tests that need to access the Chrono data directory
# (since we use a relative path to it).
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

# ------------------------------------------------------------------------------

message(STATUS "Test programs for FSI module...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    target_link_libraries(${PROGRAM} gtest_main)
    add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
    set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
