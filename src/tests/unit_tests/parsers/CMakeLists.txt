set(LIBRARIES ChronoEngine ChronoEngine_parsers)
include_directories( ${CH_INCLUDES} ${CH_PARSERS_INCLUDES} )

set(TESTS "")

if(HAVE_URDF)
  SET(TESTS
    ${TESTS}
    utest_PARSERS_URDF_simple
    ##utest_PARSERS_URDF_package
  )
endif()

if(HAVE_ROS)
  add_definitions(-DHAVE_ROS)
endif()

message(STATUS "Unit test programs for Parsers module...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

    # Update the working dir for tests, so that they can find the Chrono data
    set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/bin"
    )
endforeach(PROGRAM)
