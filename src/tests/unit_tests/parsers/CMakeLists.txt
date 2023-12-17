SET(LIBRARIES ChronoEngine ChronoEngine_parsers)
INCLUDE_DIRECTORIES( ${CH_INCLUDES} ${CH_PARSERS_INCLUDES} )

SET(TESTS
  utest_PARSERS_URDF_simple
  utest_PARSERS_URDF_package
)

if(HAVE_ROS)
  add_definitions(-DHAVE_ROS)
endif()

MESSAGE(STATUS "Unit test programs for Parsers module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

    # Update the working dir for tests, so that they can find the Chrono data
    SET_TESTS_PROPERTIES(${PROGRAM} PROPERTIES
        WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/bin"
    )
ENDFOREACH(PROGRAM)
