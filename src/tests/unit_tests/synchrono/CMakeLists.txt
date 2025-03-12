SET(LIBRARIES Chrono_core Chrono_synchrono ${SYNCHRONO_LIBRARIES})

SET(TESTS
    utest_SYN_MPI
    utest_SYN_agent_initialization
)

MESSAGE(STATUS "Add unit test programs for SYNCHRONO module")

FOREACH(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    # Note: we don't add these to the list of tests to get run
ENDFOREACH(PROGRAM)
