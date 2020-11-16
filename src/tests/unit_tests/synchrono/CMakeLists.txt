SET(LIBRARIES ChronoEngine ChronoEngine_synchrono ${SYNCHRONO_LIBRARIES})
INCLUDE_DIRECTORIES( ${CH_INCLUDES} ${CH_SYNCHRONO_INCLUDES} )

SET(TESTS
    utest_SYN_MPI
    utest_SYN_agent_initialization
)

MESSAGE(STATUS "Unit test programs for SYNCHRONO module...")

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
    # Note: we don't add these to the list of tests to get run
ENDFOREACH(PROGRAM)
