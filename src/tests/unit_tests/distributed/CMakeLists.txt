# Unit tests for the Chrono::Distributed module
# ==================================================================

#--------------------------------------------------------------
# Additional include paths (for ChronoEngine_Utils)
INCLUDE_DIRECTORIES(${CH_DISTRIBUTED_INCLUDES} ${CH_MULTICORE_INCLUDES})

# Libraries
SET(LIBRARIES
    ChronoEngine
    ChronoEngine_multicore
    ChronoEngine_distributed
)

#--------------------------------------------------------------
# List of all executables

SET(TESTS
	utest_DISTR_collision
)

MESSAGE(STATUS "Unit test programs for DISTRIBUTED module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_DISTRIBUTED_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
