# Unit tests for the SMC contact with the Chrono::Multicore module
# ==================================================================

#--------------------------------------------------------------
# Additional include paths (for ChronoEngine_Utils)
INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})

# Libraries
SET(LIBRARIES
    ChronoEngine
    ChronoEngine_multicore
)

#--------------------------------------------------------------
# List of all executables

SET(TESTS
    utest_SMCm_cohesion
    utest_SMCm_cor_normal
    utest_SMCm_rolling_gravity
    utest_SMCm_sliding_gravity
    utest_SMCm_spinning_gravity
    utest_SMCm_stacking
)

MESSAGE(STATUS "Unit test programs for SMC contact in MULTICORE module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp" ../utest_SMC.h)
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp" ../utest_SMC.h)

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
 