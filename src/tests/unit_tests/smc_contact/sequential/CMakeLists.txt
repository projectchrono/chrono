# Unit tests for the SMC contact with the core Chrono module
# ==================================================================

#--------------------------------------------------------------
# List of all executables

SET(TESTS
    utest_SMC_cohesion
    utest_SMC_cor_normal
    utest_SMC_rolling_gravity
    utest_SMC_sliding_gravity
    utest_SMC_spinning_gravity
    utest_SMC_stacking
    utest_SMC_sphere_sphere
)

MESSAGE(STATUS "Add unit test programs for SMC contact in CORE module")

FOREACH(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp" ../utest_SMC.h)
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp" ../utest_SMC.h)

    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} Chrono_core gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    ###ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
 