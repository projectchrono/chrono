# Unit tests for the Chrono::Multicore module
# ==================================================================

SET(LIBRARIES Chrono_core Chrono_multicore)

IF (CH_ENABLE_MODULE_OPENGL)
    SET(LIBRARIES ${LIBRARIES} Chrono_opengl)
ENDIF()

#--------------------------------------------------------------
# List of all executables

SET(TESTS
    utest_MCORE_real3
    utest_MCORE_real4
    utest_MCORE_matrix
    utest_MCORE_gravity
    utest_MCORE_shafts
    utest_MCORE_rotmotors
    utest_MCORE_other_math
    utest_MCORE_linactuator
    utest_MCORE_bodyauxref
    utest_MCORE_joints_dvi
    utest_MCORE_narrowphase
    utest_MCORE_jacobians
    utest_MCORE_contact_forces
)

MESSAGE(STATUS "Add unit test programs for MULTICORE module")

FOREACH(PROGRAM ${TESTS})   
    set(FILES ${PROGRAM}.cpp ../ut_utils.h)
    ADD_EXECUTABLE(${PROGRAM} ${FILES})
    SOURCE_GROUP("" FILES ${FILES})

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES FOLDER tests)
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
