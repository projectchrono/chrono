# Unit tests for the Chrono::Multicore module
# ==================================================================

INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})
SET(LIBRARIES ChronoEngine ChronoEngine_multicore)

IF (ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)
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

if(USE_MULTICORE_CUDA)
   set(TESTS ${TESTS}
       utest_MCORE_svd
   )
endif()

MESSAGE(STATUS "Unit test programs for MULTICORE module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")
    
    set(FILES ${PROGRAM}.cpp ../ut_utils.h)
    ADD_EXECUTABLE(${PROGRAM} ${FILES})
    SOURCE_GROUP("" FILES ${FILES})

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
