set(TESTS
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

set(UTILS
   ../ut_utils.h)

set(LIBS Chrono_core Chrono_multicore)
if(CH_ENABLE_MODULE_VSG)
    set(LIBS ${LIBS} Chrono_vsg)
endif()

message(STATUS "Add unit test programs for MULTICORE module")
build_utests(YES "${TESTS}" "${LIBS}" "${UTILS}")
