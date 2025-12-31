set(TESTS
    utest_SMC_cohesion
    utest_SMC_cor_normal
    utest_SMC_rolling_gravity
    utest_SMC_sliding_gravity
    utest_SMC_spinning_gravity
    utest_SMC_stacking
    utest_SMC_sphere_sphere
)

set(UTILS
   ../utest_SMC.h)

set(LIBS Chrono_core)

message(STATUS "Add unit test programs for SMC contact in CORE module")
build_utests(NO "${TESTS}" "${LIBS}" "${UTILS}")
