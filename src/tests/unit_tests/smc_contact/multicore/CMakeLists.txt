set(TESTS
    utest_SMCm_cohesion
    utest_SMCm_cor_normal
    utest_SMCm_rolling_gravity
    utest_SMCm_sliding_gravity
    utest_SMCm_spinning_gravity
    utest_SMCm_stacking
)

set(UTILS
   ../utest_SMC.h)

set(LIBS Chrono_core Chrono_multicore)

message(STATUS "Add unit test programs for SMC contact in MULTICORE module")
build_utests(YES "${TESTS}" "${LIBS}" "${UTILS}")
 