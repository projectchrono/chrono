SET(TESTS
    utest_CH_slider_pend
    utest_CH_double_pend
    utest_CH_shafts
    utest_CH_compute_contact
    utest_CH_assembly
    utest_CH_composite_inertia
)

SET(LIBS Chrono_core)

MESSAGE(STATUS "Add unit test programs for PHYSICS module")
build_utests(YES "${TESTS}" "${LIBS}")
