SET(TESTS
    utest_CH_slider_pend
    utest_CH_double_pend
    utest_CH_shafts
    utest_CH_compute_contact
    utest_CH_assembly
    utest_CH_composite_inertia
)

if(CH_ENABLE_MODULE_PARDISO_MKL)
  set(TESTS ${TESTS}
    utest_CH_gears
  )
endif()

set(LIBS Chrono_core)
if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS Chrono_pardisomkl)
endif()


MESSAGE(STATUS "Add unit test programs for PHYSICS module")
build_utests(YES "${TESTS}" "${LIBS}")
