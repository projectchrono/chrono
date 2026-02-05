SET(TESTS
    pyutest_CH_compute_contact
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

MESSAGE(STATUS "Add PyChrono unit test programs for PHYSICS module")
build_pyutests(YES "${TESTS}" "${LIBS}")
