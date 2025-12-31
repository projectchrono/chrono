set(TESTS
    utest_MOD_eigensolver
    utest_MOD_curved_beam
)

set(LIBS Chrono_core Chrono_modal)

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS Chrono_pardisomkl)
endif()

message(STATUS "Add unit test programs for MODAL module")
build_utests(YES "${TESTS}" "${LIBS}")
