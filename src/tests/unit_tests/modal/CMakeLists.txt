set(TESTS
)

if(CH_ENABLE_MODULE_FEA)
    set(TESTS ${TESTS}
        utest_MOD_eigensolver
        utest_MOD_curved_beam
        utest_MOD_cache_reduced
    )
endif()

set(LIBS Chrono_core Chrono_modal)

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS Chrono_pardisomkl)
endif()

message(STATUS "Add unit test programs for MODAL module")
build_utests(YES "${TESTS}" "modal" "${LIBS}")
