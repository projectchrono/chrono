set(TESTS
    utest_FSI-SPH_Poiseuille_flow
    utest_FSI-SPH_domain_guard
    utest_FSI-SPH_domain_update
    utest_FSI-SPH_setter_guard
    utest_FSI-SPH_rheology_error_flag
)

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsisph)
if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS Chrono_fsisph_vsg)
endif()

message(STATUS "Add unit test programs for FSI module")
build_utests(YES "${TESTS}" "sph" "${LIBS}")
