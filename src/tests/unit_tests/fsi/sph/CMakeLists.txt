set(TESTS
    utest_FSI_Poiseuille_flow
)

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsisph)
if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS Chrono_fsisph_vsg)
endif()

message(STATUS "Add unit test programs for FSI module")
build_utests(YES "${TESTS}" "${LIBS}")
