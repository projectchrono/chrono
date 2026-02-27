set(TESTS
    pyutest_FSI-SPH_Poiseuille_flow
    pyutest_FSI-SPH_multithread
)

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsisph)
if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS Chrono_fsisph_vsg)
endif()

message(STATUS "Add PyChrono unit test programs for FSI module")
build_pyutests(YES "${TESTS}" "${LIBS}")
