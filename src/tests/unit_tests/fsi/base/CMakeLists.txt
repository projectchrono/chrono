set(TESTS
    utest_FSI_step_exception
)

if(CH_ENABLE_MODULE_FEA)
  list(APPEND TESTS utest_FSI_mesh_forces)
endif()

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsi)

message(STATUS "Add unit test programs for FSI base module")
build_utests(YES "${TESTS}" "fsi" "${LIBS}")
