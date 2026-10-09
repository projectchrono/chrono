set(TESTS
    utest_FSI_step_exception
)

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsi)

message(STATUS "Add unit test programs for FSI base module")
build_utests(YES "${TESTS}" "fsi" "${LIBS}")
