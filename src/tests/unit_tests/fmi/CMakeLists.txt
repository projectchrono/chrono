set(TESTS
    utest_FMI_resource_path
)

set(LIBS Chrono_core Chrono_fmi)

message(STATUS "Add unit test programs for FMI module")
build_utests(YES "${TESTS}" "fmi" "${LIBS}")
