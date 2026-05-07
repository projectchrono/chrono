set(TESTS
    utest_CAS_create
)

set(LIBS Chrono_core Chrono_cascade)

message(STATUS "Add unit test programs for CASCADE module")
build_utests(YES "${TESTS}" "cascade" "${LIBS}")
