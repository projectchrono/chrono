set(TESTS
    pyutest_CAS_create
)

set(LIBS Chrono_core Chrono_cascade)

message(STATUS "Add PyChrono unit test programs for CASCADE module")
build_pyutests(YES "${TESTS}" "${LIBS}")
