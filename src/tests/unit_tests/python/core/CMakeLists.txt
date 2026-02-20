SET(TESTS
    pyutest_CH_ChVector
)

SET(LIBS Chrono_core)

MESSAGE(STATUS "Add PyChrono unit test programs for CORE module")
build_pyutests(YES "${TESTS}" "${LIBS}")
