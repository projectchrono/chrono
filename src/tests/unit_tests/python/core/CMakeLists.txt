SET(TESTS
    pyutest_CH_ChVector
    pyutest_CH_linalg
    pyutest_CH_numpy
)

SET(LIBS Chrono_core)

MESSAGE(STATUS "Add PyChrono unit test programs for CORE module")
build_pyutests(YES "${TESTS}" "${LIBS}")
