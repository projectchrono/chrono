SET(TESTS
    pyutest_SEN_gps
    pyutest_SEN_data_access
)

set(LIBS Chrono_core Chrono_sensor)

MESSAGE(STATUS "Add PyChrono unit test programs for SENSOR module")
build_pyutests(YES "${TESTS}" "${LIBS}")
