set(TESTS
    pyutest_VEH_tire_contact
)

set(LIBS Chrono_core Chrono_vehicle)

message(STATUS "Add PyChrono unit test programs for VEHICLE module")
build_pyutests(YES "${TESTS}" "${LIBS}")
