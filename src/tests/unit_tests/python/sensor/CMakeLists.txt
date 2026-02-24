set(TESTS
    pyutest_SEN_gps
)

if(CH_USE_SENSOR_OPTIX)
   set(TESTS ${TESTS}
       pyutest_SEN_data_access
   )
endif()

set(LIBS Chrono_core Chrono_sensor)

message(STATUS "Add PyChrono unit test programs for SENSOR module")
build_pyutests(YES "${TESTS}" "${LIBS}")
