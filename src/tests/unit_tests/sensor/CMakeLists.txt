set(TESTS
    utest_SEN_gps
)

if(CH_USE_SENSOR_OPTIX)
  set(TESTS ${TESTS}
    utest_SEN_interface
    utest_SEN_optixengine
    utest_SEN_optixgeometry
    utest_SEN_optixpipeline
    utest_SEN_radar
    utest_SEN_threadsafety      
  )
endif()

set(LIBS Chrono_core Chrono_sensor)

message(STATUS "Add unit test programs for SENSOR module")
build_utests(YES "${TESTS}" "${LIBS}")
