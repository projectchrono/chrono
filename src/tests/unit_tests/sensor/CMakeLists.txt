SET(LIBRARIES ChronoEngine ChronoEngine_sensor ${SENSOR_LIBRARIES})
INCLUDE_DIRECTORIES( ${CH_INCLUDES} ${CH_SENSOR_INCLUDES} )

SET(TESTS
    utest_SEN_gps
    utest_SEN_interface
    utest_SEN_optixengine
    utest_SEN_optixgeometry
    utest_SEN_optixpipeline
    utest_SEN_threadsafety    
    utest_SEN_radar
)

MESSAGE(STATUS "Unit test programs for SENSOR module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
