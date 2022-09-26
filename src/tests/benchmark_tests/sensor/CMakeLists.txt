if(NOT ENABLE_MODULE_SENSOR)
    return()
endif()


SET(LIBRARIES ChronoEngine ChronoEngine_sensor ${SENSOR_LIBRARIES})
INCLUDE_DIRECTORIES( ${CH_INCLUDES} ${CH_SENSOR_INCLUDES} )


# message(STATUS "\n\nbenchmark include directories: ${CH_IRRLICHTINC}\n\n")
# ------------------------------------------------------------------------------

set(TESTS
    btest_SEN_perf_meshes
    btest_SEN_instanced_meshes
    btest_SEN_perf_reflections
    btest_SEN_real_sphere
    btest_SEN_lidar_beam
    btest_SEN_scene_scale
    btest_SEN_lidar_spin
    btest_SEN_cornell_box
    btest_SEN_vis_materials
    btest_SEN_camera_lens
    )

# ------------------------------------------------------------------------------

MESSAGE(STATUS "benchmark test programs for SENSOR module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} benchmark_main)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH(PROGRAM)
