if(NOT CH_ENABLE_MODULE_SENSOR)
    return()
endif()

set(LIBRARIES Chrono_core Chrono_sensor ${SENSOR_LIBRARIES})

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

MESSAGE(STATUS "Add benchmark test programs for SENSOR module")

FOREACH(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} benchmark_main)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH(PROGRAM)
