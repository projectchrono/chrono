if(NOT CH_ENABLE_MODULE_SENSOR)
    return()
endif()

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

set(LIBS Chrono_core Chrono_sensor ${SENSOR_LIBRARIES})

message(STATUS "Add benchmark test programs for SENSOR module")
build_btests("${TESTS}" "${LIBS}")
