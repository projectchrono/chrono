# Demo programs for multicore RoboSimian simulations
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_MULTICORE OR NOT ENABLE_MODULE_OPENGL)
  message(STATUS "  Warning: the multicore RoboSimian projects require Chrono::Multicore and Chrono::OpenGL!")
  return()
endif()

# ------------------------------------------------------------------------------

set(DEMOS
    demo_ROBOT_RoboSimian_Granular
)

set(UTILITIES
    granular.h
    granular.cpp
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} "${PROGRAM}.cpp" ${UTILITIES})
    source_group(""  FILES "${PROGRAM}.cpp" ${UTILITIES})
    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_multicore
                          ChronoEngine_vehicle
                          ChronoEngine_opengl
                          ChronoModels_robot)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
