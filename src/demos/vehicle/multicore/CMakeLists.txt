#=============================================================================
# CMake configuration file for demos of Chrono::Vehicle with Chrono::Multicore.
# These example use the OpenGL module for run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_MULTICORE OR NOT ENABLE_MODULE_OPENGL)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_Multicore_HMMWV
    demo_VEH_Multicore_M113
    demo_VEH_Multicore_TireTestRig
)

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_multicore
                          ChronoEngine_opengl
                          ChronoEngine_vehicle
                          ChronoModels_vehicle)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
