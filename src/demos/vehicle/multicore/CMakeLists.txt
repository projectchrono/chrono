#=============================================================================
# CMake configuration file for demos of Chrono::Vehicle with Chrono::Multicore.
# These example use the VSG module for run-time visualization.
#=============================================================================

if(NOT CH_ENABLE_MODULE_MULTICORE OR NOT CH_ENABLE_MODULE_VSG)
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
                          Chrono_multicore
                          Chrono_vsg
                          Chrono_vehicle
                          ChronoModels_vehicle)
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
