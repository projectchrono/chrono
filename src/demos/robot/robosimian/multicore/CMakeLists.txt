# Demo programs for multicore RoboSimian simulations
# ------------------------------------------------------------------------------

if(NOT CH_ENABLE_MODULE_MULTICORE OR NOT CH_ENABLE_MODULE_VSG)
  message(STATUS "  Warning: the multicore RoboSimian projects require Chrono::Multicore and Chrono::VSG")
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
    add_executable(${PROGRAM} "${PROGRAM}.cpp" ${UTILITIES})
    source_group(""  FILES "${PROGRAM}.cpp" ${UTILITIES})
    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM}
                          Chrono_core
                          Chrono_multicore
                          Chrono_vehicle
                          Chrono_vsg
                          ChronoModels_robot)

                          
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()
