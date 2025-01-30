# Demo programs for sequential RoboSimian simulations
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_IRRLICHT)
  message(STATUS "  Warning: the sequential RoboSimian projects require Chrono::Irrlicht")
  return()
endif()

# ------------------------------------------------------------------------------

set(DEMOS
    demo_ROBOT_RoboSimian_Rigid
    demo_ROBOT_RoboSimian_SCM
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM} "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_vehicle
                          ChronoEngine_irrlicht
                          ChronoModels_robot)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
