# Demo programs for sequential Viper simulations
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_VEHICLE)
  message(STATUS "  Warning: the ViperRover projects require Chrono::Vehicle!")
  return()
endif()

if(NOT ENABLE_MODULE_IRRLICHT)
  message(STATUS "  Warning: the ViperRover projects require Chrono::Irrlicht!")
  return()
endif()


# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_IRRLICHTINC})
set(LIBRARIES 
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_robot
    ChronoEngine_irrlicht
)

# ------------------------------------------------------------------------------

set(DEMOS
    demo_ROBOT_Viper_Rigid
    demo_ROBOT_Viper_SCM
)

if(ENABLE_MODULE_SENSOR)
  set(DEMOS ${DEMOS}
      demo_ROBOT_Viper_SCM_Sensor
  )
  include_directories(${CH_SENSOR_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_sensor)
endif()

if(ENABLE_MODULE_FSI)
  set(DEMOS ${DEMOS}
      demo_ROBOT_Viper_SPH
  )
  include_directories(${CH_FSI_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_fsi)
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM} ${LIBRARIES})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
