# Demo programs for simulations with models from the Chrono::Robots library
# ------------------------------------------------------------------------------

set(DEMOS "")

if(CH_ENABLE_MODULE_IRRLICHT)
  set(DEMOS ${DEMOS}
      demo_ROBOT_LittleHexy
  )
endif()

# ------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VEHICLE)

  if(CH_ENABLE_MODULE_IRRLICHT OR CH_ENABLE_MODULE_VSG)
    set(DEMOS ${DEMOS}
        demo_ROBOT_Curiosity_Rigid
        demo_ROBOT_Curiosity_SCM    
        demo_ROBOT_Turtlebot_Rigid
        demo_ROBOT_Viper_Rigid
        demo_ROBOT_Viper_SCM
    )

    if(CH_ENABLE_MODULE_SENSOR)
      set(DEMOS ${DEMOS}
          demo_ROBOT_Curiosity_SCM_Sensor
          demo_ROBOT_Viper_SCM_Sensor
      )
    endif()
  endif()

  if(CH_ENABLE_MODULE_FSI AND CH_ENABLE_MODULE_VSG)
    set(DEMOS ${DEMOS}
        demo_ROBOT_Viper_CRM
    )
  endif()

endif()

# ------------------------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "ChronoModels_robot")

if(CH_ENABLE_MODULE_VEHICLE)
  list(APPEND LIBS "Chrono_vehicle")
endif()

if(CH_ENABLE_MODULE_SENSOR)
  list(APPEND LIBS "Chrono_sensor")
endif()

if(CH_ENABLE_MODULE_FSI)
  list(APPEND LIBS "Chrono_fsisph")
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_irrlicht")
    if(CH_ENABLE_MODULE_VEHICLE)
      list(APPEND LIBS "Chrono_vehicle_irrlicht")
    endif()
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vsg")
    if(CH_ENABLE_MODULE_VEHICLE) 
      list(APPEND LIBS "Chrono_vehicle_vsg")
    endif()
    if(CH_ENABLE_MODULE_FSI)
      list(APPEND LIBS "Chrono_fsisph_vsg")
    endif()
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

message(STATUS "Add demo programs for ROBOT models")

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()

# ------------------------------------------------------------------------------
# Add subdirectories

add_subdirectory(industrial)
add_subdirectory(robosimian)
