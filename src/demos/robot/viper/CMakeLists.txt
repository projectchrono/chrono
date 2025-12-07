if(NOT CH_ENABLE_MODULE_VEHICLE)
  return()
endif()

if(NOT CH_ENABLE_MODULE_IRRLICHT AND NOT CH_ENABLE_MODULE_VSG)
  return()
endif()

# ------------------------------------------------------------------------------

set(DEMOS ${DEMOS}
  demo_ROBOT_Viper_Rigid
  demo_ROBOT_Viper_SCM
)

if(CH_ENABLE_MODULE_SENSOR)
  set(DEMOS ${DEMOS}
      demo_ROBOT_Viper_SCM_Sensor
  )
endif()

if(CH_ENABLE_MODULE_FSI AND CH_ENABLE_MODULE_VSG)
  set(DEMOS ${DEMOS}
      demo_ROBOT_Viper_CRM
      demo_ROBOT_ViperWheel_CRM
      demo_ROBOT_ViperWheel_CRM_slope
  )
endif()

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "ChronoModels_robot")

if(CH_ENABLE_MODULE_VEHICLE)
  list(APPEND LIBS "Chrono_vehicle")
endif()

if(CH_ENABLE_MODULE_SENSOR)
  list(APPEND LIBS "Chrono_sensor")
endif()

if(CH_ENABLE_MODULE_FSI_SPH)
  list(APPEND LIBS "Chrono_fsisph")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS "Chrono_mumps")
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
    if(CH_ENABLE_MODULE_FSI_SPH)
      list(APPEND LIBS "Chrono_fsisph_vsg")
    endif()
endif()

# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()
