#--------------------------------------------------------------
# List of all sensor demos

set(DEMOS
  demo_SEN_GPSIMU
)

if(CH_USE_SENSOR_OPTIX)

  set(DEMOS ${DEMOS}
      demo_SEN_buildtest
      demo_SEN_JSON
      demo_SEN_camera
      demo_SEN_cornell_box
      demo_SEN_phys_cam
      demo_SEN_lidar
      demo_SEN_radar
      demo_SEN_radar1
      demo_SEN_tachometer  
  )

  # Demos that combine sensor simulation with vehicle dynamics
  if(CH_ENABLE_MODULE_IRRLICHT AND CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
      set(DEMOS ${DEMOS}
        demo_SEN_HMMWV
        demo_SEN_Gator
        demo_SEN_deformableSoil
      )
  endif()

  # demo_SEN_CRM_Rendering uses chrono::vehicle::CRMTerrain directly, so it
  # must only be built when the Vehicle module is enabled and it must link
  # Chrono_vehicle explicitly (see target-specific link below).
  if(CH_ENABLE_MODULE_FSI_SPH AND CH_ENABLE_MODULE_VEHICLE)
      set(DEMOS ${DEMOS}
        demo_SEN_CRM_Rendering
      )
  elseif(CH_ENABLE_MODULE_FSI_SPH)
    message(WARNING "Skipping demo_SEN_CRM_Rendering because it requires CH_ENABLE_MODULE_VEHICLE=ON")
  endif()
  
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBRARIES Chrono_core)
list(APPEND LIBRARIES "ChronoModels_robot")
list(APPEND LIBRARIES "Chrono_sensor")

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBRARIES "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_VEHICLE) 
   if(CH_ENABLE_MODULE_IRRLICHT)
      list(APPEND LIBRARIES "Chrono_vehicle")
      list(APPEND LIBRARIES "Chrono_vehicle_irrlicht")
   endif()
   if(CH_ENABLE_MODULE_VEHICLE_MODELS)
      list(APPEND LIBRARIES "ChronoModels_vehicle")
   endif()
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBRARIES "Chrono_vsg")
    if(CH_ENABLE_MODULE_VEHICLE) 
      list(APPEND LIBRARIES "Chrono_vehicle_vsg")
    endif()
    if(CH_ENABLE_MODULE_FSI_SPH)
      list(APPEND LIBRARIES "Chrono_fsisph_vsg")
    endif()
endif()

#--------------------------------------------------------------
# Add executables

message(STATUS "Add demo programs for SENSOR module")

foreach(PROGRAM ${DEMOS})
  # MESSAGE(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  # Visual Studio needs to have the debugger working directory set explicitly to the output directory
  # (specifically, so that the data directory is found when running demos in the debugger)
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBRARIES})
  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

  install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})

endforeach()
