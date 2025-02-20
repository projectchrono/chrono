#--------------------------------------------------------------
# List of all sensor demos

# Demos that show the sensor framework
set(DEMOS
  demo_SEN_buildtest
  demo_SEN_camera
  demo_SEN_JSON
  demo_SEN_lidar
  demo_SEN_GPSIMU
  demo_SEN_radar
  demo_SEN_tachometer
  demo_SEN_radar1
)

# Demos that combine sensor simulation with vehicle dynamics
if(CH_ENABLE_MODULE_IRRLICHT AND CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
    set(DEMOS ${DEMOS}
      demo_SEN_HMMWV
      demo_SEN_Gator
      demo_SEN_deformableSoil
    )
endif()

# Demos that require tensorrt
if(CH_USE_TENSOR_RT)
    set(DEMOS ${DEMOS}
      demo_SEN_NNCamera
    )
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBRARIES Chrono_core)
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

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

endforeach()

#--------------------------------------------------------------
# Demos that are self-determining

add_subdirectory(demo_SEN_rl_infer)
