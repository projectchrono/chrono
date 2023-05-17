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
if(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE AND ENABLE_MODULE_VEHICLE_MODELS)
    set(DEMOS ${DEMOS}
      demo_SEN_HMMWV
      demo_SEN_Gator
      demo_SEN_deformableSoil
    )
endif()

# Demos that require tensorrt
if(USE_TENSOR_RT)
    set(DEMOS ${DEMOS}
      demo_SEN_NNCamera
    )
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBRARIES "ChronoEngine")
list(APPEND LIBRARIES "ChronoEngine_sensor")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHT_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_VEHICLE) 
   if(ENABLE_MODULE_IRRLICHT)
      list(APPEND LIBRARIES "ChronoEngine_vehicle_irrlicht")
   endif()
   if(ENABLE_MODULE_VEHICLE_MODELS)
      list(APPEND LIBRARIES "ChronoModels_vehicle")
   endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(LINKER_FLAGS  "${LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

#--------------------------------------------------------------
# Add executables

message(STATUS "Demo programs for Sensor module...")

foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  # Visual Studio needs to have the debugger working directory set explicitly to the output directory
  # (specifically, so that the data directory is found when running demos in the debugger)
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

  set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
  target_link_libraries(${PROGRAM} ${LIBRARIES})

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

endforeach()

#--------------------------------------------------------------
# Demos that are self-determining

add_subdirectory(demo_SEN_rl_infer)
