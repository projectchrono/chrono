
IF(USE_TENSOR_RT AND ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE AND ENABLE_MODULE_VEHICLE_MODELS)

  #--------------------------------------------------------------

  # Add executables

  MESSAGE(STATUS "RL inference demo...")

  INCLUDE_DIRECTORIES(${CH_IRRLICHT_INCLUDES})

  ADD_EXECUTABLE(demo_SEN_rl_infer
    demo_SEN_rl_infer.cpp
    InferenceDriver.cpp)

  SOURCE_GROUP(""  FILES  "demo_SEN_rl_infer.cpp
    InferenceDriver.cpp
    InferenceDriver.h
  ")

  # Visual Studio needs to have the debugger working directory set explicitly to the output directory
  # (specifically, so that the data directory is found when running demos in the debugger)
  IF(MSVC)
    set_property(TARGET demo_SEN_rl_infer PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$(OutputPath)")
    set_target_properties(demo_SEN_rl_infer PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  ENDIF()

  TARGET_LINK_LIBRARIES(demo_SEN_rl_infer
    ChronoEngine
    ChronoEngine_sensor
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ChronoEngine_vehicle_irrlicht
    ChronoModels_vehicle
    )
  ADD_DEPENDENCIES (demo_SEN_rl_infer
      ChronoEngine
      ChronoEngine_sensor
      ChronoEngine_irrlicht
      ChronoEngine_vehicle
      ChronoEngine_vehicle_irrlicht
      ChronoModels_vehicle
  )


  INSTALL(TARGETS demo_SEN_rl_infer DESTINATION ${CH_INSTALL_DEMO})

ENDIF()
