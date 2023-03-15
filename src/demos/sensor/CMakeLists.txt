#--------------------------------------------------------------
# List of all sensor demos



list(APPEND CHRONO_LIBRARIES "ChronoEngine")
list(APPEND CHRONO_LIBRARIES "ChronoEngine_sensor")

set(CHRONO_COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_SENSOR_CXX_FLAGS}" )
set(CHRONO_LINK_FLAGS ${CH_LINKERFLAG_EXE})

#List of demos that show the sensor framework
SET(DEMOS
  demo_SEN_buildtest
  demo_SEN_camera
  demo_SEN_JSON
  demo_SEN_lidar
  demo_SEN_GPSIMU
  demo_SEN_radar
  demo_SEN_tachometer
  demo_SEN_radar1
)

#list of demos that combine sensor simulation with vehicle dynamics
IF(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE)
    SET(DEMOS
      ${DEMOS}
      demo_SEN_HMMWV
      demo_SEN_Gator
      demo_SEN_deformableSoil
    )
    list(APPEND CHRONO_LIBRARIES "ChronoEngine_irrlicht")
    list(APPEND CHRONO_LIBRARIES "ChronoEngine_vehicle")
    list(APPEND CHRONO_LIBRARIES "ChronoEngine_vehicle_irrlicht")
    list(APPEND CHRONO_LIBRARIES "ChronoModels_vehicle")
    set(CHRONO_COMPILE_FLAGS "${CHRONO_COMPILE_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}" )
ENDIF()

#demos that require tensorrt
IF(USE_TENSOR_RT)
    SET(DEMOS
      ${DEMOS}
      demo_SEN_NNCamera
    )
ENDIF()

#--------------------------------------------------------------
# Additional include paths

# INCLUDE_DIRECTORIES( ${CH_IRRLICHT_INCLUDES} )

#--------------------------------------------------------------
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

MESSAGE(STATUS "Demo programs for Sensor module...")

#FOR OPTIONAL VEHICLE WITH IRRLICHT DEMO
IF(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE)
    INCLUDE_DIRECTORIES(${CH_IRRLICHT_INCLUDES})
ENDIF()

FOREACH(PROGRAM ${DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  # Visual Studio needs to have the debugger working directory set explicitly to the output directory
  # (specifically, so that the data directory is found when running demos in the debugger)
  SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
    COMPILE_FLAGS "${CHRONO_COMPILE_FLAGS}"
    LINK_FLAGS "${CHRONO_LINK_FLAGS}"
    )
  TARGET_LINK_LIBRARIES(${PROGRAM}
    ${CHRONO_LIBRARIES}
    )
  ADD_DEPENDENCIES (${PROGRAM}
      ${CHRONO_LIBRARIES}
  )

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()



#demos that are self-determining
add_subdirectory(demo_SEN_rl_infer)
