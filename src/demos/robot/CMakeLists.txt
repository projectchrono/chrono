# Demo programs for simulations with models from the Chrono::Robots library
# ------------------------------------------------------------------------------

# MESSAGE(STATUS "Demo programs for ROBOT models...")

set(DEMOS "")

if(ENABLE_MODULE_IRRLICHT)
  set(DEMOS ${DEMOS}
      demo_ROBOT_LittleHexy
  )
endif()

# ------------------------------------------------------------------------------

if(ENABLE_MODULE_VEHICLE)

	if(ENABLE_MODULE_IRRLICHT OR ENABLE_MODULE_VSG)
	  set(DEMOS ${DEMOS}
		  demo_ROBOT_Curiosity_Rigid
		  demo_ROBOT_Curiosity_SCM    
		  demo_ROBOT_Turtlebot_Rigid
		  demo_ROBOT_Viper_Rigid
		  demo_ROBOT_Viper_SCM
	  )

	  if(ENABLE_MODULE_SENSOR)
		set(DEMOS ${DEMOS}
			demo_ROBOT_Curiosity_SCM_Sensor
			demo_ROBOT_Viper_SCM_Sensor
		)
	  endif()
	endif()

	if(ENABLE_MODULE_VSG OR ENABLE_MODULE_OPENGL)
	  if(ENABLE_MODULE_FSI)
		set(DEMOS ${DEMOS}
			demo_ROBOT_Viper_SPH
		)
	  endif()
	endif()

endif()

# ------------------------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoModels_robot")
if(ENABLE_MODULE_VEHICLE)
  list(APPEND LIBS "ChronoEngine_vehicle")
endif()

if(ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "ChronoEngine_irrlicht")
    if(ENABLE_MODULE_VEHICLE)
      list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
    endif()
endif()

if(ENABLE_MODULE_VSG)
    list(APPEND LIBS "ChronoEngine_vsg")
    if(ENABLE_MODULE_VEHICLE) 
      list(APPEND LIBS "ChronoEngine_vehicle_vsg")
    endif()
endif()

if(ENABLE_MODULE_OPENGL)
    list(APPEND LIBS "ChronoEngine_opengl")
endif()

if(ENABLE_MODULE_SENSOR)
  list(APPEND LIBS "ChronoEngine_sensor")
endif()

if(ENABLE_MODULE_FSI)
  list(APPEND LIBS "ChronoEngine_fsi_sph")
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

#MESSAGE(STATUS "Demo programs for ROBOTICS...")

foreach(PROGRAM ${DEMOS})
    #MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()

# ------------------------------------------------------------------------------
# Add subdirectories

add_subdirectory(industrial)
add_subdirectory(robosimian)
