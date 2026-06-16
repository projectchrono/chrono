
# NOTE (schema-driven rewrite, see src/chrono_ros/CLAUDE.md): demos are restored
# in Phase 5 as the built-in handlers they exercise are re-delivered. Done so
# far: custom handler (clock/body/tf + custom pub/sub) and the sensor demo
# (IMU/GPS family). Camera/lidar, vehicle, viper, etc. return in later batches.
set(DEMOS
  demo_ROS_custom_handler
)

# demo_ROS_sensor exercises the IMU/GPS sensor handlers, which require Chrono::Sensor.
if(CH_ENABLE_MODULE_SENSOR)
  list(APPEND DEMOS demo_ROS_sensor)
endif()

# ------------------------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_ros")
list(APPEND LIBS "ChronoModels_robot")

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_irrlicht")
    if (CH_ENABLE_MODULE_VEHICLE)
        list(APPEND LIBS "Chrono_vehicle")
        list(APPEND LIBS "Chrono_vehicle_irrlicht")
    endif()
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vsg")
    if (CH_ENABLE_MODULE_VEHICLE)
        list(APPEND LIBS "Chrono_vehicle")
        list(APPEND LIBS "Chrono_vehicle_vsg")
    endif()
endif()

if(CH_ENABLE_MODULE_SENSOR)
  list(APPEND LIBS "Chrono_sensor")
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

message(STATUS "Add demo programs for ROS module")

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

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
