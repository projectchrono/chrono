
# NOTE (schema-driven rewrite, see src/chrono_ros/CLAUDE.md): demos are restored
# in Phase 5 as the built-in handlers they exercise are re-delivered. Each demo
# is gated on the Chrono modules its handlers require, so the set scales with the
# enabled modules. Core demos (custom handler, hydraulic crane) are always built.
set(DEMOS
  demo_ROS_custom_handler
  demo_ROS_hydraulic_crane
)

# demo_ROS_sensor exercises the IMU/GPS/camera/lidar handlers (Chrono::Sensor).
if(CH_ENABLE_MODULE_SENSOR)
  list(APPEND DEMOS demo_ROS_sensor)
endif()

# demo_ROS_urdf exercises RobotModel (ChParserURDF ctor) + TF AddURDF.
if(CH_ENABLE_MODULE_PARSERS AND CH_USE_URDF)
  list(APPEND DEMOS demo_ROS_urdf)
endif()

# demo_ROS_viper exercises the Viper rover motor-control handler; needs the
# robot models library.
if(TARGET ChronoModels_robot)
  list(APPEND DEMOS demo_ROS_viper)
endif()

# Vehicle demos: demo_ROS_vehicle (JSON Audi) needs Chrono::Vehicle;
# demo_ROS_two_managers additionally needs the HMMWV model (ChronoModels_vehicle).
if(CH_ENABLE_MODULE_VEHICLE)
  list(APPEND DEMOS demo_ROS_vehicle)
  if(TARGET ChronoModels_vehicle)
    list(APPEND DEMOS demo_ROS_two_managers)
  endif()
endif()

# ------------------------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_ros")

if(TARGET ChronoModels_robot)
  list(APPEND LIBS "ChronoModels_robot")
endif()

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

# Chrono::Vehicle is needed by the vehicle demos regardless of whether a run-time
# visualization module is enabled (they can run headless).
if(CH_ENABLE_MODULE_VEHICLE)
    list(APPEND LIBS "Chrono_vehicle")
    if(TARGET ChronoModels_vehicle)
        list(APPEND LIBS "ChronoModels_vehicle")
    endif()
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_irrlicht")
    if (CH_ENABLE_MODULE_VEHICLE)
        list(APPEND LIBS "Chrono_vehicle_irrlicht")
    endif()
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vsg")
    if (CH_ENABLE_MODULE_VEHICLE)
        list(APPEND LIBS "Chrono_vehicle_vsg")
    endif()
endif()

if(CH_ENABLE_MODULE_SENSOR)
  list(APPEND LIBS "Chrono_sensor")
endif()

if(CH_ENABLE_MODULE_PARSERS)
  list(APPEND LIBS "Chrono_parsers")
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
