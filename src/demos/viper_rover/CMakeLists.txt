# Demo programs for sequential RoboSimian simulations
# ------------------------------------------------------------------------------
message(STATUS "Demo programs for VIPER model...")

if(NOT ENABLE_MODULE_VEHICLE)
  message("  Warning: the Viper_Rover projects require Chrono::Vehicle!")
  return()
endif()

if(NOT ENABLE_MODULE_IRRLICHT)
  message("  Warning: the Viper_Rover projects require Chrono::Irrlicht!")
  return()
endif()


# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_IRRLICHTINC})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

# ------------------------------------------------------------------------------

set(DEMOS
    demo_Viper_Rigid
    demo_Viper_SCM
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_vehicle
                          ChronoModels_viper
                          ChronoEngine_irrlicht
                        )

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
