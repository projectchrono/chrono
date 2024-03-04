#=============================================================================
# CMake configuration file for Chrono Vehicle FMI support
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

# Set common output directory for all Chrono::Vehicle FMUs
set(VEHICLE_FMU_COMPONENT_MAIN_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vehicleFMU")

# Generate configuration header ChConfigVehicleFMI.h
set(CHRONO_VEHICLE_FMU_DIR "#define CHRONO_VEHICLE_FMU_DIR \"${VEHICLE_FMU_COMPONENT_MAIN_DIR}/\"")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigVehicleFMI.h.in
               ${PROJECT_BINARY_DIR}/chrono_vehicle/ChConfigVehicleFMI.h)  

# List of FMU subdirectories
set(FMUS 
    wheeled_vehicle
    force_element_tire
    path_follower_driver
    )

# Create custom target with commands for copying FMU dependencies
add_custom_target(COPY_VEHICLE_FMU_DEPENDENCIES)
add_dependencies(COPY_VEHICLE_FMU_DEPENDENCIES ChronoEngine_vehicle)

message(STATUS "Add Chrono::Vehicle FMUs...")

# Create each FMU and add a custom command to copy its necessary dependencies.
# Note that we use a single custom target (COPY_VEHICLE_FMU_DEPENDENCIES) to hold all
# necessary custom commands for each FMU. The dependencies must be copied and available
# during the build process of the FMUs (since the FMU code must be executed in order
# to generate its model description). As such, the copying must happen in a pre-build
# phase. However, the target directories are created only during the FMU build process;
# to address this circular dependency, we also add explicit commands to create the output
# directories.

foreach(FMU ${FMUS})

  message(STATUS "...add ${FMU} FMU")

  add_subdirectory(${FMU})

  message(STATUS "   FMU_MODEL_IDENTIFIER: ${FMU_MODEL_IDENTIFIER}")
  message(STATUS "   FMU_DIRECTORY: ${FMU_DIRECTORY}")
  message(STATUS "   FMU_FILENAME: ${FMU_FILENAME}")
  message(STATUS "   FMU_RUNTIME_OUTPUT_DIRECTORY: ${FMU_RUNTIME_OUTPUT_DIRECTORY}")
  message(STATUS "   FMU_COMPONENT_DLL_DEPENDENCIES: ${FMU_COMPONENT_DLL_DEPENDENCIES}")

  if(NOT ${FMU_COMPONENT_DLL_DEPENDENCIES} STREQUAL "")
    message(STATUS "   Add command to copy ${FMU} FMU DLL dependencies")
    add_custom_command(
      TARGET COPY_VEHICLE_FMU_DEPENDENCIES PRE_BUILD
      COMMENT "Create directory for ${FMU} FMU DLL dependencies."
      COMMAND ${CMAKE_COMMAND} -E make_directory ${FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
    add_custom_command(
      TARGET COPY_VEHICLE_FMU_DEPENDENCIES PRE_BUILD
      COMMENT "Copy ${FMU} FMU DLL dependencies."
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FMU_COMPONENT_DLL_DEPENDENCIES} ${FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
  endif()
  
  add_dependencies(${FMU_MODEL_IDENTIFIER} COPY_VEHICLE_FMU_DEPENDENCIES)

endforeach()
