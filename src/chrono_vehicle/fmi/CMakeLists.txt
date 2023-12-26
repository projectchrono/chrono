#=============================================================================
# CMake configuration file for Chrono Vehicle FMI support
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

# List of FMU subdirectories
set(FMUS 
    wheeled_vehicle
    wheeled_vehicle_complete)

# Create custom target with commands for copying FMU dependencies
add_custom_target(COPY_VEHICLE_FMU_DEPENDENCIES)
add_dependencies(COPY_VEHICLE_FMU_DEPENDENCIES ChronoEngine_vehicle)

message(STATUS "Add Chrono::Vehicle FMUs...")

# Create each FMU and add custom command to copy its necessary dependencies 
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
      TARGET COPY_VEHICLE_FMU_DEPENDENCIES POST_BUILD
      COMMENT "Copy ${FMU} FMU DLL dependencies"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FMU_COMPONENT_DLL_DEPENDENCIES} ${FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
  endif()
  
  add_dependencies(${FMU_MODEL_IDENTIFIER} COPY_VEHICLE_FMU_DEPENDENCIES)

endforeach()
