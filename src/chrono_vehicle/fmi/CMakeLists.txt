#=============================================================================
# CMake configuration file for Chrono Vehicle FMI support
#=============================================================================

# Set common output directory for all Chrono::Vehicle FMUs
set(VEHICLE_FMU_MAIN_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vehicleFMU")

# Generate configuration header ChConfigVehicleFMI.h
set(CHRONO_VEHICLE_FMU_DIR "#define CHRONO_VEHICLE_FMU_DIR \"${VEHICLE_FMU_MAIN_DIR}/\"")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigVehicleFMI.h.in
               ${PROJECT_BINARY_DIR}/chrono_vehicle/ChConfigVehicleFMI.h)  

# Create all FMUs

if(FMU_EXPORT_SUPPORT)

  message(STATUS "Add Chrono::Vehicle co-simulation FMUs...")
  add_subdirectory(cosim)
  
  message(STATUS "Add Chrono::Vehicle model exchange FMUs...")
  add_subdirectory(modex)

endif()
