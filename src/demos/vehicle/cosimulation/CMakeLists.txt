# ------------------------------------------------------------------------------
# Co-simulation framework demos
# ------------------------------------------------------------------------------

if(NOT MPI_FOUND)
  return()
endif()

if(NOT ENABLE_MODULE_VEHICLE_COSIM)
  return()
endif()

# ------------------------------------------------------------------------------

set(PROGRAMS 
    demo_VEH_Cosim_WheelRig
    demo_VEH_Cosim_Viper
    demo_VEH_Cosim_Curiosity
)

if(ENABLE_MODULE_VEHICLE_MODELS)

  set(PROGRAMS ${PROGRAMS}
      demo_VEH_Cosim_WheeledVehicle
      demo_VEH_Cosim_TrackedVehicle
  )

  if(ENABLE_MODULE_FSI)
    set(PROGRAMS ${PROGRAMS}
        demo_VEH_Cosim_WheeledVehicle_SPH)
  endif()

  if(ENABLE_MODULE_MULTICORE AND ENABLE_MODULE_OPENGL)
    set(PROGRAMS ${PROGRAMS}
        demo_VEH_Cosim_WheeledVehicle_OMP)
  endif()

endif()

# ------------------------------------------------------------------------------

message(STATUS "Add demo programs for VEHICLE COSIM module")

foreach(PROGRAM ${PROGRAMS})
  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  source_group("" FILES ${PROGRAM}.cpp)
  target_link_libraries(${PROGRAM} ChronoModels_vehicle)
  target_link_libraries(${PROGRAM} ChronoEngine_vehicle_cosim)
endforeach()

add_subdirectory(external)
