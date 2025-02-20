# ------------------------------------------------------------------------------
# Co-simulation framework demos
# ------------------------------------------------------------------------------

if(NOT MPI_FOUND)
  return()
endif()

if(NOT CH_ENABLE_MODULE_VEHICLE_COSIM)
  return()
endif()

# ------------------------------------------------------------------------------

set(PROGRAMS 
    demo_VEH_Cosim_WheelRig
    demo_VEH_Cosim_Viper
    demo_VEH_Cosim_Curiosity
)

if(CH_ENABLE_MODULE_VEHICLE_MODELS)

  set(PROGRAMS ${PROGRAMS}
      demo_VEH_Cosim_WheeledVehicle
      demo_VEH_Cosim_TrackedVehicle
  )

  if(CH_ENABLE_MODULE_FSI)
    set(PROGRAMS ${PROGRAMS}
        demo_VEH_Cosim_WheeledVehicle_SPH)
  endif()

  if(CH_ENABLE_MODULE_MULTICORE AND CH_ENABLE_MODULE_OPENGL)
    set(PROGRAMS ${PROGRAMS}
        demo_VEH_Cosim_WheeledVehicle_OMP)
  endif()

endif()

# ------------------------------------------------------------------------------

list(APPEND LIBS "Chrono_vehicle_cosim")
list(APPEND LIBS "ChronoModels_vehicle")

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

##if(CH_ENABLE_MODULE_MUMPS)
##    list(APPEND LIBS "Chrono_mumps")
##endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "Chrono_postprocess")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add demo programs for VEHICLE COSIM module")

foreach(PROGRAM ${PROGRAMS})
  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  source_group("" FILES ${PROGRAM}.cpp)
  target_link_libraries(${PROGRAM} ${LIBS})
  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()
endforeach()

add_subdirectory(external)
