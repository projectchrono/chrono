#=============================================================================
# CMake configuration file for demos of Chrono::Vehicle co-simulation FMUs.
# These example programs require run-time visualization.
#=============================================================================

if(NOT CH_ENABLE_MODULE_VEHICLE_FMI)
  return()
endif()

#--------------------------------------------------------------

set(DEMOS
  demo_VEH_FMI2_WheeledVehiclePtrain_a
  demo_VEH_FMI2_WheeledVehiclePtrain_b

  demo_VEH_FMI2_WheeledVehicle_a
  demo_VEH_FMI2_WheeledVehicle_b
)

#--------------------------------------------------------------
# Create co-simulation drivers

foreach(PROGRAM ${DEMOS})

  # MESSAGE(STATUS "...add ${PROGRAM}")

  #message("FMU_FORGE_DIR = ${FMU_FORGE_DIR}")
  #message("FMI2_PLATFORM = ${FMI2_PLATFORM}")
  #message("FMI3_PLATFORM = ${FMI3_PLATFORM}")
  #message("CMAKE_SHARED_LIBRARY_SUFFIX = ${CMAKE_SHARED_LIBRARY_SUFFIX}")

  add_executable(${PROGRAM})
  source_group("" FILES ${PROGRAM}.cpp)
  target_sources(${PROGRAM} PRIVATE ${PROGRAM}.cpp)
  target_include_directories(${PROGRAM} PUBLIC ${FMU_FORGE_DIR})

  target_compile_definitions(${PROGRAM} PUBLIC FMI2_PLATFORM="${FMI2_PLATFORM}")
  target_compile_definitions(${PROGRAM} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

  target_link_libraries(${PROGRAM} Chrono_core Chrono_vehicle)

  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

  ###add_dependencies(${PROGRAM} ${CHRONO_VEHICLE_FMU})
endforeach()

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(demo_VEH_FMI2_WheeledVehiclePtrain_a FMU2cs_WheeledVehiclePtrain)
  add_dependencies(demo_VEH_FMI2_WheeledVehiclePtrain_b FMU2cs_WheeledVehiclePtrain FMU2cs_PathFollowerDriver FMU2cs_ForceElementTire)

  add_dependencies(demo_VEH_FMI2_WheeledVehicle_a FMU2cs_WheeledVehicle)
  add_dependencies(demo_VEH_FMI2_WheeledVehicle_b FMU2cs_WheeledVehicle FMU2cs_PathFollowerDriver FMU2cs_ForceElementTire FMU2cs_Powertrain)
endif()
