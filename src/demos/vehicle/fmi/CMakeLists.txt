#=============================================================================
# CMake configuration file for demos of Chrono::Vehicle FMUs.
# These example programs require run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_VEHICLE_FMI)
  return()
endif()

#--------------------------------------------------------------

set(DEMOS
demo_VEH_FMI_WheeledVehicle_a
demo_VEH_FMI_WheeledVehicle_b
)

#--------------------------------------------------------------
# Create co-simulation drivers

foreach(PROGRAM ${DEMOS})

  message(STATUS "...add ${PROGRAM}")

  #message("FMU_TOOLS_DIR = ${FMU_TOOLS_DIR}")
  #message("FMI_PLATFORM = ${FMI_PLATFORM}")
  #message("CMAKE_SHARED_LIBRARY_SUFFIX = ${CMAKE_SHARED_LIBRARY_SUFFIX}")

  add_executable(${PROGRAM})
  source_group("" FILES ${PROGRAM}.cpp)
  target_sources(${PROGRAM} PRIVATE ${PROGRAM}.cpp)
  target_include_directories(${PROGRAM} PUBLIC ${FMU_TOOLS_DIR})

  target_compile_definitions(${PROGRAM} PUBLIC FMU_OS_SUFFIX="${FMI_PLATFORM}")
  target_compile_definitions(${PROGRAM} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

  target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_vehicle)

  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

  ###add_dependencies(${PROGRAM} ${CHRONO_VEHICLE_FMU})
endforeach()