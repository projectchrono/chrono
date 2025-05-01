#=============================================================================
# CMake configuration file for demos of Chrono::Vehicle model exchange FMUs.
# These example programs require run-time visualization.
#=============================================================================

if(NOT CH_ENABLE_MODULE_VEHICLE_FMI)
  return()
endif()

#--------------------------------------------------------------

set(DEMOS
  demo_VEH_FMI2_PathFollower
)

#--------------------------------------------------------------

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")
list(APPEND LIBS "Chrono_fmi")

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_vehicle_irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "Chrono_postprocess")
endif()

#--------------------------------------------------------------
# Create model exchange drivers

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM})
  source_group("" FILES ${PROGRAM}.cpp)
  target_sources(${PROGRAM} PRIVATE ${PROGRAM}.cpp)
  target_include_directories(${PROGRAM} PUBLIC ${FMU_FORGE_DIR})

  target_compile_definitions(${PROGRAM} PUBLIC FMI2_PLATFORM="${FMI2_PLATFORM}")
  target_compile_definitions(${PROGRAM} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

  target_link_libraries(${PROGRAM} ${LIBS})

  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

  ###add_dependencies(${PROGRAM} ${CHRONO_VEHICLE_FMU})
endforeach()

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(demo_VEH_FMI2_PathFollower FMU2me_PathFollowerDriver)
endif()
