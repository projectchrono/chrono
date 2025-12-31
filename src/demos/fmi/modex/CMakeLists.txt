#--------------------------------------------------------------------------
# Create FMUs
#--------------------------------------------------------------------------

if(${FMU_EXPORT_SUPPORT})

  # Create the Van der Pol FMUs (FMI 2.0 and 3.0)
  add_subdirectory(fmu2_vdp)
  add_subdirectory(fmu3_vdp)

  # Create hydraulic actuator FMU (FMI 2.0)
  add_subdirectory(fmu2_actuator)

endif()

#--------------------------------------------------------------------------
# Create driver for VdP model exchange FMU
#--------------------------------------------------------------------------

set(PROGRAMv demo_FMI_VdP_modex)

# MESSAGE(STATUS "...add ${PROGRAMv}")

add_executable(${PROGRAMv})
source_group("" FILES ${PROGRAMv}.cpp)
target_sources(${PROGRAMv} PRIVATE ${PROGRAMv}.cpp)
target_include_directories(${PROGRAMv} PUBLIC ${FMU_FORGE_DIR})

target_compile_definitions(${PROGRAMv} PUBLIC DEMO_FMU_MAIN_DIR="${DEMO_FMU_MAIN_DIR}")

if (${FMU_EXPORT_SUPPORT})
  target_compile_definitions(${PROGRAMv} PUBLIC VDP_FMU2_FILENAME="${VDP_FMU2_FILENAME}")
  target_compile_definitions(${PROGRAMv} PUBLIC VDP_FMU3_FILENAME="${VDP_FMU3_FILENAME}")
endif()

target_link_libraries(${PROGRAMv} Chrono_core Chrono_fmi)

set_property(TARGET ${PROGRAMv} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAMv}>")

if(MSVC)
  set_target_properties(${PROGRAMv} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(${PROGRAMv} ${VDP_FMU2_MODEL_IDENTIFIER})
  add_dependencies(${PROGRAMv} ${VDP_FMU3_MODEL_IDENTIFIER})
endif()

#--------------------------------------------------------------------------
# Create driver for hydraulic actuato model exchange FMU
#--------------------------------------------------------------------------

set(PROGRAMa demo_FMI_hydraulic_crane_modex)

add_executable(${PROGRAMa})
source_group("" FILES ${PROGRAMa}.cpp)
target_sources(${PROGRAMa} PRIVATE ${PROGRAMa}.cpp)
target_include_directories(${PROGRAMa} PUBLIC ${FMU_FORGE_DIR})

target_compile_definitions(${PROGRAMa} PUBLIC DEMO_FMU_MAIN_DIR="${DEMO_FMU_MAIN_DIR}")

if (${FMU_EXPORT_SUPPORT})
  target_compile_definitions(${PROGRAMa} PUBLIC ACTUATOR_FMU2_FILENAME="${ACTUATOR_FMU2_FILENAME}")
  #target_compile_definitions(${PROGRAMa} PUBLIC ACTUATOR_FMU3_FILENAME="${ACTUATOR_FMU3_FILENAME}")
endif()

target_link_libraries(${PROGRAMa} Chrono_core Chrono_fmi)
if(CH_ENABLE_MODULE_IRRLICHT)
  target_link_libraries(${PROGRAMa} Chrono_irrlicht)
endif()
if(CH_ENABLE_MODULE_VSG)
  target_link_libraries(${PROGRAMa} Chrono_vsg)
endif()

set_property(TARGET ${PROGRAMa} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAMa}>")

if(MSVC)
  set_target_properties(${PROGRAMa} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(${PROGRAMa} ${ACTUATOR_FMU2_MODEL_IDENTIFIER})
endif()
