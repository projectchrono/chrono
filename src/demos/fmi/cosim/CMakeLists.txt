#--------------------------------------------------------------------------
# Create FMUs
#--------------------------------------------------------------------------

if(${FMU_EXPORT_SUPPORT})

  # Create the crane FMUs (FMI 2.0 and 3.0)
  add_subdirectory(fmu2_crane)
  add_subdirectory(fmu3_crane)
   
  # Create the actuator FMUs (FMI 2.0 and 3.0)
  add_subdirectory(fmu2_actuator)
  add_subdirectory(fmu3_actuator)

endif()

#--------------------------------------------------------------------------
# Create co-simulation driver for FMI 2.0
#--------------------------------------------------------------------------

set(PROGRAM2 demo_FMI2_hydraulic_crane_cosim)

# MESSAGE(STATUS "...add ${PROGRAM2}")

add_executable(${PROGRAM2})
if(MSVC)
  set_target_properties(${PROGRAM2} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()
source_group("" FILES ${PROGRAM2}.cpp)
target_sources(${PROGRAM2} PRIVATE ${PROGRAM2}.cpp)
target_include_directories(${PROGRAM2} PUBLIC ${FMU_FORGE_DIR})

target_compile_definitions(${PROGRAM2} PUBLIC FMI2_PLATFORM="${FMI2_PLATFORM}")
target_compile_definitions(${PROGRAM2} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

target_compile_definitions(${PROGRAM2} PUBLIC DEMO_FMU_MAIN_DIR="${DEMO_FMU_MAIN_DIR}")

if (${FMU_EXPORT_SUPPORT})
  target_compile_definitions(${PROGRAM2} PUBLIC CRANE_FMU2_MODEL_IDENTIFIER="${CRANE_FMU2_MODEL_IDENTIFIER}")
  target_compile_definitions(${PROGRAM2} PUBLIC CRANE_FMU2_DIRECTORY="${CRANE_FMU2_DIRECTORY}")
  target_compile_definitions(${PROGRAM2} PUBLIC CRANE_FMU2_FILENAME="${CRANE_FMU2_FILENAME}")
  target_compile_definitions(${PROGRAM2} PUBLIC CRANE_FMU2_RUNTIME_OUTPUT_DIRECTORY="${CRANE_FMU2_RUNTIME_OUTPUT_DIRECTORY}")
  
  target_compile_definitions(${PROGRAM2} PUBLIC ACTUATOR_FMU2_MODEL_IDENTIFIER="${ACTUATOR_FMU2_MODEL_IDENTIFIER}")
  target_compile_definitions(${PROGRAM2} PUBLIC ACTUATOR_FMU2_DIRECTORY="${ACTUATOR_FMU2_DIRECTORY}")
  target_compile_definitions(${PROGRAM2} PUBLIC ACTUATOR_FMU2_FILENAME="${ACTUATOR_FMU2_FILENAME}")
  target_compile_definitions(${PROGRAM2} PUBLIC ACTUATOR_FMU2_RUNTIME_OUTPUT_DIRECTORY="${ACTUATOR_FMU2_RUNTIME_OUTPUT_DIRECTORY}")
endif()

target_link_libraries(${PROGRAM2} Chrono_core)

set_property(TARGET ${PROGRAM2} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM2}>")

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(${PROGRAM2} ${CRANE_FMU2_MODEL_IDENTIFIER} ${ACTUATOR_FMU2_MODEL_IDENTIFIER})
endif()

#--------------------------------------------------------------------------
# Create co-simulation driver for FMI 3.0
#--------------------------------------------------------------------------

set(PROGRAM3 demo_FMI3_hydraulic_crane_cosim)

# MESSAGE(STATUS "...add ${PROGRAM3}")

add_executable(${PROGRAM3})
source_group("" FILES ${PROGRAM3}.cpp)
target_sources(${PROGRAM3} PRIVATE ${PROGRAM3}.cpp)
target_include_directories(${PROGRAM3} PUBLIC ${FMU_FORGE_DIR})

target_compile_definitions(${PROGRAM3} PUBLIC FMI3_PLATFORM="${FMI3_PLATFORM}")
target_compile_definitions(${PROGRAM3} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

target_compile_definitions(${PROGRAM3} PUBLIC DEMO_FMU_MAIN_DIR="${DEMO_FMU_MAIN_DIR}")

if (${FMU_EXPORT_SUPPORT})
  target_compile_definitions(${PROGRAM3} PUBLIC CRANE_FMU3_MODEL_IDENTIFIER="${CRANE_FMU3_MODEL_IDENTIFIER}")
  target_compile_definitions(${PROGRAM3} PUBLIC CRANE_FMU3_DIRECTORY="${CRANE_FMU3_DIRECTORY}")
  target_compile_definitions(${PROGRAM3} PUBLIC CRANE_FMU3_FILENAME="${CRANE_FMU3_FILENAME}")
  target_compile_definitions(${PROGRAM3} PUBLIC CRANE_FMU3_RUNTIME_OUTPUT_DIRECTORY="${CRANE_FMU3_RUNTIME_OUTPUT_DIRECTORY}")
  
  target_compile_definitions(${PROGRAM3} PUBLIC ACTUATOR_FMU3_MODEL_IDENTIFIER="${ACTUATOR_FMU3_MODEL_IDENTIFIER}")
  target_compile_definitions(${PROGRAM3} PUBLIC ACTUATOR_FMU3_DIRECTORY="${ACTUATOR_FMU3_DIRECTORY}")
  target_compile_definitions(${PROGRAM3} PUBLIC ACTUATOR_FMU3_FILENAME="${ACTUATOR_FMU3_FILENAME}")
  target_compile_definitions(${PROGRAM3} PUBLIC ACTUATOR_FMU3_RUNTIME_OUTPUT_DIRECTORY="${ACTUATOR_FMU3_RUNTIME_OUTPUT_DIRECTORY}")
endif()

target_link_libraries(${PROGRAM3} Chrono_core)

set_property(TARGET ${PROGRAM3} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM3}>")

if(MSVC)
  set_target_properties(${PROGRAM3} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(${PROGRAM3} ${CRANE_FMU3_MODEL_IDENTIFIER} ${ACTUATOR_FMU3_MODEL_IDENTIFIER})
endif()
