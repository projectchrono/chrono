#--------------------------------------------------------------------------
# Create FMUs
#--------------------------------------------------------------------------

if(${FMU_EXPORT_SUPPORT})

  # Create the crane FMU
  add_subdirectory(fmu_crane)
  #message("   CRANE_FMU_MODEL_IDENTIFIER: ${CRANE_FMU_MODEL_IDENTIFIER}")
  #message("   CRANE_FMU_DIRECTORY: ${CRANE_FMU_DIRECTORY}")
  #message("   CRANE_FMU_FILENAME: ${CRANE_FMU_FILENAME}")
  #message("   CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY: ${CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY}")
   
  # Create the actuator FMU
  add_subdirectory(fmu_actuator)
  #message("   ACTUATOR_FMU_MODEL_IDENTIFIER: ${ACTUATOR_FMU_MODEL_IDENTIFIER}")
  #message("   ACTUATOR_FMU_DIRECTORY: ${ACTUATOR_FMU_DIRECTORY}")
  #message("   ACTUATOR_FMU_FILENAME: ${ACTUATOR_FMU_FILENAME}")
  #message("   ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY: ${ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY}")

endif()

#--------------------------------------------------------------------------
# Create co-simulation driver
#--------------------------------------------------------------------------

set(PROGRAM demo_FMI2_hydraulic_crane_cosim)

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM})
source_group("" FILES ${PROGRAM}.cpp)
target_sources(${PROGRAM} PRIVATE ${PROGRAM}.cpp)
target_include_directories(${PROGRAM} PUBLIC ${FMU_TOOLS_DIR})

target_compile_definitions(${PROGRAM} PUBLIC FMI2_PLATFORM="${FMI2_PLATFORM}")
target_compile_definitions(${PROGRAM} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

target_compile_definitions(${PROGRAM} PUBLIC DEMO_FMU_MAIN_DIR="${DEMO_FMU_MAIN_DIR}")

if (${FMU_EXPORT_SUPPORT})
  target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_MODEL_IDENTIFIER="${CRANE_FMU_MODEL_IDENTIFIER}")
  target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_DIRECTORY="${CRANE_FMU_DIRECTORY}")
  target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_FILENAME="${CRANE_FMU_FILENAME}")
  target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY="${CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY}")
  
  target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_MODEL_IDENTIFIER="${ACTUATOR_FMU_MODEL_IDENTIFIER}")
  target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_DIRECTORY="${ACTUATOR_FMU_DIRECTORY}")
  target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_FILENAME="${ACTUATOR_FMU_FILENAME}")
  target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY="${ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY}")
endif()

target_link_libraries(${PROGRAM} ChronoEngine)

set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(${PROGRAM} ${CRANE_FMU_MODEL_IDENTIFIER} ${ACTUATOR_FMU_MODEL_IDENTIFIER})
endif()
