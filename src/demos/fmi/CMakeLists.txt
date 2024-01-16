#--------------------------------------------------------------------------
# Create FMUs
#--------------------------------------------------------------------------

add_custom_target(COPY_DEMO_FMU_DEPENDENCIES)
add_dependencies(COPY_DEMO_FMU_DEPENDENCIES ChronoEngine)
MESSAGE(STATUS "Demo programs for FMI module...")

# Create the crane FMU
add_subdirectory(fmu_crane)
#message("   CRANE_FMU_MODEL_IDENTIFIER: ${CRANE_FMU_MODEL_IDENTIFIER}")
#message("   CRANE_FMU_DIRECTORY: ${CRANE_FMU_DIRECTORY}")
#message("   CRANE_FMU_FILENAME: ${CRANE_FMU_FILENAME}")
#message("   CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY: ${CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY}")
#message("   CRANE_FMU_COMPONENT_DLL_DEPENDENCIES: ${CRANE_FMU_COMPONENT_DLL_DEPENDENCIES}")

if(NOT ${CRANE_FMU_COMPONENT_DLL_DEPENDENCIES} STREQUAL "")
    #message("   Add command to copy crane FMU DLL dependencies")
    add_custom_command(
      TARGET COPY_DEMO_FMU_DEPENDENCIES PRE_BUILD
      COMMENT "Create directory for crane FMU DLL dependencies."
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
    add_custom_command(
      TARGET COPY_DEMO_FMU_DEPENDENCIES PRE_BUILD
      COMMENT "Copy crane FMU DLL dependencies."
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CRANE_FMU_COMPONENT_DLL_DEPENDENCIES} ${CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
endif()
add_dependencies(${FMU_MODEL_IDENTIFIER} COPY_DEMO_FMU_DEPENDENCIES)

# Create the actuator FMU
add_subdirectory(fmu_actuator)
#message("   ACTUATOR_FMU_MODEL_IDENTIFIER: ${ACTUATOR_FMU_MODEL_IDENTIFIER}")
#message("   ACTUATOR_FMU_DIRECTORY: ${ACTUATOR_FMU_DIRECTORY}")
#message("   ACTUATOR_FMU_FILENAME: ${ACTUATOR_FMU_FILENAME}")
#message("   ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY: ${ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY}")
#message("   ACTUATOR_FMU_COMPONENT_DLL_DEPENDENCIES: ${ACTUATOR_FMU_COMPONENT_DLL_DEPENDENCIES}")

if(NOT ${ACTUATOR_FMU_COMPONENT_DLL_DEPENDENCIES} STREQUAL "")
    #message("   Add command to copy actuator FMU DLL dependencies")
    add_custom_command(
      TARGET COPY_DEMO_FMU_DEPENDENCIES PRE_BUILD
      COMMENT "Create directory for actuator FMU DLL dependencies."
      COMMAND ${CMAKE_COMMAND} -E make_directory ${ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
    add_custom_command(
      TARGET COPY_DEMO_FMU_DEPENDENCIES PRE_BUILD
      COMMENT "Copy actuator FMU DLL dependencies."
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ACTUATOR_FMU_COMPONENT_DLL_DEPENDENCIES} ${ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY}
    )
endif()
add_dependencies(${FMU_MODEL_IDENTIFIER} COPY_DEMO_FMU_DEPENDENCIES)

#--------------------------------------------------------------------------
# Create co-simulation driver
#--------------------------------------------------------------------------

set(PROGRAM demo_FMI_hydraulic_crane_cosim)

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM})
source_group("" FILES ${PROGRAM}.cpp)
target_sources(${PROGRAM} PRIVATE ${PROGRAM}.cpp)
target_include_directories(${PROGRAM} PUBLIC ${FMU_TOOLS_DIR})

target_compile_definitions(${PROGRAM} PUBLIC FMU_OS_SUFFIX="${FMI_PLATFORM}")
target_compile_definitions(${PROGRAM} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_MODEL_IDENTIFIER="${CRANE_FMU_MODEL_IDENTIFIER}")
target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_DIRECTORY="${CRANE_FMU_DIRECTORY}")
target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_FILENAME="${CRANE_FMU_FILENAME}")
target_compile_definitions(${PROGRAM} PUBLIC CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY="${CRANE_FMU_RUNTIME_OUTPUT_DIRECTORY}")

target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_MODEL_IDENTIFIER="${ACTUATOR_FMU_MODEL_IDENTIFIER}")
target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_DIRECTORY="${ACTUATOR_FMU_DIRECTORY}")
target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_FILENAME="${ACTUATOR_FMU_FILENAME}")
target_compile_definitions(${PROGRAM} PUBLIC ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY="${ACTUATOR_FMU_RUNTIME_OUTPUT_DIRECTORY}")

target_link_libraries(${PROGRAM} ChronoEngine)

set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

add_dependencies(${PROGRAM} ${CRANE_FMU_MODEL_IDENTIFIER} ${ACTUATOR_FMU_MODEL_IDENTIFIER})
