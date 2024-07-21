#--------------------------------------------------------------------------
# Create FMUs
#--------------------------------------------------------------------------

if(${FMU_EXPORT_SUPPORT})

  # Create the Van der Pol FMUs (FMI 2.0 and 3.0)
  add_subdirectory(fmu2_vdp)
  add_subdirectory(fmu3_vdp)

endif()

#--------------------------------------------------------------------------
# Create external FMU driver
#--------------------------------------------------------------------------

set(PROGRAM demo_FMI_VdP_modex)

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM})
source_group("" FILES ${PROGRAM}.cpp)
target_sources(${PROGRAM} PRIVATE ${PROGRAM}.cpp)
target_include_directories(${PROGRAM} PUBLIC ${FMU_TOOLS_DIR})

#target_compile_definitions(${PROGRAM} PUBLIC FMU_OS_SUFFIX="${FMI2_PLATFORM}")
#target_compile_definitions(${PROGRAM} PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

target_compile_definitions(${PROGRAM} PUBLIC DEMO_FMU_MAIN_DIR="${DEMO_FMU_MAIN_DIR}")

if (${FMU_EXPORT_SUPPORT})
  #target_compile_definitions(${PROGRAM} PUBLIC VDP_FMU_MODEL_IDENTIFIER="${VDP_FMU2_MODEL_IDENTIFIER}")
  #target_compile_definitions(${PROGRAM} PUBLIC VDP_FMU_DIRECTORY="${VDP_FMU2_DIRECTORY}")
  target_compile_definitions(${PROGRAM} PUBLIC VDP_FMU_FILENAME="${VDP_FMU2_FILENAME}")
  #target_compile_definitions(${PROGRAM} PUBLIC VDP_FMU_RUNTIME_OUTPUT_DIRECTORY="${VDP_FMU2_RUNTIME_OUTPUT_DIRECTORY}")
endif()

target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_fmi)

set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

if (${FMU_EXPORT_SUPPORT})
  add_dependencies(${PROGRAM} ${VDP_FMU2_MODEL_IDENTIFIER})
endif()
