# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

set(DEMOS
    demo_GPU_ballCosim
    demo_GPU_movingBoundary
    demo_GPU_fixedTerrain
    demo_GPU_mixer
    demo_GPU_repose
)

# ------------------------------------------------------------------------------

list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_gpu")

if(ENABLE_MODULE_OPENGL)
    list(APPEND LIBS "ChronoEngine_gpu_opengl")
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

message(STATUS "Add demo programs for GPU module")

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
