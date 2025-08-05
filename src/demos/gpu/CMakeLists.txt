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

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_gpu")

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_gpu_vsg")
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

    if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
