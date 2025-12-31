# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

set(DEMOS
    demo_DEM_ballCosim
    demo_DEM_movingBoundary
    demo_DEM_fixedTerrain
    demo_DEM_mixer
    demo_DEM_repose
)

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_dem")

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_dem_vsg")
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

message(STATUS "Add demo programs for DEM module")

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
