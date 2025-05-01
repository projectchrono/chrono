if(NOT CH_ENABLE_MODULE_VEHICLE OR NOT CH_ENABLE_MODULE_VEHICLE_MODELS)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_VEH_hmmwvDLC
    btest_VEH_hmmwvSCM
    btest_VEH_m113Acc
    )

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
  list(APPEND LIBS "Chrono_vehicle_irrlicht")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for VEHICLE module")

foreach(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS} benchmark_main)
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
