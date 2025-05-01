#--------------------------------------------------------------
# Files for comparing SynChrono scaling performance to baseline Chrono::Vehicle performance
#
# Requires the SynChrono and Chrono::Vehicle modules 
#--------------------------------------------------------------

if(NOT CH_ENABLE_MODULE_VEHICLE OR NOT CH_ENABLE_MODULE_SYNCHRONO)
  return()
endif()  

# ------------------------------------------------------------------------------

set(TESTS
    btest_SCM_SYNscaling
    btest_SCM_VEHscaling
    )

# ------------------------------------------------------------------------------

set(LIBRARIES
    Chrono_core
    Chrono_vehicle
    Chrono_synchrono
    ChronoModels_vehicle
    )

if (CH_ENABLE_MODULE_IRRLICHT)
    set(LIBRARIES ${LIBRARIES} Chrono_irrlicht Chrono_vehicle_irrlicht)
endif()

include_directories(${SYN_INCLUDES})

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for SCM scaling")

foreach(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES})
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
