if(NOT CH_ENABLE_MODULE_FSI)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_FSI_RigidBCE_Scaling
    )

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_fsisph")

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS "Chrono_fsisph_vsg")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for FSI module")

foreach(PROGRAM ${TESTS})
    #message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS} benchmark_main)
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
