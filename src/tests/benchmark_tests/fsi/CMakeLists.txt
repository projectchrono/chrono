if(NOT ENABLE_MODULE_FSI)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_FSI_RigidBCE_Scaling
    )

# ------------------------------------------------------------------------------

list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_fsisph")

if(ENABLE_MODULE_VSG)
  list(APPEND LIBS "ChronoEngine_fsisph_vsg")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for FSI module")

foreach(PROGRAM ${TESTS})
    #message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS} benchmark_main)
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
