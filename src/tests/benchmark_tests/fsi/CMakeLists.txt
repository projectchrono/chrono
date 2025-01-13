if(NOT ENABLE_MODULE_FSI)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_FSI_RigidBCE_Scaling
    )

# ------------------------------------------------------------------------------

include_directories(${CH_FSI_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_fsi_sph")

if(ENABLE_MODULE_VSG)
  include_directories(${CH_VSG_INCLUDES})
  list(APPEND LIBS "ChronoEngine_vsg")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Benchmark test programs for FSI module...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS} benchmark_main)
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
