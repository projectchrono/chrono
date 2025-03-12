if(NOT CH_ENABLE_MODULE_MULTICORE)
    return()
endif()

set(TESTS
    btest_MCORE_settling
    )

# ------------------------------------------------------------------------------

SET(LIBRARIES
    Chrono_core
    Chrono_multicore
)

if(CH_ENABLE_MODULE_OPENGL)
    set(LIBRARIES ${LIBRARIES} Chrono_opengl)
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for MULTICORE module")

foreach(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} benchmark_main)
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
