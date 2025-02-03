if(NOT ENABLE_MODULE_MULTICORE)
    return()
endif()

set(TESTS
    btest_MCORE_settling
    )

# ------------------------------------------------------------------------------

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_multicore
)

if(ENABLE_MODULE_OPENGL)
    set(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

#    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#       set (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
#    endif()
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for MULTICORE module")

foreach(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} benchmark_main)
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
