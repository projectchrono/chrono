if(NOT ENABLE_MODULE_MULTICORE)
    return()
endif()

set(TESTS
    btest_MCORE_settling
    )

# ------------------------------------------------------------------------------

include_directories(${CH_INCLUDES})
include_directories(${CH_MULTICORE_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
SET(LIBRARIES
    ChronoEngine
    ChronoEngine_multicore
)

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    set(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       set (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
    endif()
endif()

# ------------------------------------------------------------------------------

message(STATUS "Benchmark test programs for MULTICORE module...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} benchmark_main)
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
