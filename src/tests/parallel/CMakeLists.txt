#--------------------------------------------------------------
set(PARALLEL_TESTS
    test_PAR_settling
)

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
set(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel)

include_directories(${CH_PARALLEL_INCLUDES})

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    list(APPEND LIBRARIES ChronoEngine_opengl)
endif()

message(STATUS "Additional test programs for PARALLEL module...")

#--------------------------------------------------------------

foreach(PROGRAM ${PARALLEL_TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )

    target_link_libraries(${PROGRAM} ${LIBRARIES})

endforeach()
