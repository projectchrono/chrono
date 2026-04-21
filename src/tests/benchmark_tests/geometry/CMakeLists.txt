if(NOT BUILD_BENCHMARKING_GEOMETRY)
    return()
endif()
    
set(TESTS
    btest_ChDelaunay2D
    )


list(APPEND LIBS Chrono_core)

message(STATUS "Add benchmark test programs for geometry module")
build_btests("${TESTS}" "${LIBS}")