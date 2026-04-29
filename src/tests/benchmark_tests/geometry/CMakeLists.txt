set(TESTS
    btest_CH_Delaunay2D
    )

list(APPEND LIBS Chrono_core)

message(STATUS "Add benchmark test programs for geometry module")
build_btests("${TESTS}" "${LIBS}")