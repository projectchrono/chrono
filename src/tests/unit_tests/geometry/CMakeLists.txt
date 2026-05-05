set(TESTS
    utest_CH_Delaunay2D
    )

set(LIBS Chrono_core)

MESSAGE(STATUS "Add unit test programs for GEOMETRY module")
build_utests(${BUILD_TESTING_BASE} "${TESTS}" "base" "${LIBS}")
