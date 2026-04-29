set(TESTS
    utest_CH_Delaunay2D
    )

set(LIBS Chrono_core)

build_utests(${BUILD_TESTING_BASE} "${TESTS}" "base" "${LIBS}")