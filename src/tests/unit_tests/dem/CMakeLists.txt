SET(TESTS
    #utest_DEM_mini
    utest_DEM_frictionrolling
    utest_DEM_meshrolling
    utest_DEM_ballistic
    utest_DEM_stack
    utest_DEM_pyramid
)

SET(LIBS
    Chrono_core
    Chrono_dem
)

MESSAGE(STATUS "Add unit test programs for DEM module")
build_utests(YES "${TESTS}" "${LIBS}")
