SET(TESTS
    utest_CH_archive
    utest_CH_ChFunctions
    utest_CH_ChVector
    utest_CH_ChQuaternion
    utest_CH_ChState
    utest_CH_coords
    utest_CH_linalg
    utest_CH_math
    utest_CH_sparsematrix
    utest_CH_ISO2631
)

SET(LIBS Chrono_core)

MESSAGE(STATUS "Add unit test programs for CORE module")
build_utests(YES "${TESTS}" "${LIBS}")
