set(TESTS
    utest_JOINT_revolute
    utest_JOINT_spherical
    utest_JOINT_universal
    utest_JOINT_prismatic
    utest_JOINT_cylindrical
    utest_JOINT_distance
    utest_JOINT_distance_unilateral
    utest_JOINT_rackpinion
    utest_JOINT_revsph
    utest_JOINT_linactuator
    utest_JOINT_transpring
    utest_JOINT_tsda
    utest_JOINT_rotspring
)


set(LIBS Chrono_core)

message(STATUS "Add unit test programs for Chrono joints")
build_utests(YES "${TESTS}" "${LIBS}")
