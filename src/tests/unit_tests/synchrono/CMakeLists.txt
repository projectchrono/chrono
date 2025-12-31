set(TESTS
    utest_SYN_MPI
    utest_SYN_agent_initialization
)

set(LIBS Chrono_core Chrono_synchrono ${SYNCHRONO_LIBRARIES})

message(STATUS "Add unit test programs for SYNCHRONO module")
build_utests(NO "${TESTS}" "${LIBS}" "${UTILS}")
