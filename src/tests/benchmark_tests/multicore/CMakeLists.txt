if(NOT CH_ENABLE_MODULE_MULTICORE)
    return()
endif()

set(TESTS
    btest_MCORE_settling
    )

# ------------------------------------------------------------------------------

SET(LIBS
    Chrono_core
    Chrono_multicore
)

if(CH_ENABLE_MODULE_VSG)
    set(LIBS ${LIBS} Chrono_vsg)
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for MULTICORE module")
build_btests("${TESTS}" "${LIBS}")
