set(TESTS
    btest_CH_ChBody
    btest_CH_joints
    btest_CH_pendulums
    btest_CH_mixerNSC
    )

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for PHYSICS module")
build_btests("${TESTS}" "${LIBS}")
