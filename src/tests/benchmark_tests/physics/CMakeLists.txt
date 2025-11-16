set(TESTS
    btest_CH_ChBody
    btest_CH_joints
    btest_CH_pendulums
    btest_CH_mixerNSC
    )

if(CH_ENABLE_MODULE_PARDISO_MKL OR CH_ENABLE_MODULE_MUMPS)
  set(TESTS ${TESTS}
      btest_CH_HHT
      )
endif()

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS Chrono_mumps)
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for PHYSICS module")
build_btests("${TESTS}" "${LIBS}")
