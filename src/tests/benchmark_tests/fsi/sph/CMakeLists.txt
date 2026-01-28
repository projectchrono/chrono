if(NOT CH_ENABLE_MODULE_FSI_SPH)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_FSI-SPH_RigidBCE_Scaling
    btest_FSI-SPH_Initialization
    )

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsisph)

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS Chrono_fsisph_vsg)
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for FSI-SPH module")
build_btests("${TESTS}" "${LIBS}")
