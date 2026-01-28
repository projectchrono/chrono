if(NOT CH_ENABLE_MODULE_FSI_TDPF)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_FSI-TDPF_Solver
    )

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsitdpf)

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS Chrono_pardisomkl)
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS Chrono_mumps)
endif()

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS Chrono_fsitdpf_vsg)
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for FSI-TDPF module")
build_btests("${TESTS}" "${LIBS}")

foreach(PROGRAM ${TESTS})
    target_include_directories(${PROGRAM} PRIVATE "${CH_HYDROCHRONO_DIR}/include")
endforeach()
