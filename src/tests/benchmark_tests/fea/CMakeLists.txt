set(TESTS
    btest_FEA_ANCFshell
    btest_FEA_contact
    btest_FEA_ANCFbeam_3243_LargeDisplacement
    btest_FEA_ANCFbeam_3333_LargeDisplacement
    btest_FEA_ANCFshell_3443_LargeDisplacement
    btest_FEA_ANCFshell_3833_LargeDisplacement
    btest_FEA_ANCFhexa_3843_LargeDisplacement
    )

if(CH_ENABLE_MODULE_PARDISO_MKL OR CH_ENABLE_MODULE_MUMPS)
  set(TESTS ${TESTS}
      btest_FEA_sparse_solver
      btest_FEA_HHT
  )
endif()

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS Chrono_irrlicht)
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS Chrono_pardisomkl)
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS Chrono_mumps)
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for FEA module")
build_btests("${TESTS}" "${LIBS}")
