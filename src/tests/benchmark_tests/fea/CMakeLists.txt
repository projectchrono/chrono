set(TESTS
    btest_FEA_ANCFshell
    btest_FEA_contact
	btest_FEA_ANCFbeam_3243_LargeDisplacement
	btest_FEA_ANCFbeam_3333_LargeDisplacement
	btest_FEA_ANCFshell_3443_LargeDisplacement
	btest_FEA_ANCFshell_3833_LargeDisplacement
	btest_FEA_ANCFhexa_3843_LargeDisplacement
    )

set(TESTS_MKL_MUMPS
   btest_FEA_sparse_solver
   )

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS "Chrono_mumps")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL OR CH_ENABLE_MODULE_MUMPS)
  set(TESTS ${TESTS} ${TESTS_MKL_MUMPS})
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for FEA module")

foreach(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS} benchmark_main)
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
