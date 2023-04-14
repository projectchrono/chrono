set(TESTS
    btest_FEA_ANCFshell
    btest_FEA_contact
	btest_FEA_ANCFbeam_3243_LargeDisplacement
	btest_FEA_ANCFbeam_3333_LargeDisplacement
	btest_FEA_ANCFshell_3443_LargeDisplacement
	btest_FEA_ANCFshell_3833_LargeDisplacement
	btest_FEA_ANCFhexa_3843_LargeDisplacement
    )

set(TESTS_MKL_MUMPS_PARPROJ
   btest_FEA_sparse_solver
   )

# ------------------------------------------------------------------------------

include_directories(${CH_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHT_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  include_directories(${CH_MKL_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_MUMPS)
  include_directories(${CH_MUMPS_INCLUDES})
  list(APPEND LIBS "ChronoEngine_mumps")
endif()

if(ENABLE_MODULE_PARDISO_PROJECT)
  include_directories(${CH_PARDISOPROJECT_INCLUDES})
  list(APPEND LIBS "ChronoEngine_pardisoproject")
  list(APPEND LIBS ${PARDISOPROJECT_LIBRARIES})
endif()

if(ENABLE_MODULE_PARDISO_MKL OR ENABLE_MODULE_MUMPS OR ENABLE_MODULE_PARDISO_PROJECT)
  set(TESTS ${TESTS} ${TESTS_MKL_MUMPS_PARPROJ})
endif()

# ------------------------------------------------------------------------------

message(STATUS "Benchmark test programs for FEA module...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS} benchmark_main)
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
