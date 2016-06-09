# Various test rigs for testing tire models
# ------------------------------------------------------------------------------

# List all tests that require Chrono::Irrlicht
set(VEH_TESTS_IRRLICHT
    test_VEH_quarterVehicle
    test_VEH_tireRig
)

# List all tests that require Chrono::Parallel, Chrono::FEA, and MPI support
set(VEH_TESTS_PARALLEL
    test_VEH_tireRig_Cosimulation
)

# ------------------------------------------------------------------------------
# Set compiler and linker flags and libraries for the required modules
# (Chrono::Engine and Chrono::Vehicle), as well as optional modules
# (Chrono::FEA, Chrono::MKL, Chrono::OpenGL)

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
set(LIBRARIES ChronoEngine ChronoEngine_vehicle)

if(ENABLE_MODULE_FEA)
    list(APPEND LIBRARIES ChronoEngine_fea)
endif()

if(ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_mkl)
endif()

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    list(APPEND LIBRARIES ChronoEngine_opengl)
endif()

# ------------------------------------------------------------------------------
# Tests that require Chrono::Irrlicht

if(ENABLE_MODULE_IRRLICHT)

    include_directories(${CH_IRRLICHTINC})

    foreach(PROGRAM ${VEH_TESTS_IRRLICHT})
        message(STATUS "...add ${PROGRAM}")
        source_group("" FILES "${PROGRAM}.cpp")
        add_executable(${PROGRAM} ${PROGRAM}.cpp)
        set_target_properties(${PROGRAM} PROPERTIES 
                              COMPILE_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                              LINK_FLAGS "${LINKER_FLAGS}")
        target_link_libraries(${PROGRAM} ${LIBRARIES} ChronoEngine_irrlicht)
    endforeach()
endif()

# ------------------------------------------------------------------------------
# Tests that require Chrono::Parallel and MPI

if(ENABLE_MODULE_PARALLEL AND ENABLE_MODULE_FEA AND MPI_CXX_FOUND)

    include_directories(${CH_PARALLEL_INCLUDES})
    include_directories(${MPI_CXX_INCLUDE_PATH})

    foreach(PROGRAM ${VEH_TESTS_PARALLEL})
        message(STATUS "...add ${PROGRAM}")
        source_group("" FILES "${PROGRAM}.cpp")
        add_executable(${PROGRAM} ${PROGRAM}.cpp)
        set_target_properties(${PROGRAM} PROPERTIES 
                              COMPILE_FLAGS "${COMPILER_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}"
                              LINK_FLAGS "${LINKER_FLAGS} ${MPI_CXX_LINK_FLAGS}")
        target_link_libraries(${PROGRAM} ${LIBRARIES} ChronoEngine_parallel ${MPI_CXX_LIBRARIES})
    endforeach()
endif()




