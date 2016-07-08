# Co-simulation tire test rig
# ------------------------------------------------------------------------------

# Required: Chrono::Parallel, Chrono::FEA, MPI support
# Optional: Chrono::MKL, Chrono::OpenGL

if(NOT ENABLE_MODULE_PARALLEL OR NOT ENABLE_MODULE_FEA OR NOT MPI_CXX_FOUND)
  return()
endif()

# ------------------------------------------------------------------------------
# Set compiler and linker flags and libraries for the required modules
# (Chrono::Engine and Chrono::Vehicle), as well as optional modules
# (Chrono::FEA, Chrono::MKL, Chrono::OpenGL)

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE} ${MPI_CXX_LINK_FLAGS}")
set(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoEngine_fea
    ChronoEngine_parallel
    ${MPI_CXX_LIBRARIES})

include_directories(${CH_PARALLEL_INCLUDES})
include_directories(${MPI_CXX_INCLUDE_PATH})

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
# Tests that require Chrono::Parallel and MPI

set(PROGRAM test_VEH_tireRig_Cosimulation)

set(TEST_FILES 
    test_VEH_tireRig_Cosimulation.cpp
    BaseNode.h
    BaseNode.cpp
    RigNode.h
    RigNode.cpp
    TerrainNode.h
    TerrainNode.cpp)

source_group("" FILES ${TEST_FILES})

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM} ${TEST_FILES})
set_target_properties(${PROGRAM} PROPERTIES 
                     COMPILE_FLAGS "${COMPILER_FLAGS}"
                     LINK_FLAGS "${LINKER_FLAGS}")
target_link_libraries(${PROGRAM} ${LIBRARIES})
