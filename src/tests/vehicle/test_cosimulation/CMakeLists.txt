#=============================================================================
# CMake configuration file for the vehicle cosimulation tests.
# These example programs requires MPI and the Chrono::FEA and Chrono::Parallel
# modules
#=============================================================================

IF(NOT MPI_CXX_FOUND OR NOT ENABLE_MODULE_FEA OR NOT ENABLE_MODULE_PARALLEL)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all tests

SET(TEST_PROGRAMS
  test_VEH_HMMWV_Cosimulation
)

#--------------------------------------------------------------
# Additional include directories

include_directories(${CH_PARALLEL_INCLUDES})
include_directories(${MPI_CXX_INCLUDE_PATH})

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
    ChronoEngine_vehicle
    ChronoModels_vehicle
    ${MPI_CXX_LIBRARIES}
)

#--------------------------------------------------------------
# Create the executables

FOREACH(PROGRAM ${TEST_PROGRAMS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                        COMPILE_FLAGS "${CH_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                        LINK_FLAGS "${LINKERFLAG_EXE} ${MPI_CXX_LINK_FLAGS}")
  TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

ENDFOREACH()