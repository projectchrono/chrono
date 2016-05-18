#=============================================================================
# CMake configuration file for the HMMWV cosimulation demo.
# This example program requires MPI and the Chrono::Parallel module
#=============================================================================

IF(NOT MPI_CXX_FOUND OR NOT ENABLE_MODULE_PARALLEL)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO  demo_VEH_HMMWV_Cosimulation)
SOURCE_GROUP("" FILES ${DEMO}.cpp)

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
# Create the executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE} ${MPI_CXX_LINK_FLAGS}")
TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES})
INSTALL(TARGETS ${DEMO} DESTINATION bin)
