#===============================================================================
# CMake configuration file for the ChronoEngine_Distributed library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

# The Chrono Parallel will build even if openmp is not found, it just won't have parallel computation support
# If TBB is defined, thrust will use TBB but the rest of the code will be single threaded

option(ENABLE_MODULE_DISTRIBUTED "Enable the Chrono Distributed module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_DISTRIBUTED)
  mark_as_advanced(FORCE BLAZE_DIR)
  return()
endif()

message(STATUS "==== Chrono Distributed module ====")

mark_as_advanced(CLEAR BLAZE_DIR)

# ------------------------------------------------------------------------------
# Additional compiler flags
# ------------------------------------------------------------------------------

set(CH_DISTRIBUTED_CXX_FLAGS "")
set(CH_DISTRIBUTED_C_FLAGS "")

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigDistr.h.in
  ${PROJECT_BINARY_DIR}/chrono_distributed/ChConfigDistr.h
  )

install(FILES "${PROJECT_BINARY_DIR}/chrono_parallel/ChConfigParallel.h"
        DESTINATION include/chrono_parallel)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the PARALLEL module
# ------------------------------------------------------------------------------

SET(CH_DISTRIBUTED_INCLUDES
    ${BLAZE_DIR}
    ${BOOST_DIR}
    ${THRUST_INCLUDE_DIR}
)

INCLUDE_DIRECTORIES(${CH_DISTRIBUTED_INCLUDES})

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_DISTRIBUTED_CXX_FLAGS "${CH_DISTRIBUTED_CXX_FLAGS}" PARENT_SCOPE)
SET(CH_DISTRIBUTED_C_FLAGS "${CH_DISTRIBUTED_C_FLAGS}" PARENT_SCOPE)
SET(CH_DISTRIBUTED_INCLUDES "${CH_DISTRIBUTED_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono distributed module
# ------------------------------------------------------------------------------

SET(ChronoEngine_Distributed_BASE
   )

SOURCE_GROUP("" FILES ${ChronoEngine_Distributed_BASE})

SET(ChronoEngine_Distributed_PHYSICS
    physics/ChSystemDistr.h
    physics/ChDomainDistr.h
    physics/ChDomainDistrLong.h
    physics/ChSystemDistr.cpp
	physics/ChDomainDistr.cpp
	physics/ChDomainDistrLong.cpp
    )
    
SOURCE_GROUP(physics FILES ${ChronoEngine_Distributed_PHYSICS})

SET(ChronoEngine_Distributed_COLLISION
	#collision/ChBroadphaseDistr.h
	#collision/ChBroadphaseDistrBasic.h
	collision/ChCollisionSystemDistr.h
	collision/ChCollision.h
	#collision/ChNarrowphaseDistr.h
	#collision/ChNarrowphaseDistrBasic.h
	collision/ChCollisionModelDistr.h
	
	#collision/ChBroadphaseDistr.cpp
	#collision/ChBroadphaseDistrBasic.cpp
	collision/ChCollisionSystemDistr.cpp
	#collision/ChNarrowphaseDistr.cpp
	#collision/ChNarrowphaseDistrBasic.cpp
	collision/ChCollisionModelDistr.cpp
	collision/ChAABBGenerator.cpp
    )

SOURCE_GROUP(collision FILES ${ChronoEngine_Distributed_COLLISION})

SET(ChronoEngine_Distributed_COMM
	comm/ChCommDistr.h
	comm/ChCommDistr.cpp
	)

SOURCE_GROUP(comm FILES ${ChronoEngine_Distributed_COMM})

SET(ChronoEngine_Distributed_COSIMULATION
	cosimulation/ChCosimulationDistr.h
	cosimulation/ChCosimulationDistr.cpp
	)

SOURCE_GROUP(cosimulation FILES ${ChronoEngine_Distributed_COSIMULATION})

SET(ChronoEngine_Distributed_SOLVER
	)

SOURCE_GROUP(solver FILES ${ChronoEngine_Distributed_SOLVER})
	
# ------------------------------------------------------------------------------
# Add the ChronoEngine_Distributed library
# ------------------------------------------------------------------------------


ADD_LIBRARY(ChronoEngine_Distributed SHARED
			${ChronoEngine_Distributed_BASE}
			${ChronoEngine_Distributed_PHYSICS}
			${ChronoEngine_Distributed_COLLISION}
			${ChronoEngine_Distributed_COMM}
			${ChronoEngine_Distributed_COSIMULATION}
			${ChronoEngine_Distributed_SOLVER}
			)
			
SET(CHRONO_DISTRIBUTED_LINKED_LIBRARIES ChronoEngine ${MPI_LIBRARIES})

 
SET_TARGET_PROPERTIES(ChronoEngine_Distributed PROPERTIES
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_DISTRIBUTED")

TARGET_LINK_LIBRARIES(ChronoEngine_Distributed ${CHRONO_DISTRIBUTED_LINKED_LIBRARIES})

INSTALL(TARGETS ChronoEngine_Distributed
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/chrono_distributed
    FILES_MATCHING PATTERN "*.h")