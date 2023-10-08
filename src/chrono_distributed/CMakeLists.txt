#===============================================================================
# CMake configuration file for the ChronoEngine_distributed library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

option(ENABLE_MODULE_DISTRIBUTED "Enable the Chrono Distributed module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_DISTRIBUTED)
  return()
endif()

message(STATUS "\n==== Chrono Distributed module ====\n")

# Return now if MPI is not available
if(NOT MPI_FOUND)
    message(WARNING "Chrono::Distributed requires MPI, but MPI not found; disabling Chrono::Distributed")
	set(ENABLE_MODULE_DISTRIBUTED OFF CACHE BOOL "Enable the Chrono Distributed module" FORCE)
	return()
endif()

# Return now if Chrono::Multicore is not enabled
if(NOT ENABLE_MODULE_MULTICORE)
    message(WARNING "Chrono::Distributed depends on Chrono::Multicore which is disabled; disabling Chrono::Distributed")
    set(ENABLE_MODULE_DISTRIBUTED OFF CACHE BOOL "Enable the Chrono Distributed module" FORCE)
    return()
endif()



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

install(FILES "${PROJECT_BINARY_DIR}/chrono_distributed/ChConfigDistr.h"
        DESTINATION include/chrono_distributed)

# ------------------------------------------------------------------------------
# Compiler and linker flags specific to this module.
# Additional include directories for this module.
# ------------------------------------------------------------------------------

set(CH_DISTRIBUTED_CXX_FLAGS "${MPI_CXX_COMPILE_FLAGS}")
set(CH_DISTRIBUTED_C_FLAGS "")

set(CH_DISTRIBUTED_LINK_FLAGS "${MPI_CXX_LINK_FLAGS}")

set(CH_DISTRIBUTED_INCLUDES "${MPI_CXX_INCLUDE_PATH}")

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

set(CH_DISTRIBUTED_CXX_FLAGS "${CH_DISTRIBUTED_CXX_FLAGS}" PARENT_SCOPE)
set(CH_DISTRIBUTED_C_FLAGS "${CH_DISTRIBUTED_C_FLAGS}" PARENT_SCOPE)
set(CH_DISTRIBUTED_LINK_FLAGS "${CH_DISTRIBUTED_LINK_FLAGS}" PARENT_SCOPE)
set(CH_DISTRIBUTED_INCLUDES "${CH_DISTRIBUTED_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono distributed module
# ------------------------------------------------------------------------------

SET(ChronoEngine_distributed_BASE
	ChApiDistributed.h
	ChDistributedDataManager.h
	ChDistributedDataManager.cpp
	ChTypesDistributed.h
    )

SOURCE_GROUP("" FILES ${ChronoEngine_distributed_BASE})

SET(ChronoEngine_distributed_PHYSICS
    physics/ChSystemDistributed.h
    physics/ChDomainDistributed.h
    physics/ChSystemDistributed.cpp
	physics/ChDomainDistributed.cpp
    )

SOURCE_GROUP(physics FILES ${ChronoEngine_distributed_PHYSICS})

SET(ChronoEngine_distributed_COLLISION
	collision/ChCollisionModelDistributed.h
	collision/ChCollisionSystemDistributed.h
	collision/ChCollisionModelDistributed.cpp
	collision/ChCollisionSystemDistributed.cpp
	collision/ChBoundary.h
	collision/ChBoundary.cpp
    )

SOURCE_GROUP(collision FILES ${ChronoEngine_distributed_COLLISION})

SET(ChronoEngine_distributed_COMM
	comm/ChCommDistributed.h
	comm/ChCommDistributed.cpp
	)

SOURCE_GROUP(comm FILES ${ChronoEngine_distributed_COMM})

SET(ChronoEngine_distributed_SOLVER
	)

SOURCE_GROUP(solver FILES ${ChronoEngine_distributed_SOLVER})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_distributed library
# ------------------------------------------------------------------------------

include_directories(${CH_MULTICORE_INCLUDES})
include_directories(${CH_DISTRIBUTED_INCLUDES})

ADD_LIBRARY(ChronoEngine_distributed
			${ChronoEngine_distributed_BASE}
			${ChronoEngine_distributed_PHYSICS}
			${ChronoEngine_distributed_COLLISION}
			${ChronoEngine_distributed_COMM}
			${ChronoEngine_distributed_COSIMULATION}
			${ChronoEngine_distributed_SOLVER}
			)

SET_TARGET_PROPERTIES(ChronoEngine_distributed PROPERTIES
	                  COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS} ${CH_DISTRIBUTED_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB} ${CH_DISTRIBUTED_LINK_FLAGS}")

target_compile_definitions(ChronoEngine_distributed PRIVATE "CH_API_COMPILE_DISTRIBUTED")
target_compile_definitions(ChronoEngine_distributed PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoEngine_distributed
                      ChronoEngine
                      ChronoEngine_multicore
                      ${MPI_CXX_LIBRARIES})

INSTALL(TARGETS ChronoEngine_distributed
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_distributed
        FILES_MATCHING PATTERN "*.h")
