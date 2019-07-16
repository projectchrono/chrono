#===============================================================================
# CMake configuration file for the Chrono_Granular library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================


option(ENABLE_MODULE_GRANULAR "Enable the Chrono Granular module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_GRANULAR)
	return()
endif()

message(STATUS "==== Chrono Granular module ====")

# ------------------------------------------------------------------------------
# Additional compiler flags
# ------------------------------------------------------------------------------

set(CH_GRANULAR_CXX_FLAGS "")
set(CH_GRANULAR_C_FLAGS "")

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----- CUDA support -----
# Return now if CUDA is not available
if(NOT CUDA_FOUND)
  message("Chrono::Granular requires CUDA")
  message(STATUS "Chrono::Granular disabled")
  set(ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono Granular module" FORCE)
  return()
endif()

option(GRANULAR_VERBOSE_PTXAS "Enable verbose output from ptxas during compilation" OFF)
mark_as_advanced(GRANULAR_VERBOSE_PTXAS)

if(GRANULAR_VERBOSE_PTXAS)
	set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --ptxas-options=-v)
endif()

message(STATUS "NVCC Flags are " ${CUDA_NVCC_FLAGS})
set(CUDA_SEPARABLE_COMPILATION OFF)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		set(CUDA_SEPARABLE_COMPILATION ON)
		set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -G -lineinfo)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		set(CUDA_NVCC_FLAGS_DEBUG ${CUDA_NVCC_FLAGS_DEBUG} -G -Xcompiler -g)
		set(CUDA_NVCC_FLAGS_RELEASE ${CUDA_NVCC_FLAGS_RELEASE} -Xcompiler -O3 -Xptxas -O3 -Xcompiler -DNDEBUG)
		set(CUDA_NVCC_FLAGS_RELWITHDEBINFO ${CUDA_NVCC_FLAGS_RELWITHDEBINFO} -Xcompiler -O2 -Xcompiler -g -Xcompiler -DNDEBUG)
		set(CUDA_NVCC_FLAGS_MINSIZEREL ${CUDA_NVCC_FLAGS_MINSIZEREL} -Xcompiler -Os -Xcompiler -DNDEBUG)
		message(STATUS "NVCC Flags are " ${CUDA_NVCC_FLAGS})
		message(STATUS "NVCC Release flags are" ${CUDA_NVCC_FLAGS_RELEASE})
		
		
		if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
			if(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 4.9)
				set(CUDA_PROPAGATE_HOST_FLAGS OFF)
			ELSE()
				message(FATAL_ERROR "Using GCC version:  ${CMAKE_CXX_COMPILER_VERSION}. GCC <= 4.9 is not supported! Please use a newer compiler")
			endif()
		endif()
		set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC --compiler-options -Wall -lineinfo)
		elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		message(FATAL_ERROR "macOS is not supported!")
endif()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -gencode=arch=compute_70,code=sm_70 -gencode=arch=compute_60,code=sm_60 -gencode=arch=compute_50,code=sm_50 -gencode=arch=compute_35,code=sm_35)
message(STATUS "NVCC Flags are " ${CUDA_NVCC_FLAGS})



# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/ChConfigGranular.h.in
	${PROJECT_BINARY_DIR}/chrono_granular/ChConfigGranular.h
	)

install(FILES "${PROJECT_BINARY_DIR}/chrono_granular/ChConfigGranular.h"
				DESTINATION include/chrono_granular)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the GRANULAR module
# ------------------------------------------------------------------------------

set(CH_GRANULAR_INCLUDES ${CUDA_INCLUDE_DIRS})

include_directories(${CH_GRANULAR_INCLUDES})

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

set(CH_GRANULAR_CXX_FLAGS "${CH_GRANULAR_CXX_FLAGS}" PARENT_SCOPE)
set(CH_GRANULAR_C_FLAGS "${CH_GRANULAR_C_FLAGS}" PARENT_SCOPE)
set(CH_GRANULAR_INCLUDES "${CH_GRANULAR_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono-Granular module
# ------------------------------------------------------------------------------

set(ChronoEngine_Granular_BASE
		ChApiGranular.h
		ChGranularDefines.h
		)

source_group("" FILES ${ChronoEngine_Granular_BASE})

set(ChronoEngine_Granular_PHYSICS
		physics/ChGranular.h
		physics/ChGranular.cpp
		physics/ChGranularTriMesh.h
		physics/ChGranularTriMesh.cpp
		physics/ChGranularBoundaryConditions.h
		)

source_group(physics FILES ${ChronoEngine_Granular_PHYSICS})

set(ChronoEngine_Granular_CUDA
		physics/ChGranularGPU_SMC.cu
		physics/ChGranularGPU_SMC.cuh
		physics/ChGranularGPU_SMC_trimesh.cu
		physics/ChGranularGPU_SMC_trimesh.cuh
		physics/ChGranularCollision.cuh
		physics/ChGranularBoundaryConditions.cuh
		physics/ChGranularHelpers.cuh
		physics/ChGranularBoxTriangle.cuh
		utils/ChCudaMathUtils.cuh
		)

source_group(cuda FILES ${ChronoEngine_Granular_CUDA})

set(ChronoEngine_Granular_UTILITIES
		utils/ChGranularUtilities.h
		utils/ChGranularJsonParser.h
		utils/ChGranularSphereDecomp.h
		)

source_group(utilities FILES ${ChronoEngine_Granular_UTILITIES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_granular library
# ------------------------------------------------------------------------------

CUDA_ADD_LIBRARY(ChronoEngine_granular SHARED
						${ChronoEngine_Granular_BASE}
						${ChronoEngine_Granular_PHYSICS}
						${ChronoEngine_Granular_CUDA}
						)
set(CHRONO_GRANULAR_LINKED_LIBRARIES ChronoEngine ${CUDA_FRAMEWORK})

set_target_properties(ChronoEngine_granular PROPERTIES
											LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
											COMPILE_DEFINITIONS "CH_API_COMPILE_GRANULAR")

target_link_libraries(ChronoEngine_granular ${CHRONO_GRANULAR_LINKED_LIBRARIES})

if(HDF5_FOUND)
    set(COMPILE_DEFS "${COMPILE_DEFS} ${H5_BUILT_AS_DYNAMIC_LIB}")
    include_directories(${HDF5_INCLUDE_DIRS})
    target_link_libraries(ChronoEngine_granular ${HDF5_CXX_LIBRARIES})
    add_definitions(-DUSE_HDF5)
endif()

install(TARGETS ChronoEngine_granular
				RUNTIME DESTINATION bin
				LIBRARY DESTINATION lib
				ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
				DESTINATION include/chrono_granular
			FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp")

mark_as_advanced(FORCE
		CUDA_BUILD_CUBIN
		CUDA_BUILD_EMULATION
		CUDA_SEPARABLE_COMPILATION
		CUDA_SDK_ROOT_DIR
		CUDA_VERBOSE_BUILD
		CUDA_HOST_COMPILER)
