cmake_minimum_required(VERSION 2.8)
project(ChronoEngine_Parallel)

#=============================================================================
# CHRONO::ENGINE   CMake configuration file for GPU unit
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================
find_package("OpenMP" REQUIRED)

# Add the OpenMP-specific compiler and linker flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")



SET(INTEL_OPT "SSE2" CACHE STRING "intel optimizations")
SET_PROPERTY(CACHE INTEL_OPT PROPERTY STRINGS "SSE2" "SSE3" "SSSE3" "SSE4.1" "SSE4.2" "AVX" "CORE-AVX-I" "CORE-AVX2")

if (${CMAKE_C_COMPILER} MATCHES "icc.*$") 
  	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -x${INTEL_OPT}")
	set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -x${INTEL_OPT}")
endif ()






#add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP)
#add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP)

#
# LIST THE FILES THAT MAKE THE GPU LIBRARY
# NOTE: to add a new source to this unit, just add its name here and re-run the CMake.
#

SET(ChronoEngine_unit_GPU_COLLISION
    src/collision/ChCAABBGenerator.h
    src/collision/ChCBroadphase.h
    src/collision/ChCNarrowphase.h
    src/collision/ChCAABBGenerator.cpp
    src/collision/ChCBroadphase.cpp
    src/collision/ChCNarrowphase.cpp

    src/collision/ChCSupportFunctions.h
    src/collision/ChCCollisionModelParallel.h
    src/collision/ChCCollisionModelParallel.cpp
    src/collision/ChCCollisionSystemParallel.h
    src/collision/ChCCollisionSystemParallel.cpp
    src/collision/ChCCollisionSystemBulletParallel.h
    src/collision/ChCCollisionSystemBulletParallel.cpp
    src/collision/ChContactContainerParallel.h
    src/collision/ChContactContainerParallel.cpp
   )
SET(ChronoEngine_unit_GPU_SOLVER
    src/solver/ChSolverParallel.h
    src/solver/ChSolverParallel.cpp
    src/solver/ChSolverBlockJacobi.cpp
    src/solver/ChSolverCG.cpp
    src/solver/ChSolverCGS.cpp
    src/solver/ChSolverGD.cpp
    src/solver/ChSolverSD.cpp
    src/solver/ChSolverAPGD.cpp
    src/solver/ChSolverMinRes.cpp
    src/solver/ChSolverBiCG.cpp
    src/solver/ChSolverBiCGStab.cpp
   )

SET(ChronoEngine_unit_GPU_CONSTRAINTS
    src/constraints/ChConstraintRigidRigid.cpp
    src/constraints/ChConstraintRigidRigid.h
    src/constraints/ChConstraintBilateral.cpp
    src/constraints/ChConstraintBilateral.h
   )

SET(ChronoEngine_unit_GPU_SOURCES
    src/ChSystemParallel.cpp
   )

SET(ChronoEngine_unit_GPU_HEADERS
    src/ChApiParallel.h
    src/ChBaseParallel.h
    src/ChParallelDefines.h
    src/ChParallelMath.h
    src/ChThrustLinearAlgebra.h
    src/ChSystemParallel.h
    src/ChDataManager.h
    src/ChLcpSolverParallel.h
    src/ChLcpSystemDescriptorParallel.h
   )
SET(ChronoEngine_unit_GPU_CU
    src/ChLcpSolverParallel.cpp
    src/ChBaseParallel.cpp
    src/ChDataManager.cpp
   )

SOURCE_GROUP(unit_GPU FILES
             ${ChronoEngine_unit_GPU_SOURCES}
             ${ChronoEngine_unit_GPU_HEADERS}
             ${ChronoEngine_unit_GPU_CU})
             
SOURCE_GROUP(unit_GPU\\collision FILES
             ${ChronoEngine_unit_GPU_COLLISION})
             
SOURCE_GROUP(unit_GPU\\solver FILES
             ${ChronoEngine_unit_GPU_SOLVER})
                   
SOURCE_GROUP(unit_GPU\\constraints FILES
	     ${ChronoEngine_unit_GPU_CONSTRAINTS})

# User specified path to ChronoEngine
SET(CHRONO_INC       ""   CACHE PATH "Where is ChronoEngine Source located")

# Include directories based on user path to ChronoEngine
SET(INC_DIRS ${CHRONO_INC} "${CHRONO_INC}/collision" "${CHRONO_INC}/collision/bullet")

# User specified path to ChronoEngine Libraries
SET(CHRONO_LIB_PATH  ""   CACHE PATH "Where is CHRONOLIB located")

# Find required Libraries
FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)

SET(INCLUDE_DIR
	${CH_CUDA_INC} 
	${CHRONO_INC} 
	${CHRONO_INC}/collision/bullet/ 
	${CMAKE_SOURCE_DIR}/src
	)

SET(INCLUDE_LIB
	#${CHRONO_LIB_POSTPROCESS}
    #${CHRONO_LIB_OPENGL}
	${CHRONO_LIB}
	)



#-----------------------------------------------------------------------------
# In most cases, you do not need to edit the lines below.

# Use the FindCUDA.cmake to find the CUDA toolkit and set many interesting 
# variables and compilation rules. This saves eons of time.
# Interesting vars that are set: paths CUDA_SDK_ROOT_DIR and CUDA_TOOLKIT_ROOT_DIR

find_package(CUDA QUIET)
if (CUDA_FOUND)
    message(STATUS "CUDA found")
else()
    message(FATAL_ERROR "CUDA not found! (you may switch off ENABLE_UNIT_GPU and build without this unit)")
endif()

# Detect what architecture we're on
# Paths we'll use to find some of the extra libraries
# Ensure that correct path for architecture is selected
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
SET(CUDA_TOOLKIT_LIB_DIR
    ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64)
SET(CUDA_SDK_LIB_DIR
    ${CUDA_SDK_ROOT_DIR}/common/lib/x64
    ${CUDA_SDK_ROOT_DIR}/common/lib/linux/x86_64
    ${CUDA_SDK_ROOT_DIR}/common/lib/darwin)
ELSE()
	message(FATAL_ERROR "Architecture detected as 32-bit. Only 64-bit Builds Supported!")
ENDIF()

find_library(CH_CUDART NAMES cudart libcudart PATHS ${CUDA_TOOLKIT_LIB_DIR})

SET (CH_CUDA_INC     	"${CUDA_TOOLKIT_ROOT_DIR}/include" )
SET (CH_CUDA_SDK_INC 	"${CUDA_SDK_ROOT_DIR}/common/inc"  )

# These need to get straightened out
# Anything using the SDK should only be used by nvcc, no need to include them for anything being sent to the system compiler
INCLUDE_DIRECTORIES(${INCLUDE_DIR} ${CUDA_TOOLKIT_INCLUDE})

# The library is added to the project, and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_Parallel SHARED
                 ${ChronoEngine_unit_GPU_COLLISION}
                 ${ChronoEngine_unit_GPU_CONSTRAINTS}
                 ${ChronoEngine_unit_GPU_SOLVER}
                 ${ChronoEngine_unit_GPU_SOURCES}
                 ${ChronoEngine_unit_GPU_HEADERS}
                 ${ChronoEngine_unit_GPU_CU})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -DWIN32_LEAN_AND_MEAN")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DWIN32_LEAN_AND_MEAN")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	SET_TARGET_PROPERTIES( ChronoEngine_Parallel PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT YES )
	FIND_LIBRARY(CUDA_FRAMEWORK CUDA)
ENDIF()


IF(${CMAKE_CONFIGURATION_TYPES} MATCHES "Debug")

ENDIF()


SET_TARGET_PROPERTIES(ChronoEngine_Parallel PROPERTIES
                      LINK_FLAGS "${CH_LINKERFLAG_GPU}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_GPU")

TARGET_LINK_LIBRARIES(ChronoEngine_Parallel ${INCLUDE_LIB} ${CUDA_FRAMEWORK} )

INSTALL(TARGETS ChronoEngine_Parallel
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
)

 mark_as_advanced(FORCE 
 CUDA_BUILD_CUBIN
 CUDA_BUILD_EMULATION
 CUDA_SEPARABLE_COMPILATION
 CUDA_SDK_ROOT_DIR
 CUDA_VERBOSE_BUILD
 CUDA_HOST_COMPILER)


