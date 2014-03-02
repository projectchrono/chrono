cmake_minimum_required(VERSION 2.8)
project(ChronoEngine_Parallel)

#=============================================================================
# CHRONO::ENGINE   CMake configuration file for Chrono_Parallel
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


IF(${CMAKE_C_COMPILER} MATCHES "icc.*$") 

ENDIF()


if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # using Clang
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	SET(GCC_OPT "sse" CACHE STRING "gcc optimizations")
	SET_PROPERTY(CACHE GCC_OPT PROPERTY STRINGS "mmx" "sse" "sse2" "sse3" "ssse3" "sse4.1" "sse4.2" "avx")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m${GCC_OPT}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m${GCC_OPT}")
    
	find_package("OpenMP" REQUIRED)
	
	# Add the OpenMP-specific compiler and linker flags
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	SET(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    
    
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
	SET(INTEL_OPT "SSE2" CACHE STRING "intel optimizations")
	SET_PROPERTY(CACHE INTEL_OPT PROPERTY STRINGS "SSE2" "SSE3" "SSSE3" "SSE4.1" "SSE4.2" "AVX" "CORE-AVX-I" "CORE-AVX2")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -x${INTEL_OPT}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -x${INTEL_OPT}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
endif()

#add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP)
#add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP)

#
# LIST THE FILES THAT MAKE THE GPU LIBRARY
# NOTE: to add a new source to this unit, just add its name here and re-run the CMake.
#

# User specified path to ChronoEngine
SET(CHRONO_INC       ""   CACHE PATH "Where is ChronoEngine Source located")

SET(ChronoEngine_Parallel_SOURCES
    src/ChSystemParallel.cpp
    src/ChLcpSolverParallel.cpp
    src/ChBaseParallel.cpp
    src/ChDataManager.cpp
    )

SET(ChronoEngine_Parallel_HEADERS
    src/ChApiParallel.h
    src/ChBaseParallel.h
    src/ChParallelDefines.h
    src/ChSystemParallel.h
    src/ChDataManager.h
    src/ChLcpSolverParallel.h
    src/ChLcpSystemDescriptorParallel.h
    )

SOURCE_GROUP(core FILES
    ${ChronoEngine_Parallel_SOURCES}
    ${ChronoEngine_Parallel_HEADERS})

SET(ChronoEngine_Parallel_MATH
    src/math/ChParallelMath.h
    src/math/ChThrustLinearAlgebra.h
    )

SOURCE_GROUP(math FILES
    ${ChronoEngine_Parallel_MATH}
    )

SET(ChronoEngine_Parallel_SOLVER
    src/solver/ChSolverParallel.h
    src/solver/ChSolverParallel.cpp
    #src/solver/ChSolverBlockJacobi.cpp
    src/solver/ChSolverCG.cpp
    src/solver/ChSolverCGS.cpp
    src/solver/ChSolverGD.cpp
    src/solver/ChSolverSD.cpp
    src/solver/ChSolverFN.cpp
    src/solver/ChSolverAPGD.cpp
    src/solver/ChSolverAPGDRS.cpp
    src/solver/ChSolverMinRes.cpp
    src/solver/ChSolverBiCG.cpp
    src/solver/ChSolverBiCGStab.cpp
    )

SOURCE_GROUP(solver FILES
    ${ChronoEngine_Parallel_SOLVER})

SET(ChronoEngine_Parallel_CONSTRAINTS
    src/constraints/ChConstraintRigidRigid.cpp
    src/constraints/ChConstraintRigidRigid.h
    src/constraints/ChConstraintBilateral.cpp
    src/constraints/ChConstraintBilateral.h
    )

SOURCE_GROUP(constraints FILES
    ${ChronoEngine_Parallel_CONSTRAINTS})

SET(ChronoEngine_Parallel_COLLISION
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

SOURCE_GROUP(collision FILES
    ${ChronoEngine_Parallel_COLLISION})

# Windows doesn't automatically pick up Bullet sources from Chrono
IF (MSVC)
    SET(ChronoEngine_Parallel_COLLISION_BULLET
    # CollisionDispatch
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp	
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp	
    # CollisionShapes
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp	
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp
    # BroadphaseCollision
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp	
    # NarrowPhaseCollision	
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp	
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp	
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp	
    # gimpact
        ${CHRONO_INC}/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h
        ${CHRONO_INC}/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.cpp
        ${CHRONO_INC}/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.h
        ${CHRONO_INC}/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.cpp
        ${CHRONO_INC}/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.h
        ${CHRONO_INC}/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.cpp
    # LinearMath
        ${CHRONO_INC}/collision/bullet/LinearMath/btAlignedAllocator.h
        ${CHRONO_INC}/collision/bullet/LinearMath/btAlignedAllocator.cpp
        ${CHRONO_INC}/collision/bullet/LinearMath/btQuickprof.h
        ${CHRONO_INC}/collision/bullet/LinearMath/btQuickprof.cpp
        )

    SOURCE_GROUP(collision\\bullet FILES
        ${ChronoEngine_Parallel_COLLISION_BULLET})

    SET(ChronoEngine_Parallel_COLLISION
        ${ChronoEngine_Parallel_COLLISION}
        ${ChronoEngine_Parallel_COLLISION_BULLET}
        )
ENDIF()



# Include directories based on user path to ChronoEngine
SET(INC_DIRS
    ${CHRONO_INC}
    "${CHRONO_INC}/collision"
    "${CHRONO_INC}/collision/bullet"
    "${CHRONO_INC}/collision/bullet/LinearMath"
    "${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision" )

# User specified path to ChronoEngine Libraries.
SET(CHRONO_LIB_PATH  ""   CACHE PATH "Where is CHRONOLIB located")

# set the compiler as a variable
IF(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    IF(CMAKE_SIZEOF_VOID_P MATCHES 4)
        SET (COMPILER "COMPILER_GCC" CACHE STRING "Compiler Type" FORCE)
    ELSE()
        SET (COMPILER "COMPILER_GCC_X64" CACHE STRING "Compiler Type" FORCE)
    ENDIF()
ENDIF()

IF(MSVC AND CMAKE_CL_64)
    SET (COMPILER "COMPILER_MSVC_X64" CACHE STRING "Compiler Type" FORCE)
ELSEIF(MSVC)
    SET (COMPILER "COMPILER_MSVC" CACHE STRING "Compiler Type" FORCE)
ENDIF()

IF(MSVC)
    # Avoid deprecation warnings
    ADD_DEFINITIONS( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    ADD_DEFINITIONS( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    
    # Specify a single Build type, so we can import the correct .lib for Chrono
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "For single-configuration generators (e.g. make) set the type of build: Release, Debug, RelWithDebugInfo, MinSizeRel")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebugInfo" "MinSizeRel")

    # Set the output directory for DLLs and EXEs
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
ENDIF()

# Justin: on MSVC Windows, I want to be able to link different libraries
# when building Debug/Release
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # find the CE library. First make sure we didn't already find it; if we did, unset the variable
    # need this to easily switch between Debug and Release build types
    IF(CHRONO_LIB)
        UNSET(CHRONO_LIB CACHE)
    ENDIF()

    # Set compiler flags based on which compiler we're using
    IF("${COMPILER}" STREQUAL "COMPILER_MSVC")
        SET(CH_BUILDFLAGS "-DWIN32; -DNOMINMAX; -MP")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET(CH_BUILDFLAGS "-DWIN64; -D_WIN64; -DNOMINMAX; -MP")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_GCC")
        SET(CH_BUILDFLAGS "-DWIN32 -D_MINGW -D_WINDOWS")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_GCC_X64")
        SET(CH_BUILDFLAGS "-DWIN64 -D_MINGW -D_WINDOWS -m64")
    ENDIF()

    # Find the Chrono library based on the build type
    FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH}/${CMAKE_BUILD_TYPE} REQUIRED)

    # Copy the Chrono DLL from/to appropriate place, based on build type
    SET(CHRONO_DLL "${CHRONO_LIB_PATH}/../bin/${CMAKE_BUILD_TYPE}/ChronoEngine.dll")
    FILE(COPY ${CHRONO_DLL} DESTINATION "${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}")

    # Report what library we use and which DLL was copied
    MESSAGE(STATUS "Found chrono lib: ${CHRONO_LIB}")
    MESSAGE(STATUS "Copied chrono dll: ${CHRONO_DLL}")
ELSE()
    # Find Chrono libs
    FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)
ENDIF()

SET(INCLUDE_DIR
    ${CH_CUDA_INC}
    ${CHRONO_INC}
    ${CHRONO_INC}/collision/bullet/
    ${CMAKE_SOURCE_DIR}/src
    ${INC_DIRS}
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
IF(CUDA_FOUND)
    message(STATUS "CUDA found")
ELSE()
    message(FATAL_ERROR "CUDA not found! (you may switch off ENABLE_UNIT_GPU and build without this unit)")
ENDIF()

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

SET(CH_CUDA_INC         "${CUDA_TOOLKIT_ROOT_DIR}/include" )
SET(CH_CUDA_SDK_INC     "${CUDA_SDK_ROOT_DIR}/common/inc"  )

# These need to get straightened out
# Anything using the SDK should only be used by nvcc, no need to include them for anything being sent to the system compiler
INCLUDE_DIRECTORIES(${INCLUDE_DIR} ${CUDA_TOOLKIT_INCLUDE})

# The library is added to the project, and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_Parallel SHARED
            ${ChronoEngine_Parallel_COLLISION}
            ${ChronoEngine_Parallel_CONSTRAINTS}
            ${ChronoEngine_Parallel_SOLVER}
            ${ChronoEngine_Parallel_MATH}
            ${ChronoEngine_Parallel_SOURCES}
            ${ChronoEngine_Parallel_HEADERS})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -DWIN32_LEAN_AND_MEAN")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DWIN32_LEAN_AND_MEAN")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET_TARGET_PROPERTIES( ChronoEngine_Parallel PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT YES)
    FIND_LIBRARY(CUDA_FRAMEWORK CUDA)
ENDIF()

SET_TARGET_PROPERTIES(ChronoEngine_Parallel PROPERTIES
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_GPU}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_GPU")

TARGET_LINK_LIBRARIES(ChronoEngine_Parallel ${INCLUDE_LIB} ${CUDA_FRAMEWORK})

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


