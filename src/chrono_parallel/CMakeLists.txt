#===============================================================================
# CMake configuration file for the ChronoEngine_Parallel library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

# The Chrono Parallel will build even if openmp is not found, it just won't have parallel computation support
# If TBB is defined, thrust will use TBB but the rest of the code will be single threaded

option(ENABLE_MODULE_PARALLEL "Enable the Chrono Parallel module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_PARALLEL)
  mark_as_advanced(FORCE BLAZE_DIR)
  mark_as_advanced(FORCE USE_PARALLEL_DOUBLE)
  mark_as_advanced(FORCE USE_PARALLEL_SIMD)
  
  # GCC > 4.9 does not support CUDA (<= 8.0) by default. 
  # Users may explicitly select CUDA support if they are sure that their compiler
  # does make use of any conflicting features, such as C++14.
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
      message(STATUS "GCC version:  ${CMAKE_CXX_COMPILER_VERSION} does not support USE_PARALLEL_CUDA by default")
      set(USE_PARALLEL_CUDA OFF CACHE OFF "")
      mark_as_advanced(FORCE USE_PARALLEL_CUDA)
    endif()
  endif()

  return()
endif()

message(STATUS "==== Chrono Parallel module ====")

mark_as_advanced(CLEAR BLAZE_DIR)
mark_as_advanced(CLEAR USE_PARALLEL_DOUBLE)
mark_as_advanced(CLEAR USE_PARALLEL_SIMD)
mark_as_advanced(CLEAR USE_PARALLEL_CUDA)

# ------------------------------------------------------------------------------
# Additional compiler flags
# ------------------------------------------------------------------------------

set(CH_PARALLEL_CXX_FLAGS "")
set(CH_PARALLEL_C_FLAGS "")

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----- CUDA support -----

find_package(CUDA)

cmake_dependent_option(USE_PARALLEL_CUDA "Enable CUDA support in Chrono::Parallel" ON "CUDA_FOUND" OFF)


IF(USE_PARALLEL_CUDA)
  SET(CUDA_SEPARABLE_COMPILATION ON)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    IF(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
      IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
        SET(CUDA_SEPARABLE_COMPILATION OFF)
        SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++11")
        SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++11")
      ENDIF()
    ENDIF()
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
  ENDIF()

#    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
#        ${CMAKE_BINARY_DIR} 
#        ${CMAKE_SOURCE_DIR}/cmake/has_cuda_gpu.c
#        CMAKE_FLAGS 
#            -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
#            -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
#        COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
#        RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
#    # message("GPUS_FOUND_MESSAGE: ${RUN_OUTPUT_VAR}") # Display number of GPUs found
#    # COMPILE_RESULT_VAR is TRUE when compile succeeds
#    # RUN_RESULT_VAR is zero when a GPU is found
#    if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
#    #set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
#         #set(CUDA_COMPUTE_CAPABILITY ${RUN_OUTPUT_VAR} CACHE STRING "Compute capability of CUDA-capable GPU present")
#         set(CUDA_GENERATE_CODE "arch=compute_${CUDA_COMPUTE_CAPABILITY},code=sm_${CUDA_COMPUTE_CAPABILITY}" CACHE STRING "Which GPU architectures to generate code for (each arch/code pair will be passed as --generate-code option to nvcc, separate multiple pairs by ;)" FORCE)
#    else()
#       #set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
#       set(CUDA_GENERATE_CODE "arch=compute_30,code=sm_30" CACHE STRING "Which GPU architectures to generate code for (each arch/code pair will be passed as --generate-code option to nvcc, separate multiple pairs by ;)" FORCE)
#    endif()

#  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -gencode=${CUDA_GENERATE_CODE})

  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -gencode=arch=compute_30,code=sm_30)

  SET(CHRONO_PARALLEL_USE_CUDA "#define CHRONO_PARALLEL_USE_CUDA")
ELSE()
  SET(CHRONO_PARALLEL_USE_CUDA "#undef CHRONO_PARALLEL_USE_CUDA")
ENDIF()

# ----- SIMD (SSE or AVX) support -----

cmake_dependent_option(USE_PARALLEL_SIMD "Enable SIMD support in Chrono::Parallel (if available)" ON "CHRONO_HAS_AVX" OFF)

if(USE_PARALLEL_SIMD)
  set(CHRONO_PARALLEL_USE_SIMD "#define CHRONO_PARALLEL_USE_SIMD")
endif()

# ----- Double precision support -----

OPTION(USE_PARALLEL_DOUBLE "Compile Chrono::Parallel with double precision math" ON)

IF(USE_PARALLEL_DOUBLE)
  SET(CHRONO_PARALLEL_USE_DOUBLE "#define CHRONO_PARALLEL_USE_DOUBLE")
ENDIF()

# ----- Thrust library -----

find_package(Thrust)

# ----- Blaze library -----

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(BLAZE_DIR ""  CACHE PATH   "Where is Blaze located?")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ENDIF()

# ----- Configure Blaze and Thrust -----

IF(ENABLE_OPENMP)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP)
ELSEIF(ENABLE_TBB)
  add_definitions(-DBLAZE_USE_CPP_THREADS)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_TBB)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_TBB)
ELSE()
  add_definitions(-DBLAZE_USE_CPP_THREADS)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CPP)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP)
ENDIF()

# ----- BOOST -- required for Blaze -----

set(BOOST_ROOT "" CACHE PATH "Where is Boost located?")
find_package(Boost REQUIRED)

if (Boost_FOUND)
  message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
else()
  mark_as_advanced(CLEAR BOOST_ROOT)
endif()

# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigParallel.h.in
  ${PROJECT_BINARY_DIR}/chrono_parallel/ChConfigParallel.h
  )

install(FILES "${PROJECT_BINARY_DIR}/chrono_parallel/ChConfigParallel.h"
        DESTINATION include/chrono_parallel)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the PARALLEL module
# ------------------------------------------------------------------------------

SET(CH_PARALLEL_INCLUDES
    ${BLAZE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${THRUST_INCLUDE_DIR}
)

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_PARALLEL_CXX_FLAGS "${CH_PARALLEL_CXX_FLAGS}" PARENT_SCOPE)
SET(CH_PARALLEL_C_FLAGS "${CH_PARALLEL_C_FLAGS}" PARENT_SCOPE)
SET(CH_PARALLEL_INCLUDES "${CH_PARALLEL_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono parallel module
# ------------------------------------------------------------------------------

SET(ChronoEngine_Parallel_BASE
    ChApiParallel.h
    ChParallelDefines.h
    ChSettings.h
    ChMeasures.h
    ChDataManager.h
    ChTimerParallel.h
    ChDataManager.cpp
    )

SOURCE_GROUP("" FILES ${ChronoEngine_Parallel_BASE})

SET(ChronoEngine_Parallel_PHYSICS
    physics/ChSystemParallel.h
    physics/Ch3DOFContainer.h
    physics/ChSystemParallel.cpp
    physics/ChSystemParallelDVI.cpp
    physics/ChSystemParallelDEM.cpp
    physics/Ch3DOFContainer.cpp
    physics/ChFluidContainer.cpp
    physics/ChFEAContainer.cpp
    physics/Ch3DOFRigidContainer.cpp
    physics/ChMPMSettings.h
    )

SOURCE_GROUP(physics FILES ${ChronoEngine_Parallel_PHYSICS})

SET(ChronoEngine_Parallel_CUDA
    #math/matrix.cu
    #math/matrixf.cu
    #math/matrixf.cuh
    #math/real3.cu
    #math/real4.cu
    #math/vec3.cu
    physics/ChMPM.cu
    physics/ChMPM.cuh
    physics/MPMUtils.h
    )

SOURCE_GROUP(cuda FILES ${ChronoEngine_Parallel_CUDA})
    
SET(ChronoEngine_Parallel_MATH
    math/ChParallelMath.h
    math/matrix.cpp
    math/matrix.h
    math/other_types.h
    math/real.h
    math/real_double.h
    math/real_single.h
    math/real2.h
    math/real2.cpp
    math/real3.cpp
    math/real3.h
    math/real4.cpp
    math/real4.h
    math/simd_avx.h
    math/simd_non.h
    math/simd_sse.h
    math/sse.h
    math/svd.h
    math/utility.h
    math/vec3.cpp
    )

SOURCE_GROUP(math FILES ${ChronoEngine_Parallel_MATH})

SET(ChronoEngine_Parallel_SOLVER
    solver/ChSystemDescriptorParallel.h
    solver/ChIterativeSolverParallel.h
    solver/ChIterativeSolverParallel.cpp
    solver/ChIterativeSolverParallelDVI.cpp
    solver/ChIterativeSolverParallelDEM.cpp
    solver/ChSolverParallel.h
    solver/ChSolverParallel.cpp
    solver/ChSolverParallelAPGD.cpp
    solver/ChSolverParallelAPGDREF.cpp
    solver/ChSolverParallelMinRes.cpp
    solver/ChSolverParallelBB.cpp
    solver/ChSolverParallelJacobi.cpp
    solver/ChSolverParallelCG.cpp
    solver/ChSolverParallelGS.cpp
    solver/ChSolverParallelSPGQP.cpp
    solver/ChShurProduct.cpp
    )

SOURCE_GROUP(solver FILES ${ChronoEngine_Parallel_SOLVER})

SET(ChronoEngine_Parallel_CONSTRAINTS
    constraints/ChConstraintRigidRigid.cpp
    constraints/ChConstraintRigidRigid.h
    constraints/ChConstraintBilateral.cpp
    constraints/ChConstraintBilateral.h
    constraints/ChConstraintUtils.cpp
    constraints/ChConstraintUtils.h
    )

SOURCE_GROUP(constraints FILES ${ChronoEngine_Parallel_CONSTRAINTS})

SET(ChronoEngine_Parallel_COLLISION
    collision/ChAABBGenerator.cpp
    collision/ChBroadphase.cpp
    collision/ChBroadphaseUtils.h
    collision/ChDataStructures.h
    collision/ChNarrowphaseUtils.h
    collision/ChNarrowphaseMPR.h
    collision/ChNarrowphaseMPR.cpp
    collision/ChNarrowphaseDispatch.cpp
    collision/ChNarrowphaseRUtils.h
    collision/ChNarrowphaseR.h
    collision/ChNarrowphaseR.cpp
    collision/ChCollision.h
    collision/ChCollisionModelParallel.h
    collision/ChCollisionModelParallel.cpp
    collision/ChCollisionSystemParallel.h
    collision/ChCollisionSystemParallel.cpp
    collision/ChCollisionSystemBulletParallel.h
    collision/ChCollisionSystemBulletParallel.cpp
    collision/ChContactContainerParallel.h
    collision/ChContactContainerParallel.cpp
    )

SOURCE_GROUP(collision FILES ${ChronoEngine_Parallel_COLLISION})

# Windows doesn't automatically pick up Bullet sources from Chrono
IF (MSVC)
    SET(ChronoEngine_Parallel_COLLISION_BULLET
    # CollisionDispatch
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp
    # CollisionShapes
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp
    # BroadphaseCollision
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
    # NarrowPhaseCollision
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp	
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp	
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp	
    # gimpact
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.cpp
    # LinearMath
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btAlignedAllocator.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btAlignedAllocator.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btQuickprof.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btQuickprof.cpp
        )

    SOURCE_GROUP(collision\\bullet FILES ${ChronoEngine_Parallel_COLLISION_BULLET})

    SET(ChronoEngine_Parallel_COLLISION
        ${ChronoEngine_Parallel_COLLISION}
        ${ChronoEngine_Parallel_COLLISION_BULLET}
        )
ENDIF()

# ------------------------------------------------------------------------------
# Add the ChronoEngine_parallel library
# ------------------------------------------------------------------------------

IF(USE_PARALLEL_CUDA)
    CUDA_ADD_LIBRARY(ChronoEngine_parallel SHARED
            ${ChronoEngine_Parallel_BASE}
            ${ChronoEngine_Parallel_PHYSICS}
            ${ChronoEngine_Parallel_CUDA}
            ${ChronoEngine_Parallel_COLLISION}
            ${ChronoEngine_Parallel_CONSTRAINTS}
            ${ChronoEngine_Parallel_SOLVER}
            ${ChronoEngine_Parallel_MATH}
            )
    SET(CHRONO_PARALLEL_LINKED_LIBRARIES ChronoEngine ${CUDA_FRAMEWORK} ${MPI_LIBRARIES} ${TBB_LIBRARIES})
ELSE()
    ADD_LIBRARY(ChronoEngine_parallel SHARED
            ${ChronoEngine_Parallel_BASE}
            ${ChronoEngine_Parallel_PHYSICS}
            ${ChronoEngine_Parallel_COLLISION}
            ${ChronoEngine_Parallel_CONSTRAINTS}
            ${ChronoEngine_Parallel_SOLVER}
            ${ChronoEngine_Parallel_MATH}
            )
    SET(CHRONO_PARALLEL_LINKED_LIBRARIES ChronoEngine ${MPI_LIBRARIES} ${TBB_LIBRARIES})
ENDIF()

IF(ENABLE_MODULE_FEA)
    SET(CHRONO_PARALLEL_LINKED_LIBRARIES ${CHRONO_PARALLEL_LINKED_LIBRARIES} ChronoEngine_fea)
ENDIF()

SET_TARGET_PROPERTIES(ChronoEngine_parallel PROPERTIES
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_PARALLEL")

TARGET_LINK_LIBRARIES(ChronoEngine_parallel ${CHRONO_PARALLEL_LINKED_LIBRARIES})

INSTALL(TARGETS ChronoEngine_parallel
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_parallel
        FILES_MATCHING PATTERN "*.h")

IF(USE_PARALLEL_CUDA)
  INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
          DESTINATION include/chrono_parallel
          FILES_MATCHING PATTERN "*.cuh")
ENDIF()

mark_as_advanced(FORCE
    CUDA_BUILD_CUBIN
    CUDA_BUILD_EMULATION
    CUDA_SEPARABLE_COMPILATION
    CUDA_SDK_ROOT_DIR
    CUDA_VERBOSE_BUILD
    CUDA_HOST_COMPILER)







