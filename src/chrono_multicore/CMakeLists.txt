#===============================================================================
# CMake configuration file for the ChronoEngine_Multicore library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

# The Chrono::Multicore module will build even if OpenMP is not found,
# it just won't have parallel computation support.
# If TBB is defined, thrust will use TBB but the rest of the code will be single
# threaded

option(ENABLE_MODULE_MULTICORE "Enable the Chrono::Multicore module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_MULTICORE)
  mark_as_advanced(FORCE BLAZE_DIR)
  mark_as_advanced(FORCE USE_MULTICORE_DOUBLE)
  mark_as_advanced(FORCE USE_MULTICORE_SIMD)
  mark_as_advanced(FORCE USE_MULTICORE_CUDA)
  return()
endif()

message(STATUS "==== Chrono Multicore module ====")

# Return now if Thrust not available
if (NOT THRUST_FOUND)
  message(WARNING "Chrono::Multicore requires Thrust, but Thrust was not found; disabling Chrono::Multicore")
  set(ENABLE_MODULE_MULTICORE OFF CACHE BOOL "Enable the Chrono::multicore module" FORCE)
  return()
endif()

mark_as_advanced(CLEAR BLAZE_DIR)
mark_as_advanced(CLEAR USE_MULTICORE_DOUBLE)
mark_as_advanced(CLEAR USE_MULTICORE_SIMD)
mark_as_advanced(CLEAR USE_MULTICORE_CUDA)

# ------------------------------------------------------------------------------
# Additional compiler flags
# ------------------------------------------------------------------------------

set(CH_MULTICORE_CXX_FLAGS "")
set(CH_MULTICORE_C_FLAGS "")

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----- CUDA support -----

cmake_dependent_option(USE_MULTICORE_CUDA "Enable CUDA support in Chrono::Multicore (if available)" OFF "CUDA_FOUND" OFF)

if(USE_MULTICORE_CUDA)
  set(CHRONO_MULTICORE_USE_CUDA "#define CHRONO_MULTICORE_USE_CUDA")
else()
  set(CHRONO_MULTICORE_USE_CUDA "#undef CHRONO_MULTICORE_USE_CUDA")
endif()

# ----- Double precision support -----

OPTION(USE_MULTICORE_DOUBLE "Compile Chrono::Multicore with double precision math" ON)

IF(USE_MULTICORE_DOUBLE)
  SET(CHRONO_MULTICORE_USE_DOUBLE "#define CHRONO_MULTICORE_USE_DOUBLE")
ENDIF()

# ----- Blaze library -----

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(BLAZE_DIR ""  CACHE PATH   "Where is Blaze located?")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
else()
  find_path(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ENDIF()

# Extract Blaze version
find_file(BLAZE_VERSION_FILENAME "Version.h" PATHS "${BLAZE_DIR}/blaze/system")
mark_as_advanced(FORCE BLAZE_VERSION_FILENAME)
if(BLAZE_VERSION_FILENAME)
  file(READ ${BLAZE_VERSION_FILENAME} BLAZE_VERSION_FILE)
  message(STATUS "Blaze version file: ${BLAZE_VERSION_FILENAME}")
  string(REGEX MATCH "#define BLAZE_MAJOR_VERSION ([0-9]*)" _BLAZE_MAJOR_VERSION ${BLAZE_VERSION_FILE})
  set(BLAZE_MAJOR_VERSION ${CMAKE_MATCH_1})
  string(REGEX MATCH "#define BLAZE_MINOR_VERSION ([0-9]*)" _BLAZE_MINOR_VERSION ${BLAZE_VERSION_FILE})
  set(BLAZE_MINOR_VERSION ${CMAKE_MATCH_1})
  set(BLAZE_VERSION "${BLAZE_MAJOR_VERSION}.${BLAZE_MINOR_VERSION}")
  message(STATUS "Blaze version: ${BLAZE_VERSION}")
else()
  message(FATAL_ERROR "Cannot find blaze/system/Version.h.  Set BLAZE_DIR.")
endif()

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

# ----- BOOST -- required only for older versions of Blaze -----

if (BLAZE_VERSION VERSION_LESS "3.2")
  set(BOOST_REQUIRED "TRUE")
else()
  set(BOOST_REQUIRED "FALSE")
endif()

if(BOOST_REQUIRED)
  set(BOOST_ROOT "" CACHE PATH "Where is Boost located?")
  find_package(Boost REQUIRED)

  if (Boost_FOUND)
    message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
  else()
    mark_as_advanced(CLEAR BOOST_ROOT)
    message(FATAL_ERROR "Boost required for Blaze version ${BLAZE_VERSION}. Specify BOOST_ROOT or use Blaze 3.2 or newer.")
  endif()

endif()

# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigMulticore.h.in
  ${PROJECT_BINARY_DIR}/chrono_multicore/ChConfigMulticore.h
  )

install(FILES "${PROJECT_BINARY_DIR}/chrono_multicore/ChConfigMulticore.h"
        DESTINATION include/chrono_multicore)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the MULTICORE module
# ------------------------------------------------------------------------------

set(CH_MULTICORE_INCLUDES
    ${BLAZE_DIR}
    ${THRUST_INCLUDE_DIR}
)

if(BOOST_REQUIRED)
  set(CH_MULTICORE_INCLUDES "${CH_MULTICORE_INCLUDES};${Boost_INCLUDE_DIRS}")
endif()

INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})

message(STATUS "Include dirs: ${CH_MULTICORE_INCLUDES}")

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_MULTICORE_CXX_FLAGS "${CH_MULTICORE_CXX_FLAGS}" PARENT_SCOPE)
SET(CH_MULTICORE_C_FLAGS "${CH_MULTICORE_C_FLAGS}" PARENT_SCOPE)
SET(CH_MULTICORE_INCLUDES "${CH_MULTICORE_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono::Multicore module
# ------------------------------------------------------------------------------

SET(ChronoEngine_Multicore_BASE
    ChApiMulticore.h
    ChMulticoreDefines.h
    ChSettings.h
    ChMeasures.h
    ChDataManager.h
    ChTimerMulticore.h
    ChDataManager.cpp
    )

SOURCE_GROUP("" FILES ${ChronoEngine_Multicore_BASE})

SET(ChronoEngine_Multicore_PHYSICS
    physics/ChSystemMulticore.h
    physics/ChSystemMulticore.cpp
    physics/ChSystemMulticoreNSC.cpp
    physics/ChSystemMulticoreSMC.cpp
    physics/Ch3DOFContainer.h
    physics/Ch3DOFContainer.cpp
    physics/ChFluidKernels.h
    physics/ChFluidContainer.cpp
    physics/ChParticleContainer.cpp
    physics/ChMPMSettings.h
    )

SOURCE_GROUP(physics FILES ${ChronoEngine_Multicore_PHYSICS})

SET(ChronoEngine_Multicore_CUDA
    cuda/matrixf.cuh
    cuda/svd.h
    cuda/ChCudaHelper.cuh
    cuda/ChGPUVector.cuh
    cuda/ChMPM.cu
    cuda/ChMPM.cuh
    cuda/ChMPMUtils.h
    )

SOURCE_GROUP(cuda FILES ${ChronoEngine_Multicore_CUDA})
    
SET(ChronoEngine_Multicore_SOLVER
    solver/ChSystemDescriptorMulticore.h
    solver/ChIterativeSolverMulticore.h
    solver/ChIterativeSolverMulticore.cpp
    solver/ChIterativeSolverMulticoreNSC.cpp
    solver/ChIterativeSolverMulticoreSMC.cpp
    solver/ChSolverMulticore.h
    solver/ChSolverMulticore.cpp
    solver/ChSolverMulticoreAPGD.cpp
    solver/ChSolverMulticoreAPGDREF.cpp
    solver/ChSolverMulticoreMINRES.cpp
    solver/ChSolverMulticoreBB.cpp
    solver/ChSolverMulticoreJacobi.cpp
    solver/ChSolverMulticoreCG.cpp
    solver/ChSolverMulticoreGS.cpp
    solver/ChSolverMulticoreSPGQP.cpp
    solver/ChShurProduct.cpp
    )

SOURCE_GROUP(solver FILES ${ChronoEngine_Multicore_SOLVER})

SET(ChronoEngine_Multicore_CONSTRAINTS
    constraints/ChConstraintRigidRigid.cpp
    constraints/ChConstraintRigidRigid.h
    constraints/ChConstraintBilateral.cpp
    constraints/ChConstraintBilateral.h
    constraints/ChConstraintUtils.cpp
    constraints/ChConstraintUtils.h
    )

SOURCE_GROUP(constraints FILES ${ChronoEngine_Multicore_CONSTRAINTS})

SET(ChronoEngine_Multicore_COLLISION
    collision/ChCollisionSystemChronoMulticore.h
    collision/ChCollisionSystemChronoMulticore.cpp
    collision/ChContactContainerMulticore.h
    collision/ChContactContainerMulticore.cpp
    collision/ChContactContainerMulticoreNSC.h
    collision/ChContactContainerMulticoreNSC.cpp
    collision/ChContactContainerMulticoreSMC.h
    collision/ChContactContainerMulticoreSMC.cpp
    )

SET(ChronoEngine_Multicore_COLLISION ${ChronoEngine_Multicore_COLLISION}
    collision/ChCollisionSystemBulletMulticore.h
    collision/ChCollisionSystemBulletMulticore.cpp
    )

SOURCE_GROUP(collision FILES ${ChronoEngine_Multicore_COLLISION})

# Windows doesn't automatically pick up Bullet sources from Chrono
IF(MSVC)
    SET(ChronoEngine_Multicore_COLLISION_BULLET
    # CollisionDispatch
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtUnionFind.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionDispatcher.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionDispatcherMt.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSphereSphereCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionObject.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSphereBoxCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionWorld.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionWorldImporter.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSimulationIslandManager.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCompoundCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCompoundCompoundCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtManifoldResult.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvexConcaveCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtEmptyCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtSphereTriangleCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvexConvexAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtDefaultCollisionConfiguration.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvexPlaneCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtActivatingCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtBox2dBox2dCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtBoxBoxCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtBoxBoxDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtGhostObject.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtConvex2dConvex2dAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtInternalEdgeUtility.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtHashedSimplePairCache.cpp
    # CollisionShapes
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBarrelShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbt2DShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCEtriangleShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleMesh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleIndexVertexArray.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCollisionShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleCallback.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCompoundShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTetrahedronShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConcaveShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtStridingMeshInterface.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtStaticPlaneShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexHullShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtOptimizedBvh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylindricalShellShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMultiSphereShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtEmptyShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMinkowskiSumShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleBuffer.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtHeightfieldTerrainShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCapsuleShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexInternalShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtUniformScalingShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBox2dShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvex2dShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPointCloudShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMultimaterialTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtScaledBvhTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtShapeHull.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtTriangleIndexVertexMaterialArray.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSdfCollisionShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtMiniSDF.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPolyhedron.cpp
    # BroadphaseCollision
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtAxisSweep3.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtSimpleBroadphase.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtOverlappingPairCache.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtDispatcher.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtDbvt.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtDbvtBroadphase.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/cbtQuantizedBvh.cpp
    # NarrowPhaseCollision
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtContinuousConvexCollision.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtRaycastCallback.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtMinkowskiPenetrationDepthSolver.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkPairDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkEpa2.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/cbtPolyhedralContactClipping.cpp
    # LinearMath
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtQuickprof.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtAlignedAllocator.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtGeometryUtil.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtConvexHull.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtConvexHullComputer.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtSerializer.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtSerializer64.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtPolarDecomposition.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtThreads.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtVector3.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/TaskScheduler/cbtTaskScheduler.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/TaskScheduler/cbtThreadSupportPosix.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/TaskScheduler/cbtThreadSupportWin32.cpp
    # gimpact
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactQuantizedBvh.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactQuantizedBvh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtTriangleShapeEx.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/cbtTriangleShapeEx.cpp
        )

    SOURCE_GROUP(collision\\bullet FILES ${ChronoEngine_Multicore_COLLISION_BULLET})

    SET(ChronoEngine_Multicore_COLLISION
        ${ChronoEngine_Multicore_COLLISION}
        ${ChronoEngine_Multicore_COLLISION_BULLET}
        )
ENDIF()

# ------------------------------------------------------------------------------
# Add the ChronoEngine_multicore library
# ------------------------------------------------------------------------------

IF(USE_MULTICORE_CUDA)
    CUDA_ADD_LIBRARY(ChronoEngine_multicore SHARED
            ${ChronoEngine_Multicore_BASE}
            ${ChronoEngine_Multicore_PHYSICS}
            ${ChronoEngine_Multicore_CUDA}
            ${ChronoEngine_Multicore_COLLISION}
            ${ChronoEngine_Multicore_CONSTRAINTS}
            ${ChronoEngine_Multicore_SOLVER}
            ) 
    SET(CHRONO_MULTICORE_LINKED_LIBRARIES ChronoEngine ${CUDA_FRAMEWORK} ${OPENMP_LIBRARIES} ${TBB_LIBRARIES})
ELSE()
    ADD_LIBRARY(ChronoEngine_multicore SHARED
            ${ChronoEngine_Multicore_BASE}
            ${ChronoEngine_Multicore_PHYSICS}
            ${ChronoEngine_Multicore_COLLISION}
            ${ChronoEngine_Multicore_CONSTRAINTS}
            ${ChronoEngine_Multicore_SOLVER}
            )
    SET(CHRONO_MULTICORE_LINKED_LIBRARIES ChronoEngine ${OPENMP_LIBRARIES} ${TBB_LIBRARIES})
ENDIF()

# On Visual Studio, disable warning C4146 from Blaze
# ("unary minus operator applied to unsigned type, result still unsigned")
if(MSVC)
    target_compile_options(ChronoEngine_multicore PUBLIC "/wd4146" )
endif()

# Disable all warnings from external code
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${ChronoEngine_Multicore_COLLISION_BULLET} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
elseif(MSVC)
  set_source_files_properties(${ChronoEngine_Multicore_COLLISION_BULLET} PROPERTIES COMPILE_OPTIONS "/w")
else()
  set_source_files_properties(${ChronoEngine_Multicore_COLLISION_BULLET} PROPERTIES COMPILE_OPTIONS "-w")
endif()

SET_TARGET_PROPERTIES(ChronoEngine_multicore PROPERTIES
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_multicore PRIVATE "CH_API_COMPILE_MULTICORE")
target_compile_definitions(ChronoEngine_multicore PRIVATE "CH_IGNORE_DEPRECATED")

target_compile_definitions(ChronoEngine_multicore PRIVATE "BT_THREADSAFE")
if (USE_BULLET_DOUBLE)
    target_compile_definitions(ChronoEngine_multicore PRIVATE "BT_USE_DOUBLE_PRECISION")
endif()
if (USE_BULLET_OPENMP)
    target_compile_definitions(ChronoEngine_multicore PRIVATE "BT_USE_OPENMP")
endif()

TARGET_LINK_LIBRARIES(ChronoEngine_multicore ${CHRONO_MULTICORE_LINKED_LIBRARIES})

INSTALL(TARGETS ChronoEngine_multicore
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_multicore
        FILES_MATCHING PATTERN "*.h")

IF(USE_MULTICORE_CUDA)
  INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
          DESTINATION include/chrono_multicore
          FILES_MATCHING PATTERN "*.cuh")
ENDIF()

# Install required chrono_thirdparty headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/easylogging
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

if(USE_MULTICORE_CUDA)
	if (DEFINED CUB_INCLUDE_DIR)
		install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/cub
			DESTINATION include/chrono_thirdparty
			FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
	endif()
endif()
