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
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btUnionFind.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btGhostObject.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp
    # CollisionShapes
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBarrelShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/bt2DShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCEtriangleShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBoxShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleMesh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCompoundShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConeShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexHullShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCylindricalShellShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btMultiSphereShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btEmptyShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleBuffer.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCapsuleShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btUniformScalingShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBox2dShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvex2dShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btShapeHull.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btSdfCollisionShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btMiniSDF.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp
    # BroadphaseCollision
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
    # NarrowPhaseCollision
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp
    # LinearMath
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btQuickprof.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btAlignedAllocator.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btGeometryUtil.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btConvexHull.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btConvexHullComputer.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btSerializer.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btSerializer64.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btPolarDecomposition.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btThreads.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btVector3.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/TaskScheduler/btTaskScheduler.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/TaskScheduler/btThreadSupportPosix.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/TaskScheduler/btThreadSupportWin32.cpp
    # gimpact
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.cpp
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
