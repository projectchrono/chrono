#===============================================================================
# CMake configuration file for the ChronoEngine_Parallel library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

# ------------------------------------------------------------------------------
# LIST THE FILES in the ChronoEngine_Parallel LIBRARY
# ------------------------------------------------------------------------------

SET(ChronoEngine_Parallel_SOURCES
    ChSystemParallel.cpp
    ChSystemParallelDVI.cpp
    ChSystemParallelDEM.cpp
    ChLcpSolverParallel.cpp
    ChLcpSolverParallelDVI.cpp
    ChLcpSolverParallelDEM.cpp
    ChBaseParallel.cpp
    ChDataManager.cpp
    )

SET(ChronoEngine_Parallel_HEADERS
    ChApiParallel.h
    ChBaseParallel.h
    ChParallelDefines.h
    ChSystemParallel.h
    ChDataManager.h
    ChLcpSolverParallel.h
    ChLcpSystemDescriptorParallel.h
    ChTimerParallel.h
    )

SOURCE_GROUP(core FILES
    ${ChronoEngine_Parallel_SOURCES}
    ${ChronoEngine_Parallel_HEADERS})

SET(ChronoEngine_Parallel_MATH
    math/ChParallelMath.h
    math/ChThrustLinearAlgebra.h
    math/mat33.h
    math/real.h
    math/real2.h
    math/real3.h
    math/real4.h
    )

SOURCE_GROUP(math FILES
    ${ChronoEngine_Parallel_MATH}
    )

SET(ChronoEngine_Parallel_SOLVER
    solver/ChSolverParallel.h
    solver/ChSolverCG.h
    solver/ChSolverCGS.h
    solver/ChSolverGD.h
    solver/ChSolverSD.h
    solver/ChSolverAPGD.h
    solver/ChSolverAPGDRS.h
    solver/ChSolverMinRes.h
    solver/ChSolverBiCG.h
    solver/ChSolverBiCGStab.h
    solver/ChSolverParallel.cpp
    #solver/ChSolverBlockJacobi.cpp
    solver/ChSolverCG.cpp
    solver/ChSolverCGS.cpp
    solver/ChSolverGD.cpp
    solver/ChSolverSD.cpp
    #solver/ChSolverFN.cpp
    solver/ChSolverAPGD.cpp
    solver/ChSolverAPGDRS.cpp
    solver/ChSolverMinRes.cpp
    solver/ChSolverBiCG.cpp
    solver/ChSolverBiCGStab.cpp
    )

SOURCE_GROUP(solver FILES
    ${ChronoEngine_Parallel_SOLVER})

SET(ChronoEngine_Parallel_CONSTRAINTS
    constraints/ChConstraintRigidRigid.cpp
    constraints/ChConstraintRigidRigid.h
    constraints/ChConstraintBilateral.cpp
    constraints/ChConstraintBilateral.h
    )

SOURCE_GROUP(constraints FILES
    ${ChronoEngine_Parallel_CONSTRAINTS})

SET(ChronoEngine_Parallel_COLLISION
    collision/ChCAABBGenerator.h
    collision/ChCAABBGenerator.cpp
    collision/ChCBroadphase.h
    collision/ChCBroadphase.cpp
    collision/ChCNarrowphase.h
    collision/ChCNarrowphaseMPRUtils.h
    collision/ChCNarrowphaseMPR.h
    collision/ChCNarrowphaseMPR.cpp
    collision/ChCNarrowphaseRUtils.h
    collision/ChCNarrowphaseR.h
    collision/ChCNarrowphaseR.cpp
    collision/ChCCollisionModelParallel.h
    collision/ChCCollisionModelParallel.cpp
    collision/ChCCollisionSystemParallel.h
    collision/ChCCollisionSystemParallel.cpp
    collision/ChCCollisionSystemBulletParallel.h
    collision/ChCCollisionSystemBulletParallel.cpp
    collision/ChContactContainerParallel.h
    collision/ChContactContainerParallel.cpp
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


# ------------------------------------------------------------------------------
# ADD THE ChronoEngine_Parallel LIBRARY
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_Parallel SHARED
            ${ChronoEngine_Parallel_COLLISION}
            ${ChronoEngine_Parallel_CONSTRAINTS}
            ${ChronoEngine_Parallel_SOLVER}
            ${ChronoEngine_Parallel_MATH}
            ${ChronoEngine_Parallel_SOURCES}
            ${ChronoEngine_Parallel_HEADERS})


IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET_TARGET_PROPERTIES( ChronoEngine_Parallel PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT YES)
    FIND_LIBRARY(CUDA_FRAMEWORK CUDA)
ENDIF()

SET_TARGET_PROPERTIES(ChronoEngine_Parallel PROPERTIES
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_GPU}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_PARALLEL")

TARGET_LINK_LIBRARIES(ChronoEngine_Parallel ${CHRONO_LIB} ${CUDA_FRAMEWORK} ${MPI_LIBRARIES})

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







