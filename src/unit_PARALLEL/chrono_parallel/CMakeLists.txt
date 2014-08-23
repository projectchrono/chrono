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
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp
    # CollisionShapes
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp
    # BroadphaseCollision
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
    # NarrowPhaseCollision
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp	
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp	
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp	
    # gimpact
        ${CH_CHRONO_SDKDIR}/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h
        ${CH_CHRONO_SDKDIR}/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.cpp
        ${CH_CHRONO_SDKDIR}/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.h
        ${CH_CHRONO_SDKDIR}/collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.cpp
        ${CH_CHRONO_SDKDIR}/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.h
        ${CH_CHRONO_SDKDIR}/collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.cpp
    # LinearMath
        ${CH_CHRONO_SDKDIR}/collision/bullet/LinearMath/btAlignedAllocator.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/LinearMath/btAlignedAllocator.cpp
        ${CH_CHRONO_SDKDIR}/collision/bullet/LinearMath/btQuickprof.h
        ${CH_CHRONO_SDKDIR}/collision/bullet/LinearMath/btQuickprof.cpp
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

TARGET_LINK_LIBRARIES(ChronoEngine_Parallel ${CHRONOENGINE_LIBRARY} ${CUDA_FRAMEWORK} ${MPI_LIBRARIES})

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







