#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_CUDA_INC} 
					 ${CH_CUDA_SDK_INC} 
				    )

#--------------------------------------------------------------
SET(ChronoEngine_collision_bullet_SOURCES
	collision/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp
	collision/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp 
	collision/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp
	collision/bullet/BulletCollision/CollisionDispatch/btUnionFind.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btGhostObject.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp 
	collision/bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btBarrelShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btBoxShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btTriangleMesh.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btCompoundShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConeShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConvexHullShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btMultiSphereShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btEmptyShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btTriangleBuffer.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btCapsuleShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btUniformScalingShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btBox2dShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConvex2dShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btShapeHull.cpp 
	collision/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp 
	collision/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp 
	collision/bullet/BulletFileLoader/bChunk.cpp
	collision/bullet/BulletFileLoader/bDNA.cpp 
	collision/bullet/BulletFileLoader/bFile.cpp 
	collision/bullet/BulletFileLoader/btBulletFile.cpp 
	collision/bullet/BulletWorldImporter/btBulletWorldImporter.cpp 
	collision/bullet/LinearMath/btQuickprof.cpp 
	collision/bullet/LinearMath/btAlignedAllocator.cpp 
	collision/bullet/LinearMath/btGeometryUtil.cpp 
	collision/bullet/LinearMath/btConvexHull.cpp 
	collision/bullet/LinearMath/btSerializer.cpp 
)



# Add executables

				    			 	
ADD_EXECUTABLE(test_gpuCD__hammad	test_gpuCD.cpp)
SOURCE_GROUP(testing\\gpu FILES  	test_gpuCD.cpp)
SOURCE_GROUP(collision\\bullet FILES  ${ChronoEngine_collision_bullet_SOURCES})
SET_TARGET_PROPERTIES(test_gpuCD__hammad PROPERTIES 
	FOLDER testing
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(test_gpuCD__hammad 
	${CUDA_SDK_ROOT_DIR}/common/lib/glut64.lib
	${CUDA_SDK_ROOT_DIR}/common/lib/glut32.lib
	${CUDA_SDK_ROOT_DIR}/common/lib/cutil64.lib
	${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cuda.lib
	${CUDA_TOOLKIT_ROOT_DIR}/lib/x64/cudart.lib
 	optimized 	${CH_CHRONOLIB}     
 	debug 		${CH_CHRONOLIB_DBG} 
 	optimized 	${CH_CHRONOGPULIB}
 	debug	    ${CH_CHRONOGPULIB_DBG}	)
ADD_DEPENDENCIES (test_gpuCD__hammad ChronoEngine ChronoEngine_GPU)