INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/src  )

SET(BulletCollision_SRCS
	BroadphaseCollision/cbtAxisSweep3.cpp
	BroadphaseCollision/cbtBroadphaseProxy.cpp
	BroadphaseCollision/cbtCollisionAlgorithm.cpp
	BroadphaseCollision/cbtDbvt.cpp
	BroadphaseCollision/cbtDbvtBroadphase.cpp
	BroadphaseCollision/cbtDispatcher.cpp
	BroadphaseCollision/cbtOverlappingPairCache.cpp
	BroadphaseCollision/cbtQuantizedBvh.cpp
	BroadphaseCollision/cbtSimpleBroadphase.cpp
	CollisionDispatch/cbtActivatingCollisionAlgorithm.cpp
	CollisionDispatch/cbtBoxBoxCollisionAlgorithm.cpp
	CollisionDispatch/cbtBox2dBox2dCollisionAlgorithm.cpp
	CollisionDispatch/cbtBoxBoxDetector.cpp
	CollisionDispatch/cbtCollisionDispatcher.cpp
	CollisionDispatch/cbtCollisionDispatcherMt.cpp
	CollisionDispatch/cbtCollisionObject.cpp
	CollisionDispatch/cbtCollisionWorld.cpp
	CollisionDispatch/cbtCollisionWorldImporter.cpp
	CollisionDispatch/cbtCompoundCollisionAlgorithm.cpp
	CollisionDispatch/cbtCompoundCompoundCollisionAlgorithm.cpp
	CollisionDispatch/cbtConvexConcaveCollisionAlgorithm.cpp
	CollisionDispatch/cbtConvexConvexAlgorithm.cpp
	CollisionDispatch/cbtConvexPlaneCollisionAlgorithm.cpp
	CollisionDispatch/cbtConvex2dConvex2dAlgorithm.cpp
	CollisionDispatch/cbtDefaultCollisionConfiguration.cpp
	CollisionDispatch/cbtEmptyCollisionAlgorithm.cpp
	CollisionDispatch/cbtGhostObject.cpp
	CollisionDispatch/cbtHashedSimplePairCache.cpp
	CollisionDispatch/cbtInternalEdgeUtility.cpp
	CollisionDispatch/cbtInternalEdgeUtility.h
	CollisionDispatch/cbtManifoldResult.cpp
	CollisionDispatch/cbtSimulationIslandManager.cpp
	CollisionDispatch/cbtSphereBoxCollisionAlgorithm.cpp
	CollisionDispatch/cbtSphereSphereCollisionAlgorithm.cpp
	CollisionDispatch/cbtSphereTriangleCollisionAlgorithm.cpp
	CollisionDispatch/cbtUnionFind.cpp
	CollisionDispatch/SphereTriangleDetector.cpp
	CollisionShapes/cbtBoxShape.cpp
	CollisionShapes/cbtBox2dShape.cpp
	CollisionShapes/cbtBvhTriangleMeshShape.cpp
	CollisionShapes/cbtCapsuleShape.cpp
	CollisionShapes/cbtCollisionShape.cpp
	CollisionShapes/cbtCompoundShape.cpp
	CollisionShapes/cbtConcaveShape.cpp
	CollisionShapes/cbtConeShape.cpp
	CollisionShapes/cbtConvexHullShape.cpp
	CollisionShapes/cbtConvexInternalShape.cpp
	CollisionShapes/cbtConvexPointCloudShape.cpp
	CollisionShapes/cbtConvexPolyhedron.cpp
	CollisionShapes/cbtConvexShape.cpp
	CollisionShapes/cbtConvex2dShape.cpp
	CollisionShapes/cbtConvexTriangleMeshShape.cpp
	CollisionShapes/cbtCylinderShape.cpp
	CollisionShapes/cbtEmptyShape.cpp
	CollisionShapes/cbtHeightfieldTerrainShape.cpp
	CollisionShapes/cbtMiniSDF.cpp
	CollisionShapes/cbtMinkowskiSumShape.cpp
	CollisionShapes/cbtMultimaterialTriangleMeshShape.cpp
	CollisionShapes/cbtMultiSphereShape.cpp
	CollisionShapes/cbtOptimizedBvh.cpp
	CollisionShapes/cbtPolyhedralConvexShape.cpp
	CollisionShapes/cbtScaledBvhTriangleMeshShape.cpp
	CollisionShapes/cbtSdfCollisionShape.cpp
	CollisionShapes/cbtShapeHull.cpp
	CollisionShapes/cbtSphereShape.cpp
	CollisionShapes/cbtStaticPlaneShape.cpp
	CollisionShapes/cbtStridingMeshInterface.cpp
	CollisionShapes/cbtTetrahedronShape.cpp
	CollisionShapes/cbtTriangleBuffer.cpp
	CollisionShapes/cbtTriangleCallback.cpp
	CollisionShapes/cbtTriangleIndexVertexArray.cpp
	CollisionShapes/cbtTriangleIndexVertexMaterialArray.cpp
	CollisionShapes/cbtTriangleMesh.cpp
	CollisionShapes/cbtTriangleMeshShape.cpp
	CollisionShapes/cbtUniformScalingShape.cpp
	Gimpact/cbtContactProcessing.cpp
	Gimpact/cbtGenericPoolAllocator.cpp
	Gimpact/cbtGImpactBvh.cpp
	Gimpact/cbtGImpactCollisionAlgorithm.cpp
	Gimpact/cbtGImpactQuantizedBvh.cpp
	Gimpact/cbtGImpactShape.cpp
	Gimpact/cbtTriangleShapeEx.cpp
	Gimpact/gim_box_set.cpp
	Gimpact/gim_contact.cpp
	Gimpact/gim_memory.cpp
	Gimpact/gim_tri_collision.cpp
	NarrowPhaseCollision/cbtContinuousConvexCollision.cpp
	NarrowPhaseCollision/cbtConvexCast.cpp
	NarrowPhaseCollision/cbtGjkConvexCast.cpp
	NarrowPhaseCollision/cbtGjkEpa2.cpp
	NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.cpp
	NarrowPhaseCollision/cbtGjkPairDetector.cpp
	NarrowPhaseCollision/cbtMinkowskiPenetrationDepthSolver.cpp
	NarrowPhaseCollision/cbtPersistentManifold.cpp
	NarrowPhaseCollision/cbtRaycastCallback.cpp
	NarrowPhaseCollision/cbtSubSimplexConvexCast.cpp
	NarrowPhaseCollision/cbtVoronoiSimplexSolver.cpp
	NarrowPhaseCollision/cbtPolyhedralContactClipping.cpp
)

SET(Root_HDRS
	../cbtBulletCollisionCommon.h
)
SET(BroadphaseCollision_HDRS
    BroadphaseCollision/cbtAxisSweep3Internal.h
	BroadphaseCollision/cbtAxisSweep3.h
	BroadphaseCollision/cbtBroadphaseInterface.h
	BroadphaseCollision/cbtBroadphaseProxy.h
	BroadphaseCollision/cbtCollisionAlgorithm.h
	BroadphaseCollision/cbtDbvt.h
	BroadphaseCollision/cbtDbvtBroadphase.h
	BroadphaseCollision/cbtDispatcher.h
	BroadphaseCollision/cbtOverlappingPairCache.h
	BroadphaseCollision/cbtOverlappingPairCallback.h
	BroadphaseCollision/cbtQuantizedBvh.h
	BroadphaseCollision/cbtSimpleBroadphase.h
)
SET(CollisionDispatch_HDRS
	CollisionDispatch/cbtActivatingCollisionAlgorithm.h
	CollisionDispatch/cbtBoxBoxCollisionAlgorithm.h
	CollisionDispatch/cbtBox2dBox2dCollisionAlgorithm.h
	CollisionDispatch/cbtBoxBoxDetector.h
	CollisionDispatch/cbtCollisionConfiguration.h
	CollisionDispatch/cbtCollisionCreateFunc.h
	CollisionDispatch/cbtCollisionDispatcher.h
	CollisionDispatch/cbtCollisionDispatcherMt.h
	CollisionDispatch/cbtCollisionObject.h
	CollisionDispatch/cbtCollisionObjectWrapper.h
	CollisionDispatch/cbtCollisionWorld.h
	CollisionDispatch/cbtCollisionWorldImporter.h
	CollisionDispatch/cbtCompoundCollisionAlgorithm.h
	CollisionDispatch/cbtCompoundCompoundCollisionAlgorithm.h
	CollisionDispatch/cbtConvexConcaveCollisionAlgorithm.h
	CollisionDispatch/cbtConvexConvexAlgorithm.h
	CollisionDispatch/cbtConvex2dConvex2dAlgorithm.h
	CollisionDispatch/cbtConvexPlaneCollisionAlgorithm.h
	CollisionDispatch/cbtDefaultCollisionConfiguration.h
	CollisionDispatch/cbtEmptyCollisionAlgorithm.h
	CollisionDispatch/cbtGhostObject.h
	CollisionDispatch/cbtHashedSimplePairCache.h
	CollisionDispatch/cbtManifoldResult.h
	CollisionDispatch/cbtSimulationIslandManager.h
	CollisionDispatch/cbtSphereBoxCollisionAlgorithm.h
	CollisionDispatch/cbtSphereSphereCollisionAlgorithm.h
	CollisionDispatch/cbtSphereTriangleCollisionAlgorithm.h
	CollisionDispatch/cbtUnionFind.h
	CollisionDispatch/SphereTriangleDetector.h
)
SET(CollisionShapes_HDRS
	CollisionShapes/cbtBoxShape.h
	CollisionShapes/cbtBox2dShape.h
	CollisionShapes/cbtBvhTriangleMeshShape.h
	CollisionShapes/cbtCapsuleShape.h
	CollisionShapes/cbtCollisionMargin.h
	CollisionShapes/cbtCollisionShape.h
	CollisionShapes/cbtCompoundShape.h
	CollisionShapes/cbtConcaveShape.h
	CollisionShapes/cbtConeShape.h
	CollisionShapes/cbtConvexHullShape.h
	CollisionShapes/cbtConvexInternalShape.h
	CollisionShapes/cbtConvexPointCloudShape.h
	CollisionShapes/cbtConvexPolyhedron.h
	CollisionShapes/cbtConvexShape.h
	CollisionShapes/cbtConvex2dShape.h
	CollisionShapes/cbtConvexTriangleMeshShape.h
	CollisionShapes/cbtCylinderShape.h
	CollisionShapes/cbtEmptyShape.h
	CollisionShapes/cbtHeightfieldTerrainShape.h
	CollisionShapes/cbtMaterial.h
	CollisionShapes/cbtMinkowskiSumShape.h
	CollisionShapes/cbtMultimaterialTriangleMeshShape.h
	CollisionShapes/cbtMultiSphereShape.h
	CollisionShapes/cbtOptimizedBvh.h
	CollisionShapes/cbtPolyhedralConvexShape.h
	CollisionShapes/cbtScaledBvhTriangleMeshShape.h
	CollisionShapes/cbtShapeHull.h
	CollisionShapes/cbtSphereShape.h
	CollisionShapes/cbtStaticPlaneShape.h
	CollisionShapes/cbtStridingMeshInterface.h
	CollisionShapes/cbtTetrahedronShape.h
	CollisionShapes/cbtTriangleBuffer.h
	CollisionShapes/cbtTriangleCallback.h
	CollisionShapes/cbtTriangleIndexVertexArray.h
	CollisionShapes/cbtTriangleIndexVertexMaterialArray.h
	CollisionShapes/cbtTriangleInfoMap.h
	CollisionShapes/cbtTriangleMesh.h
	CollisionShapes/cbtTriangleMeshShape.h
	CollisionShapes/cbtTriangleShape.h
	CollisionShapes/cbtUniformScalingShape.h
)
SET(Gimpact_HDRS
	Gimpact/cbtBoxCollision.h
	Gimpact/cbtClipPolygon.h
	Gimpact/cbtContactProcessingStructs.h
	Gimpact/cbtContactProcessing.h
	Gimpact/cbtGenericPoolAllocator.h
	Gimpact/cbtGeometryOperations.h
	Gimpact/cbtGImpactBvhStructs.h
	Gimpact/cbtGImpactBvh.h
	Gimpact/cbtGImpactCollisionAlgorithm.h
	Gimpact/cbtGImpactMassUtil.h
	Gimpact/cbtGImpactQuantizedBvhStructs.h
	Gimpact/cbtGImpactQuantizedBvh.h
	Gimpact/cbtGImpactShape.h
	Gimpact/cbtQuantization.h
	Gimpact/cbtTriangleShapeEx.h
	Gimpact/gim_array.h
	Gimpact/gim_basic_geometry_operations.h
	Gimpact/gim_bitset.h
	Gimpact/gim_box_collision.h
	Gimpact/gim_box_set.h
	Gimpact/gim_clip_polygon.h
	Gimpact/gim_contact.h
	Gimpact/gim_geom_types.h
	Gimpact/gim_geometry.h
	Gimpact/gim_hash_table.h
	Gimpact/gim_linear_math.h
	Gimpact/gim_math.h
	Gimpact/gim_memory.h
	Gimpact/gim_radixsort.h
	Gimpact/gim_tri_collision.h
)
SET(NarrowPhaseCollision_HDRS
	NarrowPhaseCollision/cbtContinuousConvexCollision.h
	NarrowPhaseCollision/cbtConvexCast.h
	NarrowPhaseCollision/cbtConvexPenetrationDepthSolver.h
	NarrowPhaseCollision/cbtDiscreteCollisionDetectorInterface.h
	NarrowPhaseCollision/cbtGjkConvexCast.h
	NarrowPhaseCollision/cbtGjkEpa2.h
	NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.h
	NarrowPhaseCollision/cbtGjkPairDetector.h
	NarrowPhaseCollision/cbtManifoldPoint.h
	NarrowPhaseCollision/cbtMinkowskiPenetrationDepthSolver.h
	NarrowPhaseCollision/cbtPersistentManifold.h
	NarrowPhaseCollision/cbtPointCollector.h
	NarrowPhaseCollision/cbtRaycastCallback.h
	NarrowPhaseCollision/cbtSimplexSolverInterface.h
	NarrowPhaseCollision/cbtSubSimplexConvexCast.h
	NarrowPhaseCollision/cbtVoronoiSimplexSolver.h
	NarrowPhaseCollision/cbtPolyhedralContactClipping.h
)

SET(BulletCollision_HDRS
	${Root_HDRS}
	${BroadphaseCollision_HDRS}
	${CollisionDispatch_HDRS}
	${CollisionShapes_HDRS}
	${Gimpact_HDRS}
	${NarrowPhaseCollision_HDRS}
)


ADD_LIBRARY(BulletCollision ${BulletCollision_SRCS} ${BulletCollision_HDRS})
SET_TARGET_PROPERTIES(BulletCollision PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletCollision PROPERTIES SOVERSION ${BULLET_VERSION})
IF (BUILD_SHARED_LIBS)
  TARGET_LINK_LIBRARIES(BulletCollision LinearMath)
ENDIF (BUILD_SHARED_LIBS)


IF (INSTALL_LIBS)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		#INSTALL of other files requires CMake 2.6
		IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
			IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS BulletCollision DESTINATION .)
			ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS BulletCollision RUNTIME DESTINATION bin
								LIBRARY DESTINATION lib${LIB_SUFFIX}
								ARCHIVE DESTINATION lib${LIB_SUFFIX})
				INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
DESTINATION ${INCLUDE_INSTALL_DIR} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
				INSTALL(FILES ../cbtBulletCollisionCommon.h
DESTINATION ${INCLUDE_INSTALL_DIR}/BulletCollision)
			ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)

		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(BulletCollision PROPERTIES FRAMEWORK true)

			SET_TARGET_PROPERTIES(BulletCollision PROPERTIES PUBLIC_HEADER "${Root_HDRS}")
			# Have to list out sub-directories manually:
			SET_PROPERTY(SOURCE ${BroadphaseCollision_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/BroadphaseCollision)
			SET_PROPERTY(SOURCE ${CollisionDispatch_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/CollisionDispatch)
			SET_PROPERTY(SOURCE ${CollisionShapes_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/CollisionShapes)
			SET_PROPERTY(SOURCE ${Gimpact_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/Gimpact)
			SET_PROPERTY(SOURCE ${NarrowPhaseCollision_HDRS} PROPERTY MACOSX_PACKAGE_LOCATION Headers/NarrowPhaseCollision)

		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF (INSTALL_LIBS)
