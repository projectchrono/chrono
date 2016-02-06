#=============================================================================
#
# CMake configuration file for the main ChronoEngine library.
#
#=============================================================================

message(STATUS "==== Chrono Engine core module ====")

#-----------------------------------------------------------------------------
# Includes for the main library
#-----------------------------------------------------------------------------

set(CH_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}")
set(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/collision/bullet" )
set(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/collision/gimpact" )
set(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/collision/convexdecomposition/HACD" )

# Make CH_INCLUDES visible in parent scope
set(CH_INCLUDES "${CH_INCLUDES}" PARENT_SCOPE)

include_directories(${CH_INCLUDES})

#-----------------------------------------------------------------------------
# Collect files in the main ChronoEngine library
#-----------------------------------------------------------------------------

# Core group

set(ChronoEngine_core_SOURCES
    core/ChLog.cpp
    core/ChClassRegister.cpp
    core/ChFileutils.cpp
    core/ChStream.cpp
    core/ChMathematics.cpp
    core/ChVector.cpp
    core/ChQuaternion.cpp
    core/ChCoordsys.cpp
    core/ChLinkedListMatrix.cpp
    core/ChQuadrature.cpp
    core/ChBezierCurve.cpp
    )

set(ChronoEngine_core_HEADERS
    core/ChApiCE.h
    core/ChChrono.h
    core/ChClassRegister.h
    core/ChCoordsys.h
    core/ChException.h
    core/ChFrame.h
    core/ChFrameMoving.h
    core/ChHashFunction.h
    core/ChHashTable.h
    core/ChLinearAlgebra.h
    core/ChLists.h
    core/ChLog.h
    core/ChMath.h
    core/ChMathematics.h
    core/ChMatrix.h
    core/ChMatrixDynamic.h
    core/ChMatrixNM.h
    core/ChMatrix33.h
    core/ChVectorDynamic.h
    core/ChPlatform.h
    core/ChQuaternion.h
    core/ChRunTimeType.h
    core/ChFileutils.h
    core/ChRealtimeStep.h
    core/ChStream.h
    core/ChTimer.h
    core/ChTransform.h
    core/ChVector.h
    core/ChSparseMatrix.h
    core/ChLinkedListMatrix.h
    core/ChWrapHashmap.h
    core/ChDistribution.h
    core/ChQuadrature.h
    core/ChTemplateExpressions.h
    core/ChBezierCurve.h
    )

source_group(core FILES
    ${ChronoEngine_core_SOURCES}
    ${ChronoEngine_core_HEADERS})

# Serialization group

set(ChronoEngine_serialization_SOURCES
    )

set(ChronoEngine_serialization_HEADERS
    serialization/ChArchive.h
    serialization/ChArchiveBinary.h
    serialization/ChArchiveAsciiDump.h
    serialization/ChArchiveJSON.h
    )

source_group(serialization FILES
    ${ChronoEngine_serialization_SOURCES}
    ${ChronoEngine_serialization_HEADERS})

# Physics group

set(ChronoEngine_physics_SOURCES
    physics/ChFilePS.cpp
    physics/ChNlsolver.cpp
    physics/ChFunction.cpp
    physics/ChLimit.cpp
    physics/ChObject.cpp
    physics/ChMarker.cpp
    physics/ChForce.cpp
    physics/ChBodyFrame.cpp
    physics/ChBody.cpp
    physics/ChBodyAuxRef.cpp
    physics/ChLinkForce.cpp
    physics/ChLinkMask.cpp
    physics/ChLinkBase.cpp
    physics/ChLink.cpp
    physics/ChLinkDistance.cpp
    physics/ChLinkMasked.cpp
    physics/ChLinkMarkers.cpp
    physics/ChLinkNumdiff.cpp
    physics/ChLinkLock.cpp
    physics/ChLinkBrake.cpp
    physics/ChLinkEngine.cpp
    physics/ChLinkGear.cpp
    physics/ChLinkPulley.cpp
    physics/ChLinkLinActuator.cpp
    physics/ChLinkPneumaticActuator.cpp
    physics/ChLinkScrew.cpp
    physics/ChLinkSpring.cpp
    physics/ChLinkSpringCB.cpp
    physics/ChLinkWheel.cpp
    physics/ChLinkClearance.cpp
    physics/ChLinkPointSpline.cpp
    physics/ChLinkTrajectory.cpp
    physics/ChLinkMate.cpp
    physics/ChLinkRackpinion.cpp
    physics/ChLinkRevolute.cpp
    physics/ChLinkRevoluteSpherical.cpp
    physics/ChLinkRevoluteTranslational.cpp
    physics/ChLinkUniversal.cpp
    physics/ChSystem.cpp
    physics/ChGlobal.cpp
    physics/ChEvents.cpp
    physics/ChSolvmin.cpp
    physics/ChProbe.cpp
    physics/ChControls.cpp
    physics/ChController.cpp
    physics/ChIterative.cpp
    physics/ChRef.cpp
    physics/ChConstraint.cpp
    physics/ChPhysicsItem.cpp
    physics/ChParticlesClones.cpp
    physics/ChIndexedParticles.cpp
    physics/ChIndexedNodes.cpp
    physics/ChNodeBase.cpp
    physics/ChNodeXYZ.cpp
    physics/ChMatterSPH.cpp
    physics/ChContactContainerBase.cpp
    physics/ChContactContainerDVI.cpp
    physics/ChContactContainerDEM.cpp
    physics/ChProximityContainerBase.cpp
    physics/ChProximityContainerSPH.cpp
    physics/ChShaft.cpp
    physics/ChShaftsGear.cpp
    physics/ChShaftsGearbox.cpp
    physics/ChShaftsGearboxAngled.cpp
    physics/ChShaftsClutch.cpp
    physics/ChShaftsPlanetary.cpp
    physics/ChShaftsMotor.cpp
    physics/ChShaftsBody.cpp
    physics/ChShaftsTorqueBase.cpp
    physics/ChShaftsTorque.cpp
    physics/ChShaftsTorsionSpring.cpp
    physics/ChShaftsTorqueConverter.cpp
    physics/ChShaftsThermalEngine.cpp
    physics/ChConveyor.cpp
    physics/ChFx.cpp
    physics/ChAssembly.cpp
    physics/ChSystemDEM.cpp
    physics/ChMaterialSurfaceDEM.cpp
    physics/ChMaterialSurface.cpp
    physics/ChContinuumMaterial.cpp
	physics/ChLoadContainer.cpp
    )

set(ChronoEngine_physics_HEADERS
    physics/ChBodyFrame.h
    physics/ChBody.h
    physics/ChBodyAuxRef.h
    physics/ChBodyEasy.h
    physics/ChConstraint.h
    physics/ChContactContainerBase.h
    physics/ChContactContainerDVI.h
    physics/ChContactContainerDEM.h
    physics/ChController.h
    physics/ChControls.h
    physics/ChConveyor.h
    physics/ChEvents.h
    physics/ChExternalObject.h
    physics/ChFilePS.h
    physics/ChForce.h
    physics/ChFunction.h
    physics/ChFx.h
    physics/ChGlobal.h
    physics/ChIndexedNodes.h
    physics/ChIndexedParticles.h
    physics/ChIterative.h
    physics/ChLimit.h
    physics/ChLinkBase.h
    physics/ChLink.h
    physics/ChLinkBrake.h
    physics/ChLinkDistance.h
    physics/ChLinkEngine.h
    physics/ChLinkForce.h
    physics/ChLinkGear.h
    physics/ChLinkLinActuator.h
    physics/ChLinkLock.h
    physics/ChLinkMarkers.h
    physics/ChLinkMask.h
    physics/ChLinkMasked.h
    physics/ChLinkNumdiff.h
    physics/ChLinkPneumaticActuator.h
    physics/ChLinkPointSpline.h
    physics/ChLinkPulley.h
    physics/ChLinksAll.h
    physics/ChLinkScrew.h
    physics/ChLinkSpring.h
    physics/ChLinkSpringCB.h
    physics/ChLinkTrajectory.h
    physics/ChLinkWheel.h
    physics/ChLinkClearance.h
    physics/ChLinkMate.h
    physics/ChLinkRackpinion.h
    physics/ChLinkRevolute.h
    physics/ChLinkRevoluteSpherical.h
    physics/ChLinkRevoluteTranslational.h
    physics/ChLinkUniversal.h
    physics/ChMarker.h
    physics/ChMaterialCouple.h
    physics/ChMaterialSurfaceBase.h
    physics/ChMaterialSurface.h
    physics/ChMaterialSurfaceDEM.h
    physics/ChMatterSPH.h
    physics/ChNlsolver.h
    physics/ChNodeBase.h
    physics/ChNodeXYZ.h
    physics/ChObject.h
    physics/ChParticlesClones.h
    physics/ChPhysicsItem.h
    physics/ChProbe.h
    physics/ChProplist.h
    physics/ChProximityContainerBase.h
    physics/ChProximityContainerSPH.h
    physics/ChRef.h
    physics/ChScriptEngine.h
    physics/ChShaft.h
    physics/ChShaftsBody.h
    physics/ChShaftsClutch.h
    physics/ChShaftsCouple.h
    physics/ChShaftsGear.h
    physics/ChShaftsGearbox.h
    physics/ChShaftsGearboxAngled.h
    physics/ChShaftsMotor.h
    physics/ChShaftsPlanetary.h
    physics/ChShaftsTorqueBase.h
    physics/ChShaftsTorque.h
    physics/ChShaftsTorsionSpring.h
    physics/ChShaftsTorqueConverter.h
    physics/ChShaftsThermalEngine.h
    physics/ChSolver.h
    physics/ChSolvmin.h
    physics/ChSystem.h
    physics/ChAssembly.h
    physics/ChSystemDEM.h
    physics/ChContactDEM.h
    physics/ChContactDVI.h
    physics/ChContactDVIrolling.h
    physics/ChTensors.h
    physics/ChContinuumMaterial.h
    physics/ChInertiaUtils.h
    physics/ChContactable.h
    physics/ChContactTuple.h
	physics/ChLoadable.h
	physics/ChLoader.h
	physics/ChLoaderU.h
	physics/ChLoaderUV.h
	physics/ChLoaderUVW.h
	physics/ChLoad.h
	physics/ChLoadsXYZnode.h
	physics/ChLoadsBody.h
	physics/ChLoadBodyMesh.h
	physics/ChLoadContainer.h
    )

source_group(physics FILES
    ${ChronoEngine_physics_SOURCES}
    ${ChronoEngine_physics_HEADERS})

# Geometry group

set(ChronoEngine_geometry_SOURCES
    geometry/ChCGeometry.cpp
    geometry/ChCSphere.cpp
    geometry/ChCEllipsoid.cpp
    geometry/ChCBox.cpp
    geometry/ChCCone.cpp
    geometry/ChCCylinder.cpp
    geometry/ChCCapsule.cpp
    geometry/ChCLine.cpp
    geometry/ChCLineArc.cpp
    geometry/ChCLineSegment.cpp
    geometry/ChCLinePath.cpp
    geometry/ChCLineBezier.cpp
    geometry/ChCLineCam.cpp
    geometry/ChCLinePoly.cpp
    geometry/ChCTriangle.cpp
    geometry/ChCTriangleMeshSoup.cpp
    geometry/ChCTriangleMeshConnected.cpp
    geometry/ChCRoundedBox.cpp
    geometry/ChCRoundedCylinder.cpp
    geometry/ChCRoundedCone.cpp
    )

set(ChronoEngine_geometry_HEADERS
    geometry/ChCGeometry.h
    geometry/ChCSphere.h
    geometry/ChCEllipsoid.h
    geometry/ChCBox.h
    geometry/ChCCone.h
    geometry/ChCCylinder.h
    geometry/ChCCapsule.h
    geometry/ChCLine.h
    geometry/ChCLineCam.h
    geometry/ChCLinePoly.h
    geometry/ChCLineArc.h
    geometry/ChCLineSegment.h
    geometry/ChCLinePath.h
    geometry/ChCLineBezier.h
    geometry/ChCTriangle.h
    geometry/ChCTriangleMesh.h
    geometry/ChCTriangleMeshSoup.h
    geometry/ChCTriangleMeshConnected.h
    geometry/ChCRoundedBox.h
    geometry/ChCRoundedCylinder.h
    geometry/ChCRoundedCone.h
    )

source_group(geometry FILES
    ${ChronoEngine_geometry_SOURCES}
    ${ChronoEngine_geometry_HEADERS})

# Asset group

set(ChronoEngine_assets_SOURCES
    assets/ChAsset.cpp
    assets/ChVisualization.cpp
    assets/ChObjShapeFile.cpp
    assets/ChTriangleMeshShape.cpp
    assets/ChSphereShape.cpp
    assets/ChEllipsoidShape.cpp
    assets/ChBoxShape.cpp
    assets/ChConeShape.cpp
    assets/ChCylinderShape.cpp
    assets/ChCapsuleShape.cpp
    assets/ChTexture.cpp
    assets/ChCamera.cpp
    assets/ChAssetLevel.cpp
    assets/ChColor.cpp
    assets/ChGlyphs.cpp
    assets/ChColorAsset.cpp
    assets/ChRoundedCylinderShape.cpp
    assets/ChRoundedBoxShape.cpp
    assets/ChRoundedConeShape.cpp
    assets/ChPathShape.cpp
    assets/ChLineShape.cpp
    assets/ChEmitterAsset.cpp
    )

set(ChronoEngine_assets_HEADERS
    assets/ChAsset.h
    assets/ChVisualization.h
    assets/ChObjShapeFile.h
    assets/ChTriangleMeshShape.h
    assets/ChSphereShape.h
    assets/ChEllipsoidShape.h
    assets/ChBoxShape.h
    assets/ChConeShape.h
    assets/ChCylinderShape.h
    assets/ChCapsuleShape.h
    assets/ChTexture.h
    assets/ChCamera.h
    assets/ChAssetLevel.h
    assets/ChColor.h
    assets/ChMaterial.h
    assets/ChGlyphs.h
    #assets/ChGenericRender.h
    assets/ChColorAsset.h
    assets/ChRoundedCylinderShape.h
    assets/ChRoundedBoxShape.h
    assets/ChRoundedConeShape.h
    assets/ChPathShape.h
    assets/ChLineShape.h
    assets/ChEmitterAsset.h
    )

source_group(assets FILES
    ${ChronoEngine_assets_SOURCES}
    ${ChronoEngine_assets_HEADERS})

# Collision group

set(ChronoEngine_collision_SOURCES
    collision/ChCCollisionModel.cpp
    collision/ChCModelBullet.cpp
    collision/ChCCollisionSystemBullet.cpp
    collision/ChCConvexDecomposition.cpp
    collision/ChCCollisionUtils.cpp
    )

set(ChronoEngine_collision_HEADERS
    collision/ChCCollisionInfo.h
    collision/ChCCollisionModel.h
    collision/ChCCollisionPair.h
    collision/ChCCollisionSystem.h
    collision/ChCCollisionSystemBullet.h
    collision/ChCConvexDecomposition.h
    collision/ChCModelBullet.h
    collision/ChCCollisionUtils.h
    )

source_group(collision FILES
    ${ChronoEngine_collision_SOURCES}
    ${ChronoEngine_collision_HEADERS})

# Edgetempest group

set(ChronoEngine_collision_edgetempest_SOURCES
    collision/edgetempest/ChCCollisionTree.cpp
    collision/edgetempest/ChCOBBTree.cpp
    collision/edgetempest/ChCAABBTree.cpp
    collision/edgetempest/ChCOBB.cpp
    collision/edgetempest/ChCAABB.cpp
    collision/edgetempest/ChCOBBcollider.cpp
    collision/edgetempest/ChCAABBcollider.cpp
    collision/edgetempest/ChCNarrowPhaseCollider.cpp
    collision/edgetempest/ChCGeometryCollider.cpp
    )

set(ChronoEngine_collision_edgetempest_HEADERS
    collision/edgetempest/ChCAABB.h
    collision/edgetempest/ChCAABBcollider.h
    collision/edgetempest/ChCAABBTree.h
    collision/edgetempest/ChCAbsoluteAABB.h
    collision/edgetempest/ChCBroadPhaseCollider.h
    collision/edgetempest/ChCBruteForce.h
    collision/edgetempest/ChCCollisionTree.h
    collision/edgetempest/ChCCompile.h
    collision/edgetempest/ChCGeometryCollider.h
    collision/edgetempest/ChCGetTime.h
    collision/edgetempest/ChCMates.h
    collision/edgetempest/ChCMatVec.h
    collision/edgetempest/ChCNarrowPhaseCollider.h
    collision/edgetempest/ChCOBB.h
    collision/edgetempest/ChCOBBcollider.h
    collision/edgetempest/ChCOBBTree.h
    collision/edgetempest/ChCSweepAndPrune.h
    )

source_group(collision\\edgetempest FILES
    ${ChronoEngine_collision_edgetempest_SOURCES}
    ${ChronoEngine_collision_edgetempest_HEADERS})

# Particle factory group

set(ChronoEngine_particlefactory_SOURCES
    )

set(ChronoEngine_particlefactory_HEADERS
    particlefactory/ChParticleEmitter.h
    particlefactory/ChRandomShapeCreator.h
    particlefactory/ChRandomParticlePosition.h
    particlefactory/ChRandomParticleAlignment.h
    particlefactory/ChRandomParticleVelocity.h
    particlefactory/ChParticleRemover.h
    particlefactory/ChParticleEventTrigger.h
    particlefactory/ChParticleProcessEvent.h
    particlefactory/ChParticleProcessor.h
    )

source_group(particlefactory FILES
    ${ChronoEngine_particlefactory_SOURCES}
    ${ChronoEngine_particlefactory_HEADERS})

# Timestepper group

set(ChronoEngine_timestepper_SOURCES
    timestepper/ChTimestepper.cpp
    )

set(ChronoEngine_timestepper_HEADERS
    timestepper/ChState.h
    timestepper/ChIntegrable.h
    timestepper/ChTimestepper.h
    timestepper/ChStaticAnalysis.h
    )

source_group(timestepper FILES
    ${ChronoEngine_timestepper_SOURCES}
    ${ChronoEngine_timestepper_HEADERS})

# Pneumatica group

set(ChronoEngine_pneumatica_SOURCES
    pneumatica/assepneumatico.cpp
    pneumatica/pistone.cpp
    pneumatica/pistone_3_2.cpp
    pneumatica/pistone_3_2_prop.cpp
    pneumatica/sis_attuatore_3_2.cpp
    pneumatica/sis_attuatore_3_2_prop.cpp
    pneumatica/sistema.cpp
    pneumatica/ugello.cpp
    pneumatica/ugello_controllato.cpp
    pneumatica/ugello_controllato_PA.cpp
    pneumatica/ugello_controllato_RA.cpp
    pneumatica/valvola_3_2.cpp
    pneumatica/valvola_3_2_prop.cpp
    )

set(ChronoEngine_pneumatica_HEADERS
    pneumatica/assepneumatico.h
    pneumatica/pistone.h
    pneumatica/pistone_3_2.h
    pneumatica/pistone_3_2_prop.h
    pneumatica/sis_attuatore_3_2.h
    pneumatica/sis_attuatore_3_2_prop.h
    pneumatica/sistema.h
    pneumatica/ugello.h
    pneumatica/ugello_controllato.h
    pneumatica/ugello_controllato_PA.h
    pneumatica/ugello_controllato_RA.h
    pneumatica/valvola_3_2.h
    pneumatica/valvola_3_2_prop.h
    pneumatica/volume.h
    )

source_group(pneumatica FILES
    ${ChronoEngine_pneumatica_SOURCES}
    ${ChronoEngine_pneumatica_HEADERS})

# Motion functions group

set(ChronoEngine_motion_functions_SOURCES
    motion_functions/ChFunction_Base.cpp
    motion_functions/ChFunction_Const.cpp
    motion_functions/ChFunction_ConstAcc.cpp
    motion_functions/ChFunction_Derive.cpp
    motion_functions/ChFunction_Fillet3.cpp
    motion_functions/ChFunction_Integrate.cpp
    motion_functions/ChFunction_Matlab.cpp
    motion_functions/ChFunction_Mirror.cpp
    motion_functions/ChFunction_Mocap.cpp
    motion_functions/ChFunction_Noise.cpp
    motion_functions/ChFunction_Operation.cpp
    motion_functions/ChFunction_Oscilloscope.cpp
    motion_functions/ChFunction_Poly345.cpp
    motion_functions/ChFunction_Poly.cpp
    motion_functions/ChFunction_Ramp.cpp
    motion_functions/ChFunction_Recorder.cpp
    motion_functions/ChFunction_Repeat.cpp
    motion_functions/ChFunction_Sequence.cpp
    motion_functions/ChFunction_Sigma.cpp
    motion_functions/ChFunction_Sine.cpp
    )

set(ChronoEngine_motion_functions_HEADERS
    motion_functions/ChFunction_Base.h
    motion_functions/ChFunction_Const.h
    motion_functions/ChFunction_ConstAcc.h
    motion_functions/ChFunction_Derive.h
    motion_functions/ChFunction_Fillet3.h
    motion_functions/ChFunction_Integrate.h
    motion_functions/ChFunction_Matlab.h
    motion_functions/ChFunction_Mirror.h
    motion_functions/ChFunction_Mocap.h
    motion_functions/ChFunction_Noise.h
    motion_functions/ChFunction_Operation.h
    motion_functions/ChFunction_Oscilloscope.h
    motion_functions/ChFunction_Poly345.h
    motion_functions/ChFunction_Poly.h
    motion_functions/ChFunction_Ramp.h
    motion_functions/ChFunction_Recorder.h
    motion_functions/ChFunction_Repeat.h
    motion_functions/ChFunction_Sequence.h
    motion_functions/ChFunction_Sigma.h
    motion_functions/ChFunction_Sine.h
    )

source_group(motion_functions FILES
    ${ChronoEngine_motion_functions_SOURCES}
    ${ChronoEngine_motion_functions_HEADERS})

# Parallel support group

set(ChronoEngine_parallel_SOURCES
    parallel/ChThreads.cpp
    parallel/ChThreadsPOSIX.cpp
    parallel/ChThreadsWIN32.cpp
    )

set(ChronoEngine_parallel_HEADERS
    parallel/ChOpenMP.h
    parallel/ChThreads.h
    parallel/ChThreadsFunct.h
    parallel/ChThreadsPOSIX.h
    parallel/ChThreadsSync.h
    parallel/ChThreadsWIN32.h
    )

source_group(parallel FILES
    ${ChronoEngine_parallel_SOURCES}
    ${ChronoEngine_parallel_HEADERS})

# LCP group

set(ChronoEngine_lcp_SOURCES
    lcp/ChLcpSystemDescriptor.cpp
    lcp/ChLcpSolver.cpp
    lcp/ChLcpIterativeSOR.cpp
    lcp/ChLcpIterativeSORmultithread.cpp
    lcp/ChLcpIterativeJacobi.cpp
    lcp/ChLcpIterativeSymmSOR.cpp
    lcp/ChLcpIterativeMINRES.cpp
    lcp/ChLcpIterativePMINRES.cpp
    lcp/ChLcpIterativeBB.cpp
    lcp/ChLcpIterativePCG.cpp
    lcp/ChLcpIterativeAPGD.cpp
    lcp/ChLcpSimplexSolver.cpp
    lcp/ChLcpConstraint.cpp
    lcp/ChLcpConstraintTwo.cpp
    lcp/ChLcpConstraintTwoGeneric.cpp
    lcp/ChLcpConstraintTwoGenericBoxed.cpp
    lcp/ChLcpConstraintTwoBodies.cpp
    lcp/ChLcpConstraintNodeFrictionT.cpp
    lcp/ChLcpConstraintNodeContactN.cpp
    lcp/ChLcpConstraintThree.cpp
    lcp/ChLcpConstraintThreeGeneric.cpp
    lcp/ChLcpConstraintThreeBBShaft.cpp
    lcp/ChLcpVariables.cpp
    lcp/ChLcpVariablesGeneric.cpp
    lcp/ChLcpVariablesGenericDiagonalMass.cpp
    lcp/ChLcpVariablesBody.cpp
    lcp/ChLcpVariablesBodySharedMass.cpp
    lcp/ChLcpVariablesBodyOwnMass.cpp
    lcp/ChLcpVariablesShaft.cpp
    lcp/ChLcpVariablesNode.cpp
    lcp/ChLcpKblockGeneric.cpp
    lcp/ChLcpSolverDEM.cpp
    )

set(ChronoEngine_lcp_HEADERS
    lcp/ChLcpConstraint.h
    lcp/ChLcpConstraintNodeContactN.h
    lcp/ChLcpConstraintNodeFrictionT.h
    lcp/ChLcpConstraintThree.h
    lcp/ChLcpConstraintThreeBBShaft.h
    lcp/ChLcpConstraintThreeGeneric.h
    lcp/ChLcpConstraintTwo.h
    lcp/ChLcpConstraintTwoBodies.h
    lcp/ChLcpConstraintTwoGeneric.h
    lcp/ChLcpConstraintTwoGenericBoxed.h
    lcp/ChLcpConstraintTuple.h
    lcp/ChLcpConstraintTwoTuples.h
    lcp/ChLcpConstraintTwoTuplesContactN.h
    lcp/ChLcpConstraintTwoTuplesFrictionT.h
    lcp/ChLcpConstraintTwoTuplesRollingN.h
    lcp/ChLcpConstraintTwoTuplesRollingT.h
    lcp/ChLcpDirectSolver.h
    lcp/ChLcpIterativeJacobi.h
    lcp/ChLcpIterativeMINRES.h
    lcp/ChLcpIterativePMINRES.h
    lcp/ChLcpIterativeBB.h
    lcp/ChLcpIterativePCG.h
    lcp/ChLcpIterativeAPGD.h
    lcp/ChLcpIterativeSolver.h
    lcp/ChLcpIterativeSOR.h
    lcp/ChLcpIterativeSORmultithread.h
    lcp/ChLcpIterativeSymmSOR.h
    lcp/ChLcpSimplexSolver.h
    lcp/ChLcpSolver.h
    lcp/ChLcpSystemDescriptor.h
    lcp/ChLcpVariables.h
    lcp/ChLcpVariablesBody.h
    lcp/ChLcpVariablesBodyOwnMass.h
    lcp/ChLcpVariablesBodySharedMass.h
    lcp/ChLcpVariablesShaft.h
    lcp/ChLcpVariablesGeneric.h
    lcp/ChLcpVariablesGenericDiagonalMass.h
    lcp/ChLcpVariablesNode.h
    lcp/ChLcpKblock.h
    lcp/ChLcpKblockGeneric.h
    lcp/ChLcpSolverDEM.h
    )

source_group(lcp FILES
    ${ChronoEngine_lcp_SOURCES}
    ${ChronoEngine_lcp_HEADERS})

# Bullet collision group

set(ChronoEngine_collision_bullet_SOURCES
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
    collision/bullet/BulletCollision/CollisionShapes/bt2DShape.cpp
	collision/bullet/BulletCollision/CollisionShapes/btCEtriangleShape.cpp
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

source_group("collision\\bullet" FILES
    ${ChronoEngine_collision_bullet_SOURCES})

# Gimpact group

set(ChronoEngine_collision_gimpact_SOURCES
    collision/gimpact/GIMPACT/core/gim_box_set.cpp
    collision/gimpact/GIMPACT/core/gim_contact.cpp
    collision/gimpact/GIMPACT/core/gim_memory.cpp
    collision/gimpact/GIMPACT/core/gim_tri_collision.cpp
    collision/gimpact/GIMPACT/Bullet/btContactProcessing.cpp
    collision/gimpact/GIMPACT/Bullet/btGenericPoolAllocator.cpp
    collision/gimpact/GIMPACT/Bullet/btGImpactBvh.cpp
    collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.cpp
    collision/gimpact/GIMPACT/Bullet/btGImpactQuantizedBvh.cpp
    collision/gimpact/GIMPACT/Bullet/btGImpactShape.cpp
    collision/gimpact/GIMPACT/Bullet/btTriangleShapeEx.cpp
    collision/gimpact/GIMPACTUtils/btGImpactConvexDecompositionShape.cpp
    collision/gimpact/ConvexDecomposition/bestfit.cpp
    collision/gimpact/ConvexDecomposition/bestfitobb.cpp
    collision/gimpact/ConvexDecomposition/cd_hull.cpp
    collision/gimpact/ConvexDecomposition/concavity.cpp
    collision/gimpact/ConvexDecomposition/ConvexBuilder.cpp
    collision/gimpact/ConvexDecomposition/ConvexDecomposition.cpp
    collision/gimpact/ConvexDecomposition/fitsphere.cpp
    collision/gimpact/ConvexDecomposition/float_math.cpp
    collision/gimpact/ConvexDecomposition/meshvolume.cpp
    collision/gimpact/ConvexDecomposition/planetri.cpp
    collision/gimpact/ConvexDecomposition/raytri.cpp
    collision/gimpact/ConvexDecomposition/splitplane.cpp
    collision/gimpact/ConvexDecomposition/vlookup.cpp
    )

source_group(collision\\gimpact FILES
    ${ChronoEngine_collision_gimpact_SOURCES})

# Convex decomposition JR group

set(ChronoEngine_collision_convexdecompJR_SOURCES
    collision/convexdecomposition/JR/NvConcavityVolume.cpp
    collision/convexdecomposition/JR/NvConvexDecomposition.cpp
    collision/convexdecomposition/JR/NvFloatMath.cpp
    collision/convexdecomposition/JR/NvMeshIslandGeneration.cpp
    collision/convexdecomposition/JR/NvRayCast.cpp
    collision/convexdecomposition/JR/NvRemoveTjunctions.cpp
    collision/convexdecomposition/JR/NvSplitMesh.cpp
    collision/convexdecomposition/JR/NvStanHull.cpp
    )

set(ChronoEngine_collision_convexdecompJR_HEADERS
    collision/convexdecomposition/JR/NvConcavityVolume.h
    collision/convexdecomposition/JR/NvConvexDecomposition.h
    collision/convexdecomposition/JR/NvFloatMath.h
    collision/convexdecomposition/JR/NvHashMap.h
    collision/convexdecomposition/JR/NvMeshIslandGeneration.h
    collision/convexdecomposition/JR/NvRayCast.h
    collision/convexdecomposition/JR/NvRemoveTjunctions.h
    collision/convexdecomposition/JR/NvSimpleTypes.h
    collision/convexdecomposition/JR/NvSplitMesh.h
    collision/convexdecomposition/JR/NvStanHull.h
    collision/convexdecomposition/JR/NvUserMemAlloc.h
    )

source_group(collision\\convexdecomp\\JR FILES 
    ${ChronoEngine_collision_convexdecompJR_SOURCES}
    ${ChronoEngine_collision_convexdecompJR_HEADERS})

# HACD group

set(ChronoEngine_collision_convexdecompHACD_SOURCES
    collision/convexdecomposition/HACD/hacdRaycastMesh.cpp
    collision/convexdecomposition/HACD/hacdMicroAllocator.cpp
    collision/convexdecomposition/HACD/hacdGraph.cpp
    collision/convexdecomposition/HACD/hacdMeshDecimator.cpp
    collision/convexdecomposition/HACD/hacdHACD.cpp
    collision/convexdecomposition/HACD/hacdManifoldMesh.cpp
    collision/convexdecomposition/HACD/hacdICHull.cpp
    )

source_group(collision\\convexdecomp\\HACD FILES
    ${ChronoEngine_collision_convexdecompHACD_SOURCES})

# HACDv2 group

set(ChronoEngine_collision_convexdecompHACDv2_SOURCES
    collision/convexdecomposition/HACDv2/AutoGeometry.cpp
    collision/convexdecomposition/HACDv2/HACD.cpp
    collision/convexdecomposition/HACDv2/ConvexHull.cpp
    collision/convexdecomposition/HACDv2/WuQuantizer.cpp
    collision/convexdecomposition/HACDv2/dgTypes.cpp
    collision/convexdecomposition/HACDv2/dgConvexHull3d.cpp
    collision/convexdecomposition/HACDv2/dgTree.cpp
    collision/convexdecomposition/HACDv2/dgGoogol.cpp
    collision/convexdecomposition/HACDv2/dgSphere.cpp
    collision/convexdecomposition/HACDv2/dgSmallDeterminant.cpp
    collision/convexdecomposition/HACDv2/dgQuaternion.cpp
    collision/convexdecomposition/HACDv2/dgMatrix.cpp
    collision/convexdecomposition/HACDv2/dgPolyhedra.cpp
    collision/convexdecomposition/HACDv2/dgMeshEffect.cpp
    collision/convexdecomposition/HACDv2/MergeHulls.cpp
    collision/convexdecomposition/HACDv2/wavefront.cpp
    )

source_group(collision\\convexdecomp\\HACDv2 FILES
    ${ChronoEngine_collision_convexdecompHACDv2_SOURCES})

# Utility group

set(ChronoEngine_utils_SOURCES
    utils/ChUtilsCreators.cpp
    utils/ChUtilsGenerators.cpp
    utils/ChUtilsInputOutput.cpp
    utils/ChUtilsChaseCamera.cpp
    utils/ChUtilsValidation.cpp
    utils/ChProfiler.cpp
    utils/ChFilters.cpp
    )

set(ChronoEngine_utils_HEADERS
    utils/ChUtilsGeometry.h
    utils/ChUtilsCreators.h
    utils/ChUtilsGenerators.h
    utils/ChUtilsSamplers.h
    utils/ChUtilsInputOutput.h
    utils/ChUtilsChaseCamera.h
    utils/ChUtilsValidation.h
    utils/ChProfiler.h
    utils/ChFilters.h
)

source_group(utils FILES
    ${ChronoEngine_utils_SOURCES}
    ${ChronoEngine_utils_HEADERS})

# Collect all source and header files for the ChronoEngine library

set(ChronoEngine_FILES
    ${ChronoEngine_core_SOURCES}
    ${ChronoEngine_core_HEADERS}
    ${ChronoEngine_serialization_SOURCES}
    ${ChronoEngine_serialization_HEADERS}
    ${ChronoEngine_physics_SOURCES}
    ${ChronoEngine_physics_HEADERS}
    ${ChronoEngine_lcp_SOURCES}
    ${ChronoEngine_lcp_HEADERS}
    ${ChronoEngine_collision_SOURCES}
    ${ChronoEngine_collision_HEADERS}
    ${ChronoEngine_geometry_SOURCES}
    ${ChronoEngine_geometry_HEADERS}
    ${ChronoEngine_assets_SOURCES}
    ${ChronoEngine_assets_HEADERS}
    ${ChronoEngine_collision_edgetempest_SOURCES}
    ${ChronoEngine_collision_edgetempest_HEADERS}
    ${ChronoEngine_collision_convexdecompJR_SOURCES}
    ${ChronoEngine_collision_convexdecompJR_HEADERS}
    ${ChronoEngine_collision_convexdecompHACD_SOURCES}
    ${ChronoEngine_collision_convexdecompHACDv2_SOURCES}
    ${ChronoEngine_pneumatica_SOURCES}
    ${ChronoEngine_pneumatica_HEADERS}
    ${ChronoEngine_particlefactory_SOURCES}
    ${ChronoEngine_particlefactory_HEADERS}
    ${ChronoEngine_timestepper_SOURCES}
    ${ChronoEngine_timestepper_HEADERS}
    ${ChronoEngine_motion_functions_SOURCES}
    ${ChronoEngine_motion_functions_HEADERS}
    ${ChronoEngine_parallel_SOURCES}
    ${ChronoEngine_parallel_HEADERS}
    ${ChronoEngine_collision_bullet_SOURCES}
    ${ChronoEngine_collision_bullet_HEADERS}
    ${ChronoEngine_collision_gimpact_SOURCES}
    ${ChronoEngine_collision_gimpact_HEADERS}
    ${ChronoEngine_utils_SOURCES}
    ${ChronoEngine_utils_HEADERS}
    )

# Add the ChronoEngine library to the project
add_library(ChronoEngine SHARED ${ChronoEngine_FILES})

if (UNIX)
  target_link_libraries(ChronoEngine pthread)
endif()

# Set some custom properties of this target
set_target_properties(ChronoEngine PROPERTIES
    COMPILE_FLAGS "${CH_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
    COMPILE_DEFINITIONS "CH_API_COMPILE")

if(XCODE_VERSION)
    set_target_properties(ChronoEngine PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT ${ENABLE_OPENMP})
endif()

# Install the main ChronoEngine library
install(TARGETS ChronoEngine
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

# Install all header files for the main ChronoEngine library 
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION include/chrono
    FILES_MATCHING REGEX ".*\\.(h|inl)$")
#    FILES_MATCHING PATTERN "*.h")
