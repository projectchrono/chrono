#=============================================================================
#
# CMake configuration file for the main ChronoEngine library.
#
#=============================================================================

message(STATUS "==== Chrono Engine core module ====")

#-----------------------------------------------------------------------------
# Double precision support for Bullet
#-----------------------------------------------------------------------------
option(USE_BULLET_DOUBLE "Compile Chrono::Engine with double precision support in Bullet" OFF)

#-----------------------------------------------------------------------------
# Includes for the main library
#-----------------------------------------------------------------------------

set(CH_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}")
set(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/collision/bullet" )
set(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/collision/gimpact" )
set(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/../chrono_thirdparty/HACD" )
set(CH_INCLUDES ${CH_INCLUDES} "${CMAKE_CURRENT_SOURCE_DIR}/../chrono_thirdparty/HACDv2" )

# Make CH_INCLUDES visible in parent scope
set(CH_INCLUDES "${CH_INCLUDES}" PARENT_SCOPE)

include_directories(${CH_INCLUDES})

#-----------------------------------------------------------------------------
# Collect files in the main ChronoEngine library
#-----------------------------------------------------------------------------

# Core group

set(ChronoEngine_core_SOURCES
    core/ChLog.cpp
    core/ChClassFactory.cpp
    core/ChFilePS.cpp
    core/ChStream.cpp
    core/ChMathematics.cpp
    core/ChQuaternion.cpp
    core/ChVector.cpp
    core/ChCoordsys.cpp
    core/ChQuadrature.cpp
    core/ChBezierCurve.cpp
    core/ChCubicSpline.cpp
    core/ChDistribution.cpp
    core/ChGlobal.cpp
    )

set(ChronoEngine_core_HEADERS
    core/ChApiCE.h
    core/ChChrono.h
    core/ChClassFactory.h
    core/ChCoordsys.h
    core/ChException.h
    core/ChFilePS.h
    core/ChFrame.h
    core/ChFrameMoving.h
    core/ChLists.h
    core/ChLog.h
    core/ChMath.h
    core/ChMathematics.h
    core/ChMatrix.h
    core/ChMatrixEigenExtensions.h
    core/ChSparseMatrixEigenExtensions.h
    core/ChSparsityPatternLearner.h
    core/ChMatrix33.h
    core/ChMatrixMBD.h
    core/ChPlatform.h
    core/ChQuaternion.h
    core/ChRealtimeStep.h
    core/ChStream.h
    core/ChTimer.h
    core/ChTransform.h
    core/ChVector.h
    core/ChVector2.h
    core/ChAlignedAllocator.h
    core/ChDistribution.h
    core/ChQuadrature.h
    core/ChTemplateExpressions.h
    core/ChBezierCurve.h
    core/ChCubicSpline.h
    core/ChBitmaskEnums.h
    core/ChGlobal.h
    core/ChFx.h
    core/ChTypes.h
	core/ChTensors.h
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
    serialization/ChArchiveXML.h
    serialization/ChArchiveExplorer.h
    )

source_group(serialization FILES
    ${ChronoEngine_serialization_SOURCES}
    ${ChronoEngine_serialization_HEADERS})

# Physics group

set(ChronoEngine_physics_SOURCES
    physics/ChObject.cpp
    physics/ChMarker.cpp
    physics/ChForce.cpp
    physics/ChBodyFrame.cpp
    physics/ChBody.cpp
    physics/ChBodyAuxRef.cpp
    physics/ChBodyEasy.cpp
    physics/ChSystem.cpp
    physics/ChSystemNSC.cpp
    physics/ChSystemSMC.cpp
    physics/ChController.cpp
    physics/ChPhysicsItem.cpp
    physics/ChParticlesClones.cpp
    physics/ChIndexedParticles.cpp
    physics/ChIndexedNodes.cpp
    physics/ChNodeBase.cpp
    physics/ChNodeXYZ.cpp
    physics/ChMatterSPH.cpp
    physics/ChProximityContainer.cpp
    physics/ChProximityContainerSPH.cpp
    physics/ChConveyor.cpp
    physics/ChAssembly.cpp
    )

set(ChronoEngine_physics_HEADERS
    physics/ChBodyFrame.h
    physics/ChBody.h
    physics/ChBodyAuxRef.h
    physics/ChBodyEasy.h
    physics/ChController.h
    physics/ChConveyor.h
    physics/ChForce.h
    physics/ChIndexedNodes.h
    physics/ChIndexedParticles.h
    physics/ChMarker.h
    physics/ChMatterSPH.h
    physics/ChNodeBase.h
    physics/ChNodeXYZ.h
    physics/ChObject.h
    physics/ChParticlesClones.h
    physics/ChPhysicsItem.h
    physics/ChProximityContainer.h
    physics/ChProximityContainerSPH.h
    physics/ChSystem.h
    physics/ChSystemNSC.h
    physics/ChSystemSMC.h    
    physics/ChAssembly.h
    physics/ChInertiaUtils.h
    )

source_group(physics FILES
    ${ChronoEngine_physics_SOURCES}
    ${ChronoEngine_physics_HEADERS})

# PHYSICS sub-groups

set(ChronoEngine_physics_contact_SOURCES
    physics/ChContactContainer.cpp
    physics/ChContactContainerNSC.cpp
    physics/ChContactContainerSMC.cpp
    physics/ChMaterialSurface.cpp
    physics/ChMaterialSurfaceSMC.cpp
    physics/ChMaterialSurfaceNSC.cpp
)

set(ChronoEngine_physics_contact_HEADERS
    physics/ChContactContainer.h
    physics/ChContactContainerNSC.h
    physics/ChContactContainerSMC.h
    physics/ChContactable.h
    physics/ChContactTuple.h
    physics/ChContactSMC.h
    physics/ChContactNSC.h
    physics/ChContactNSCrolling.h
    physics/ChMaterialSurface.h
    physics/ChMaterialSurfaceNSC.h
    physics/ChMaterialSurfaceSMC.h
)

source_group(physics\\contact FILES
    ${ChronoEngine_physics_contact_SOURCES}
    ${ChronoEngine_physics_contact_HEADERS})

set(ChronoEngine_physics_links_SOURCES
    physics/ChLimit.cpp
    physics/ChLinkForce.cpp
    physics/ChLinkMask.cpp
    physics/ChLinkBase.cpp
    physics/ChLink.cpp
    physics/ChLinkDistance.cpp
    physics/ChLinkMarkers.cpp
    physics/ChLinkLock.cpp
    physics/ChLinkBrake.cpp
    physics/ChLinkBushing.cpp
    physics/ChLinkGear.cpp
    physics/ChLinkPulley.cpp
    physics/ChLinkLinActuator.cpp
    physics/ChLinkScrew.cpp
    physics/ChLinkTSDA.cpp
    physics/ChLinkSpring.cpp
    physics/ChLinkSpringCB.cpp
    physics/ChLinkRotSpringCB.cpp
    physics/ChLinkClearance.cpp
    physics/ChLinkPointSpline.cpp
    physics/ChLinkTrajectory.cpp
    physics/ChLinkMate.cpp
    physics/ChLinkRackpinion.cpp
    physics/ChLinkRevolute.cpp
    physics/ChLinkRevoluteSpherical.cpp
    physics/ChLinkRevoluteTranslational.cpp
    physics/ChLinkUniversal.cpp
    physics/ChLinkMotor.cpp
    physics/ChLinkMotorRotation.cpp
    physics/ChLinkMotorRotationAngle.cpp
    physics/ChLinkMotorRotationSpeed.cpp
    physics/ChLinkMotorRotationTorque.cpp
    physics/ChLinkMotorRotationDriveline.cpp
    physics/ChLinkMotorLinear.cpp
    physics/ChLinkMotorLinearPosition.cpp
    physics/ChLinkMotorLinearSpeed.cpp
    physics/ChLinkMotorLinearForce.cpp
    physics/ChLinkMotorLinearDriveline.cpp
	physics/ChLinkMotionImposed.cpp
)

set(ChronoEngine_physics_links_HEADERS
    physics/ChLimit.h
    physics/ChLinkBase.h
    physics/ChLink.h
    physics/ChLinkBrake.h
    physics/ChLinkBushing.h
    physics/ChLinkDistance.h
    physics/ChLinkForce.h
    physics/ChLinkGear.h
    physics/ChLinkLinActuator.h
    physics/ChLinkLock.h
    physics/ChLinkMarkers.h
    physics/ChLinkMask.h
    physics/ChLinkPointSpline.h
    physics/ChLinkPulley.h
    physics/ChLinksAll.h
    physics/ChLinkScrew.h
    physics/ChLinkTSDA.h
    physics/ChLinkSpring.h
    physics/ChLinkSpringCB.h
    physics/ChLinkRotSpringCB.h
    physics/ChLinkTrajectory.h
    physics/ChLinkClearance.h
    physics/ChLinkMate.h
    physics/ChLinkRackpinion.h
    physics/ChLinkRevolute.h
    physics/ChLinkRevoluteSpherical.h
    physics/ChLinkRevoluteTranslational.h
    physics/ChLinkUniversal.h
    physics/ChLinkMotor.h
    physics/ChLinkMotorRotation.h
    physics/ChLinkMotorRotationAngle.h
    physics/ChLinkMotorRotationSpeed.h
    physics/ChLinkMotorRotationTorque.h
    physics/ChLinkMotorRotationDriveline.h
    physics/ChLinkMotorLinear.h
    physics/ChLinkMotorLinearPosition.h
    physics/ChLinkMotorLinearSpeed.h
    physics/ChLinkMotorLinearForce.h
    physics/ChLinkMotorLinearDriveline.h
	physics/ChLinkMotionImposed.h
)

source_group(physics\\links FILES
    ${ChronoEngine_physics_links_SOURCES}
    ${ChronoEngine_physics_links_HEADERS})

set(ChronoEngine_physics_loads_SOURCES
    physics/ChLoadContainer.cpp
    physics/ChLoad.cpp
    physics/ChLoadsBody.cpp
    physics/ChLoadsXYZnode.cpp
    physics/ChLoadBodyMesh.cpp
)

set(ChronoEngine_physics_loads_HEADERS
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

source_group(physics\\loads FILES
    ${ChronoEngine_physics_loads_SOURCES}
    ${ChronoEngine_physics_loads_HEADERS})

set(ChronoEngine_physics_shafts_SOURCES
    physics/ChShaft.cpp
    physics/ChShaftsGear.cpp
    physics/ChShaftsGearbox.cpp
    physics/ChShaftsGearboxAngled.cpp
    physics/ChShaftsClutch.cpp
    physics/ChShaftsPlanetary.cpp
    physics/ChShaftsMotor.cpp
    physics/ChShaftsMotorAngle.cpp
    physics/ChShaftsMotorSpeed.cpp
    physics/ChShaftsMotorTorque.cpp
    physics/ChShaftsBody.cpp
    physics/ChShaftsTorqueBase.cpp
    physics/ChShaftsTorque.cpp
    physics/ChShaftsTorsionSpring.cpp
    physics/ChShaftsTorqueConverter.cpp
    physics/ChShaftsThermalEngine.cpp
    physics/ChShaftsLoads.cpp
)

set(ChronoEngine_physics_shafts_HEADERS
    physics/ChShaft.h
    physics/ChShaftsBody.h
    physics/ChShaftsClutch.h
    physics/ChShaftsCouple.h
    physics/ChShaftsGear.h
    physics/ChShaftsGearbox.h
    physics/ChShaftsGearboxAngled.h
    physics/ChShaftsMotor.h
    physics/ChShaftsMotorAngle.h
    physics/ChShaftsMotorSpeed.h
    physics/ChShaftsMotorTorque.h
    physics/ChShaftsPlanetary.h
    physics/ChShaftsTorqueBase.h
    physics/ChShaftsTorque.h
    physics/ChShaftsTorsionSpring.h
    physics/ChShaftsTorqueConverter.h
    physics/ChShaftsThermalEngine.h
    physics/ChShaftsLoads.h
)

source_group(physics\\shafts FILES
    ${ChronoEngine_physics_shafts_SOURCES}
    ${ChronoEngine_physics_shafts_HEADERS})

# FEA group

set(ChronoEngine_fea_SOURCES
    fea/ChBuilderBeam.cpp
    fea/ChBeamSection.cpp
    fea/ChBeamSectionCosserat.cpp
    fea/ChContinuumMaterial.cpp
    fea/ChGaussIntegrationRule.cpp
    fea/ChGaussPoint.cpp
    fea/ChMesh.cpp
    fea/ChMeshFileLoader.cpp
    fea/ChMeshExporter.cpp
    fea/ChMatterMeshless.cpp
    fea/ChProximityContainerMeshless.cpp
    fea/ChPolarDecomposition.cpp
    fea/ChMatrixCorotation.cpp
    fea/ChVisualizationFEAmesh.cpp
    fea/ChLinkPointFrame.cpp
    fea/ChLinkDirFrame.cpp
    fea/ChLinkPointPoint.cpp
    fea/ChLinkPointTriface.cpp
    fea/ChLinkBeamIGAslider.cpp
    fea/ChContactSurface.cpp
    fea/ChContactSurfaceNodeCloud.cpp
    fea/ChContactSurfaceMesh.cpp
    fea/ChMeshSurface.cpp
    fea/ChLoadContactSurfaceMesh.cpp
    fea/ChMaterialShellANCF.cpp
    fea/ChMaterialShellReissner.cpp
	fea/ChMaterialShellKirchhoff.cpp
    )

set(ChronoEngine_fea_HEADERS
    fea/ChBeamSection.h
    fea/ChBeamSectionCosserat.h
    fea/ChBuilderBeam.h
    fea/ChContinuumMaterial.h
    fea/ChContinuumPoisson3D.h
    fea/ChContinuumThermal.h
    fea/ChContinuumElectrostatics.h
    fea/ChFaceTetra_4.h
    fea/ChFaceBrick_9.h
    fea/ChFaceHexa_8.h
    fea/ChLoadsBeam.h
    fea/ChGaussIntegrationRule.h
    fea/ChGaussPoint.h
    fea/ChMesh.h
    fea/ChMeshExporter.h
    fea/ChMeshFileLoader.h
    fea/ChMatterMeshless.h
    fea/ChProximityContainerMeshless.h
    fea/ChPolarDecomposition.h
    fea/ChMatrixCorotation.h
    fea/ChVisualizationFEAmesh.h
    fea/ChLinkPointFrame.h
    fea/ChLinkDirFrame.h
    fea/ChLinkPointPoint.h
    fea/ChLinkPointTriface.h
    fea/ChLinkBeamIGAslider.h
    fea/ChContactSurface.h
    fea/ChContactSurfaceNodeCloud.h
    fea/ChContactSurfaceMesh.h
    fea/ChMeshSurface.h
    fea/ChLoadContactSurfaceMesh.h
    fea/ChRotUtils.h
    fea/ChMaterialShellANCF.h
    fea/ChMaterialShellReissner.h
	fea/ChMaterialShellKirchhoff.h
)

source_group(fea FILES
    ${ChronoEngine_fea_SOURCES}
    ${ChronoEngine_fea_HEADERS})

# FEA sub-groups

set(ChronoEngine_fea_nodes_SOURCES
    fea/ChNodeFEAxyz.cpp
    fea/ChNodeFEAxyzrot.cpp
    fea/ChNodeFEAxyzP.cpp
    fea/ChNodeFEAxyzD.cpp
    fea/ChNodeFEAxyzDD.cpp
    fea/ChNodeFEAcurv.cpp
)

set(ChronoEngine_fea_nodes_HEADERS
    fea/ChNodeFEAbase.h 
    fea/ChNodeFEAxyz.h
    fea/ChNodeFEAxyzrot.h
    fea/ChNodeFEAxyzP.h
    fea/ChNodeFEAxyzD.h
    fea/ChNodeFEAxyzDD.h
    fea/ChNodeFEAcurv.h
)

source_group(fea\\nodes FILES
    ${ChronoEngine_fea_nodes_SOURCES}
    ${ChronoEngine_fea_nodes_HEADERS})

set(ChronoEngine_fea_elements_SOURCES
    fea/ChElementBeamANCF.cpp
    fea/ChElementBeamEuler.cpp
    fea/ChElementCableANCF.cpp
    fea/ChElementGeneric.cpp
    fea/ChElementSpring.cpp
    fea/ChElementBar.cpp
    fea/ChElementTetra_4.cpp
    fea/ChElementTetra_10.cpp
    fea/ChElementHexa_8.cpp
    fea/ChElementHexa_20.cpp 
    fea/ChElementShellANCF.cpp
    fea/ChElementShellANCF_8.cpp
    fea/ChElementShellReissner4.cpp
	fea/ChElementShellBST.cpp
    fea/ChElementBrick.cpp
    fea/ChElementBrick_9.cpp
)

set(ChronoEngine_fea_elements_HEADERS
    fea/ChElementBase.h
    fea/ChElementGeneric.h
    fea/ChElementCorotational.h
    fea/ChElementSpring.h
    fea/ChElementBar.h
    fea/ChElementBeam.h
    fea/ChElementBeamANCF.h
    fea/ChElementBeamEuler.h
    fea/ChElementBeamIGA.h
    fea/ChElementCableANCF.h
    fea/ChElementBrick.h 
    fea/ChElementBrick_9.h
    fea/ChElementTetrahedron.h
    fea/ChElementTetra_4.h
    fea/ChElementTetra_10.h
    fea/ChElementHexahedron.h
    fea/ChElementHexa_8.h
    fea/ChElementHexa_20.h
    fea/ChElementShell.h
    fea/ChElementShellANCF.h
    fea/ChElementShellANCF_8.h
    fea/ChElementShellReissner4.h
	fea/ChElementShellBST.h
)

source_group(fea\\elements FILES
    ${ChronoEngine_fea_elements_SOURCES}
    ${ChronoEngine_fea_elements_HEADERS})

# Geometry group

set(ChronoEngine_geometry_SOURCES
    geometry/ChGeometry.cpp
    geometry/ChSphere.cpp
    geometry/ChEllipsoid.cpp
    geometry/ChBox.cpp
    geometry/ChCone.cpp
    geometry/ChCylinder.cpp
    geometry/ChCapsule.cpp
    geometry/ChLine.cpp
    geometry/ChLineArc.cpp
    geometry/ChLineSegment.cpp
    geometry/ChLinePath.cpp
    geometry/ChLineBezier.cpp
    geometry/ChLineNurbs.cpp
    geometry/ChLineBspline.cpp
    geometry/ChLineCam.cpp
    geometry/ChLinePoly.cpp
    geometry/ChTriangle.cpp
    geometry/ChTriangleMeshSoup.cpp
    geometry/ChTriangleMeshConnected.cpp
    geometry/ChRoundedBox.cpp
    geometry/ChRoundedCylinder.cpp
    geometry/ChRoundedCone.cpp
    geometry/ChSurface.cpp
    geometry/ChSurfaceNurbs.cpp
    geometry/ChVolume.cpp
    )

set(ChronoEngine_geometry_HEADERS
    geometry/ChGeometry.h
    geometry/ChSphere.h
    geometry/ChEllipsoid.h
    geometry/ChBox.h
    geometry/ChCone.h
    geometry/ChCylinder.h
    geometry/ChCapsule.h
    geometry/ChLine.h
    geometry/ChLineCam.h
    geometry/ChLinePoly.h
    geometry/ChLineArc.h
    geometry/ChLineSegment.h
    geometry/ChLinePath.h
    geometry/ChLineBezier.h
    geometry/ChLineNurbs.h
    geometry/ChLineBspline.h
    geometry/ChTriangle.h
    geometry/ChTriangleMesh.h
    geometry/ChTriangleMeshSoup.h
    geometry/ChTriangleMeshConnected.h
    geometry/ChRoundedBox.h
    geometry/ChRoundedCylinder.h
    geometry/ChRoundedCone.h
    geometry/ChSurface.h
    geometry/ChSurfaceNurbs.h
    geometry/ChVolume.h
    geometry/ChBasisToolsBspline.h
    geometry/ChBasisToolsNurbs.h
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
	assets/ChBarrelShape.cpp
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
    assets/ChPointPointDrawing.cpp
    assets/ChSurfaceShape.cpp
    )

set(ChronoEngine_assets_HEADERS
    assets/ChAsset.h
    assets/ChVisualization.h
    assets/ChObjShapeFile.h
    assets/ChTriangleMeshShape.h
    assets/ChSphereShape.h
    assets/ChEllipsoidShape.h
	assets/ChBarrelShape.h
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
    assets/ChPointPointDrawing.h
    assets/ChSurfaceShape.h
    )

source_group(assets FILES
    ${ChronoEngine_assets_SOURCES}
    ${ChronoEngine_assets_HEADERS})

# Collision group

set(ChronoEngine_collision_SOURCES
    collision/ChCollisionInfo.cpp
    collision/ChCollisionShape.cpp
    collision/ChCollisionModel.cpp
    collision/ChCollisionModelBullet.cpp
    collision/ChCollisionSystemBullet.cpp
    collision/ChConvexDecomposition.cpp
    collision/ChCollisionUtils.cpp
    )

set(ChronoEngine_collision_HEADERS
    collision/ChCollisionInfo.h
    collision/ChCollisionShape.h
    collision/ChCollisionModel.h
    collision/ChCollisionPair.h
    collision/ChCollisionSystem.h
    collision/ChCollisionShapeBullet.h
    collision/ChCollisionModelBullet.h
    collision/ChCollisionSystemBullet.h
    collision/ChConvexDecomposition.h
    collision/ChCollisionUtils.h
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
    timestepper/ChIntegrable.cpp
    timestepper/ChTimestepper.cpp
    timestepper/ChTimestepperHHT.cpp
    timestepper/ChStaticAnalysis.cpp
    timestepper/ChAssemblyAnalysis.cpp
    )

set(ChronoEngine_timestepper_HEADERS
    timestepper/ChState.h
    timestepper/ChIntegrable.h
    timestepper/ChTimestepper.h
    timestepper/ChTimestepperHHT.h
    timestepper/ChStaticAnalysis.h
    timestepper/ChAssemblyAnalysis.h
    timestepper/ChUpdateFlags.h
    )

source_group(timestepper FILES
    ${ChronoEngine_timestepper_SOURCES}
    ${ChronoEngine_timestepper_HEADERS})

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
    motion_functions/ChFunction_Setpoint.cpp
	motion_functions/ChFunctionPosition.cpp
	motion_functions/ChFunctionPosition_XYZfunctions.cpp
	motion_functions/ChFunctionPosition_line.cpp
	motion_functions/ChFunctionPosition_setpoint.cpp
	motion_functions/ChFunctionRotation.cpp
	motion_functions/ChFunctionRotation_axis.cpp
	motion_functions/ChFunctionRotation_ABCfunctions.cpp
	motion_functions/ChFunctionRotation_setpoint.cpp
	motion_functions/ChFunctionRotation_spline.cpp
	motion_functions/ChFunctionRotation_SQUAD.cpp
    )

set(ChronoEngine_motion_functions_HEADERS
    motion_functions/ChFunction.h
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
    motion_functions/ChFunction_Setpoint.h
	motion_functions/ChFunctionPosition.h
	motion_functions/ChFunctionPosition_XYZfunctions.h
	motion_functions/ChFunctionPosition_line.h
	motion_functions/ChFunctionPosition_setpoint.h
	motion_functions/ChFunctionRotation.h
	motion_functions/ChFunctionRotation_axis.h
	motion_functions/ChFunctionRotation_ABCfunctions.h
	motion_functions/ChFunctionRotation_setpoint.h
	motion_functions/ChFunctionRotation_spline.h
	motion_functions/ChFunctionRotation_SQUAD.h
    )

if(CH_CXX14)
  set(ChronoEngine_motion_functions_HEADERS ${ChronoEngine_motion_functions_HEADERS}
      motion_functions/ChFunction_Lambda.h)
endif()

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

# Solver group

set(ChronoEngine_solver_SOURCES
    solver/ChSystemDescriptor.cpp
    solver/ChSolver.cpp
    solver/ChDirectSolverLS.cpp
    solver/ChIterativeSolver.cpp
    solver/ChIterativeSolverLS.cpp
    solver/ChIterativeSolverVI.cpp
    solver/ChSolverPSOR.cpp
    solver/ChSolverPJacobi.cpp
    solver/ChSolverPSSOR.cpp
    solver/ChSolverPMINRES.cpp
    solver/ChSolverBB.cpp
    solver/ChSolverAPGD.cpp
    solver/ChKblockGeneric.cpp
    solver/ChSolvmin.cpp
    solver/ChNlsolver.cpp
    )

set(ChronoEngine_solver_HEADERS
    solver/ChSystemDescriptor.h
    solver/ChSolver.h
    solver/ChSolverLS.h
    solver/ChSolverVI.h
    solver/ChDirectSolverLS.h
    solver/ChIterativeSolver.h
    solver/ChIterativeSolverLS.h
    solver/ChIterativeSolverVI.h
    solver/ChSolverPJacobi.h
    solver/ChSolverPMINRES.h
    solver/ChSolverBB.h
    solver/ChSolverAPGD.h
    solver/ChSolverPSOR.h
    solver/ChSolverPSSOR.h
    solver/ChKblock.h
    solver/ChKblockGeneric.h
    solver/ChSolvmin.h
    solver/ChNlsolver.h
    )

source_group(solver FILES
    ${ChronoEngine_solver_SOURCES}
    ${ChronoEngine_solver_HEADERS})

#Solver sub-groups

set(ChronoEngine_solver_variables_SOURCES
    solver/ChVariables.cpp
    solver/ChVariablesGeneric.cpp
    solver/ChVariablesGenericDiagonalMass.cpp
    solver/ChVariablesBody.cpp
    solver/ChVariablesBodySharedMass.cpp
    solver/ChVariablesBodyOwnMass.cpp
    solver/ChVariablesShaft.cpp
    solver/ChVariablesNode.cpp
)

set(ChronoEngine_solver_variables_HEADERS
    solver/ChVariables.h
    solver/ChVariablesBody.h
    solver/ChVariablesBodyOwnMass.h
    solver/ChVariablesBodySharedMass.h
    solver/ChVariablesShaft.h
    solver/ChVariablesGeneric.h
    solver/ChVariablesGenericDiagonalMass.h
    solver/ChVariablesNode.h
)

source_group(solver\\variables FILES
    ${ChronoEngine_solver_variables_SOURCES}
    ${ChronoEngine_solver_variables_HEADERS})

set(ChronoEngine_solver_constraints_SOURCES
    solver/ChConstraint.cpp
    solver/ChConstraintTwo.cpp
    solver/ChConstraintTwoGeneric.cpp
    solver/ChConstraintTwoGenericBoxed.cpp
    solver/ChConstraintTwoBodies.cpp
    solver/ChConstraintThree.cpp
    solver/ChConstraintThreeGeneric.cpp
    solver/ChConstraintThreeBBShaft.cpp
    solver/ChConstraintNgeneric.cpp
)

set(ChronoEngine_solver_constraints_HEADERS
    solver/ChConstraint.h
    solver/ChConstraintThree.h
    solver/ChConstraintThreeBBShaft.h
    solver/ChConstraintThreeGeneric.h
    solver/ChConstraintTwo.h
    solver/ChConstraintTwoBodies.h
    solver/ChConstraintTwoGeneric.h
    solver/ChConstraintTwoGenericBoxed.h
    solver/ChConstraintTuple.h
    solver/ChConstraintTwoTuples.h
    solver/ChConstraintTwoTuplesContactN.h
    solver/ChConstraintTwoTuplesFrictionT.h
    solver/ChConstraintTwoTuplesRollingN.h
    solver/ChConstraintTwoTuplesRollingT.h
    solver/ChConstraintNgeneric.h
)

source_group(solver\\constraints FILES
    ${ChronoEngine_solver_constraints_SOURCES}
    ${ChronoEngine_solver_constraints_HEADERS})

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

# HACD group

set(ChronoEngine_collision_convexdecompHACD_SOURCES
    ../chrono_thirdparty/HACD/hacdRaycastMesh.h
    ../chrono_thirdparty/HACD/hacdMicroAllocator.h
    ../chrono_thirdparty/HACD/hacdGraph.h
    ../chrono_thirdparty/HACD/hacdMeshDecimator.h
    ../chrono_thirdparty/HACD/hacdHACD.h
    ../chrono_thirdparty/HACD/hacdManifoldMesh.h
    ../chrono_thirdparty/HACD/hacdICHull.h
    ../chrono_thirdparty/HACD/hacdSArray.h
    ../chrono_thirdparty/HACD/hacdVector.h
    ../chrono_thirdparty/HACD/hacdVersion.h
#
    ../chrono_thirdparty/HACD/hacdRaycastMesh.cpp
    ../chrono_thirdparty/HACD/hacdMicroAllocator.cpp
    ../chrono_thirdparty/HACD/hacdGraph.cpp
    ../chrono_thirdparty/HACD/hacdMeshDecimator.cpp
    ../chrono_thirdparty/HACD/hacdHACD.cpp
    ../chrono_thirdparty/HACD/hacdManifoldMesh.cpp
    ../chrono_thirdparty/HACD/hacdICHull.cpp
    )

source_group(collision\\convexdecomp\\HACD FILES
    ${ChronoEngine_collision_convexdecompHACD_SOURCES})

# HACDv2 group

set(ChronoEngine_collision_convexdecompHACDv2_SOURCES
    ../chrono_thirdparty/HACDv2/AutoGeometry.h
    ../chrono_thirdparty/HACDv2/HACD.h
    ../chrono_thirdparty/HACDv2/ConvexHull.h
    ../chrono_thirdparty/HACDv2/WuQuantizer.h
    ../chrono_thirdparty/HACDv2/dgTypes.h
    ../chrono_thirdparty/HACDv2/dgRefCounter.h
    ../chrono_thirdparty/HACDv2/dgConvexHull3d.h
    ../chrono_thirdparty/HACDv2/dgHeap.h
    ../chrono_thirdparty/HACDv2/dgStack.h
    ../chrono_thirdparty/HACDv2/dgList.h
    ../chrono_thirdparty/HACDv2/dgTree.h
    ../chrono_thirdparty/HACDv2/dgGoogol.h
    ../chrono_thirdparty/HACDv2/dgSphere.h
    ../chrono_thirdparty/HACDv2/dgSmallDeterminant.h
    ../chrono_thirdparty/HACDv2/dgQuaternion.h
    ../chrono_thirdparty/HACDv2/dgArray.h
    ../chrono_thirdparty/HACDv2/dgVector.h
    ../chrono_thirdparty/HACDv2/dgMatrix.h
    ../chrono_thirdparty/HACDv2/dgPolyhedra.h
    ../chrono_thirdparty/HACDv2/dgMeshEffect.h
    ../chrono_thirdparty/HACDv2/MergeHulls.h
    ../chrono_thirdparty/HACDv2/wavefront.h
#
    ../chrono_thirdparty/HACDv2/AutoGeometry.cpp
    ../chrono_thirdparty/HACDv2/HACD.cpp
    ../chrono_thirdparty/HACDv2/ConvexHull.cpp
    ../chrono_thirdparty/HACDv2/WuQuantizer.cpp
    ../chrono_thirdparty/HACDv2/dgTypes.cpp
    ../chrono_thirdparty/HACDv2/dgConvexHull3d.cpp
    ../chrono_thirdparty/HACDv2/dgTree.cpp
    ../chrono_thirdparty/HACDv2/dgGoogol.cpp
    ../chrono_thirdparty/HACDv2/dgSphere.cpp
    ../chrono_thirdparty/HACDv2/dgSmallDeterminant.cpp
    ../chrono_thirdparty/HACDv2/dgQuaternion.cpp
    ../chrono_thirdparty/HACDv2/dgMatrix.cpp
    ../chrono_thirdparty/HACDv2/dgPolyhedra.cpp
    ../chrono_thirdparty/HACDv2/dgMeshEffect.cpp
    ../chrono_thirdparty/HACDv2/MergeHulls.cpp
    ../chrono_thirdparty/HACDv2/wavefront.cpp
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
    utils/ChCompositeInertia.cpp
    utils/ChParserOpenSim.cpp
    utils/ChParserAdams.cpp
    utils/ChAdamsTokenizer.yy.cpp
    utils/ChConvexHull.cpp
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
    utils/ChCompositeInertia.h
    utils/ChParserOpenSim.h
    utils/ChParserAdams.h
    utils/ChConvexHull.h
)

if(BUILD_BENCHMARKING)
    set(ChronoEngine_utils_HEADERS ${ChronoEngine_utils_HEADERS} utils/ChBenchmark.h)
endif()

source_group(utils FILES
    ${ChronoEngine_utils_SOURCES}
    ${ChronoEngine_utils_HEADERS})

# TinyObj Group

set(ChronoEngine_tiny_obj_FILES
  ../chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc
  ../chrono_thirdparty/tinyobjloader/tiny_obj_loader.h
  )

source_group(utils\\tiny_obj FILES
    ${ChronoEngine_tiny_obj_FILES})

# Collect all source and header files for the ChronoEngine library

set(ChronoEngine_FILES
    ${ChronoEngine_core_SOURCES}
    ${ChronoEngine_core_HEADERS}
    ${ChronoEngine_serialization_SOURCES}
    ${ChronoEngine_serialization_HEADERS}
    ${ChronoEngine_physics_SOURCES}
    ${ChronoEngine_physics_HEADERS}
    ${ChronoEngine_physics_contact_SOURCES}
    ${ChronoEngine_physics_contact_HEADERS}
    ${ChronoEngine_physics_links_SOURCES}
    ${ChronoEngine_physics_links_HEADERS}
    ${ChronoEngine_physics_loads_SOURCES}
    ${ChronoEngine_physics_loads_HEADERS}
    ${ChronoEngine_physics_shafts_SOURCES}
    ${ChronoEngine_physics_shafts_HEADERS}
    ${ChronoEngine_fea_SOURCES}
    ${ChronoEngine_fea_HEADERS}
    ${ChronoEngine_fea_nodes_SOURCES}
    ${ChronoEngine_fea_nodes_HEADERS}
    ${ChronoEngine_fea_elements_SOURCES}
    ${ChronoEngine_fea_elements_HEADERS}
    ${ChronoEngine_solver_SOURCES}
    ${ChronoEngine_solver_HEADERS}
    ${ChronoEngine_solver_constraints_SOURCES}
    ${ChronoEngine_solver_constraints_HEADERS}
    ${ChronoEngine_solver_variables_SOURCES}
    ${ChronoEngine_solver_variables_HEADERS}
    ${ChronoEngine_collision_SOURCES}
    ${ChronoEngine_collision_HEADERS}
    ${ChronoEngine_geometry_SOURCES}
    ${ChronoEngine_geometry_HEADERS}
    ${ChronoEngine_assets_SOURCES}
    ${ChronoEngine_assets_HEADERS}
    ${ChronoEngine_collision_edgetempest_SOURCES}
    ${ChronoEngine_collision_edgetempest_HEADERS}
    ${ChronoEngine_collision_convexdecompHACD_SOURCES}
    ${ChronoEngine_collision_convexdecompHACDv2_SOURCES}
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
    ${ChronoEngine_tiny_obj_FILES}
    )

# Add the ChronoEngine library to the project
add_library(ChronoEngine SHARED ${ChronoEngine_FILES})

if (UNIX)
  target_link_libraries(ChronoEngine pthread)
endif()

# Set some custom properties of this target
set_target_properties(ChronoEngine PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine PRIVATE "CH_API_COMPILE")
target_compile_definitions(ChronoEngine PRIVATE "CH_IGNORE_DEPRECATED")

if (USE_BULLET_DOUBLE)
    target_compile_definitions(ChronoEngine PRIVATE "BT_USE_DOUBLE_PRECISION")
endif()

if(XCODE_VERSION)
    set_target_properties(ChronoEngine PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT ${ENABLE_OPENMP})
endif()
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if(MSVC AND MSVC_VERSION GREATER_EQUAL 1900)
	    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	endif()
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
