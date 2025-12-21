#=============================================================================
#
# CMake configuration file for the core Chrono library.
#
#=============================================================================

message(STATUS "\n==== Chrono Engine core module ====\n")

# -------------------------------------------------------------------------------------------------
# Double precision support for Bullet
# -------------------------------------------------------------------------------------------------

option(CH_USE_BULLET_DOUBLE "Compile Chrono with double precision support in Bullet" OFF)
cmake_dependent_option(CH_USE_BULLET_OPENMP "Compile Chrono with OpenMP support in Bullet" ON "CH_ENABLE_OPENMP" OFF)

# -------------------------------------------------------------------------------------------------
# Collect files in the core Chrono library
# -------------------------------------------------------------------------------------------------

# Core group

set(Chrono_core_SOURCES
    core/ChClassFactory.cpp
    core/ChQuaternion.cpp
    core/ChRotation.cpp
    core/ChVector3.cpp
    core/ChCoordsys.cpp
    core/ChQuadrature.cpp
    core/ChBezierCurve.cpp
    core/ChCubicSpline.cpp
    core/ChRandom.cpp
    core/ChDataPath.cpp
    )
set(Chrono_core_HEADERS
    core/ChApiCE.h
    core/ChChrono.h
    core/ChClassFactory.h
    core/ChCoordsys.h
    core/ChFrame.h
    core/ChFrameMoving.h
    core/ChMatrix.h
    core/ChMatrixEigenExtensions.h
    core/ChSparseMatrixEigenExtensions.h
    core/ChSparsityPatternLearner.h
    core/ChMatrix33.h
    core/ChMatrixMBD.h
    core/ChPlatform.h
    core/ChQuaternion.h
    core/ChRotation.h
    core/ChRealtimeStep.h
    core/ChTimer.h
    core/ChVector3.h
    core/ChVector2.h
    core/ChAlignedAllocator.h
    core/ChRandom.h
    core/ChQuadrature.h
    core/ChTemplateExpressions.h
    core/ChBezierCurve.h
    core/ChCubicSpline.h
    core/ChDataPath.h
    core/ChTypes.h
    core/ChTensors.h
    )
source_group(core FILES ${Chrono_core_SOURCES} ${Chrono_core_HEADERS})

# Serialization group

set(Chrono_serialization_SOURCES
    serialization/ChArchive.cpp
    serialization/ChArchiveBinary.cpp
    serialization/ChObjectExplorer.cpp
    serialization/ChArchiveJSON.cpp
    serialization/ChArchiveXML.cpp
    serialization/ChArchiveASCII.cpp
    )
set(Chrono_serialization_HEADERS
    serialization/ChArchive.h
    serialization/ChArchiveBinary.h
    serialization/ChObjectExplorer.h
    serialization/ChArchiveJSON.h
    serialization/ChArchiveXML.h
    serialization/ChArchiveASCII.h
    )
source_group(serialization FILES ${Chrono_serialization_SOURCES} ${Chrono_serialization_HEADERS})

# Physics group

set(Chrono_physics_SOURCES
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
    physics/ChPhysicsItem.cpp
    physics/ChParticleCloud.cpp
    physics/ChIndexedParticles.cpp
    physics/ChIndexedNodes.cpp
    physics/ChNodeBase.cpp
    physics/ChNodeXYZ.cpp
    physics/ChProximityContainer.cpp
    physics/ChConveyor.cpp
    physics/ChFeeder.cpp
    physics/ChExternalDynamicsODE.cpp
    physics/ChExternalDynamicsDAE.cpp
    physics/ChAssembly.cpp
    physics/ChMassProperties.cpp
    )
set(Chrono_physics_HEADERS
    physics/ChBodyFrame.h
    physics/ChBody.h
    physics/ChBodyAuxRef.h
    physics/ChBodyEasy.h
    physics/ChConveyor.h
    physics/ChFeeder.h
    physics/ChForce.h
    physics/ChIndexedNodes.h
    physics/ChIndexedParticles.h
    physics/ChMarker.h
    physics/ChNodeBase.h
    physics/ChNodeXYZ.h
    physics/ChObject.h
    physics/ChParticleCloud.h
    physics/ChPhysicsItem.h
    physics/ChProximityContainer.h
    physics/ChSystem.h
    physics/ChSystemNSC.h
    physics/ChSystemSMC.h
    physics/ChExternalDynamicsODE.h
    physics/ChExternalDynamicsDAE.h
    physics/ChAssembly.h
    physics/ChMassProperties.h
    )
source_group(physics FILES ${Chrono_physics_SOURCES} ${Chrono_physics_HEADERS})

# PHYSICS sub-groups

set(Chrono_physics_contact_SOURCES
    physics/ChContactContainer.cpp
    physics/ChContactContainerNSC.cpp
    physics/ChContactContainerSMC.cpp
    physics/ChContactable.cpp
    physics/ChContact.cpp
    physics/ChContactSMC.cpp
    physics/ChContactNSC.cpp
    physics/ChContactNSCrolling.cpp
    physics/ChContactMaterial.cpp
    physics/ChContactMaterialSMC.cpp
    physics/ChContactMaterialNSC.cpp
    )
set(Chrono_physics_contact_HEADERS
    physics/ChContactContainer.h
    physics/ChContactContainerNSC.h
    physics/ChContactContainerSMC.h
    physics/ChContactable.h
    physics/ChContact.h
    physics/ChContactSMC.h
    physics/ChContactNSC.h
    physics/ChContactNSCrolling.h
    physics/ChContactMaterial.h
    physics/ChContactMaterialNSC.h
    physics/ChContactMaterialSMC.h
    )
source_group(physics\\contact FILES ${Chrono_physics_contact_SOURCES} ${Chrono_physics_contact_HEADERS})

set(Chrono_physics_links_SOURCES
    physics/ChLinkLimit.cpp
    physics/ChLinkForce.cpp
    physics/ChLinkMask.cpp
    physics/ChLinkBase.cpp
    physics/ChLink.cpp
    physics/ChLinkDistance.cpp
    physics/ChLinkMarkers.cpp
    physics/ChLinkBushing.cpp
    physics/ChLinkLock.cpp
    physics/ChLinkLockBrake.cpp
    physics/ChLinkLockGear.cpp
    physics/ChLinkLockPulley.cpp
    physics/ChLinkLockLinActuator.cpp
    physics/ChLinkLockScrew.cpp
    physics/ChLinkLockClearance.cpp
    physics/ChLinkLockPointSpline.cpp
    physics/ChLinkLockTrajectory.cpp
    physics/ChLinkMate.cpp
    physics/ChLinkRevolute.cpp
    physics/ChLinkRevoluteSpherical.cpp
    physics/ChLinkRevoluteTranslational.cpp
    physics/ChLinkUniversal.cpp
    physics/ChLinkRSDA.cpp
    physics/ChLinkTSDA.cpp
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
    physics/ChJoint.cpp
    )
set(Chrono_physics_links_HEADERS
    physics/ChLinkLimit.h
    physics/ChLinkBase.h
    physics/ChLink.h
    physics/ChLinksAll.h
    physics/ChLinkBushing.h
    physics/ChLinkDistance.h
    physics/ChLinkForce.h
    physics/ChLinkMarkers.h
    physics/ChLinkMask.h
    physics/ChLinkLockBrake.h
    physics/ChLinkLockGear.h
    physics/ChLinkLockLinActuator.h
    physics/ChLinkLock.h
    physics/ChLinkLockPointSpline.h
    physics/ChLinkLockPulley.h
    physics/ChLinkLockScrew.h
    physics/ChLinkLockTrajectory.h
    physics/ChLinkLockClearance.h
    physics/ChLinkMate.h
    physics/ChLinkRevolute.h
    physics/ChLinkRevoluteSpherical.h
    physics/ChLinkRevoluteTranslational.h
    physics/ChLinkUniversal.h
    physics/ChLinkRSDA.h
    physics/ChLinkTSDA.h
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
    physics/ChLinkMotorAll.h
    physics/ChJoint.h
    )
source_group(physics\\links FILES ${Chrono_physics_links_SOURCES} ${Chrono_physics_links_HEADERS})

set(Chrono_physics_loads_SOURCES
    physics/ChLoaderU.cpp
    physics/ChLoaderUV.cpp
    physics/ChLoaderUVW.cpp
    physics/ChLoad.cpp
    physics/ChLoadsBody.cpp
    physics/ChLoadsNodeXYZ.cpp
    physics/ChLoadBodyMesh.cpp
    physics/ChLoadHydrodynamics.cpp
    physics/ChLoadContainer.cpp
    )
set(Chrono_physics_loads_HEADERS
    physics/ChLoadable.h
    physics/ChLoader.h
    physics/ChLoaderU.h
    physics/ChLoaderUV.h
    physics/ChLoaderUVW.h
    physics/ChLoad.h
    physics/ChLoadsNodeXYZ.h
    physics/ChLoadsBody.h
    physics/ChLoadBodyMesh.h
    physics/ChLoadHydrodynamics.h
    physics/ChLoadContainer.h
    )
source_group(physics\\loads FILES ${Chrono_physics_loads_SOURCES} ${Chrono_physics_loads_HEADERS})

set(Chrono_physics_hydraulics_SOURCES
    physics/ChHydraulicCircuit.cpp
    physics/ChHydraulicActuator.cpp
    )
set(Chrono_physics_hydraulics_HEADERS
    physics/ChHydraulicCircuit.h
    physics/ChHydraulicActuator.h
    )
source_group(physics\\hydraulics FILES ${Chrono_physics_hydraulics_SOURCES} ${Chrono_physics_hydraulics_HEADERS})

set(Chrono_physics_shafts_SOURCES
    physics/ChShaft.cpp
    physics/ChShaftBodyConstraint.cpp
    physics/ChShaftsGear.cpp
    physics/ChShaftsGearbox.cpp
    physics/ChShaftsGearboxAngled.cpp
    physics/ChShaftsClutch.cpp
    physics/ChShaftsPlanetary.cpp
    physics/ChShaftsMotor.cpp
    physics/ChShaftsMotorPosition.cpp
    physics/ChShaftsMotorSpeed.cpp
    physics/ChShaftsMotorLoad.cpp
    physics/ChShaftsTorque.cpp
    physics/ChShaftsAppliedTorque.cpp
    physics/ChShaftsTorsionSpring.cpp
    physics/ChShaftsTorqueConverter.cpp
    physics/ChShaftsThermalEngine.cpp
    physics/ChShaftsLoads.cpp
    physics/ChShaftsFreewheel.cpp
    )
set(Chrono_physics_shafts_HEADERS
    physics/ChShaft.h
    physics/ChShaftBodyConstraint.h
    physics/ChShaftsClutch.h
    physics/ChShaftsCouple.h
    physics/ChShaftsGear.h
    physics/ChShaftsGearbox.h
    physics/ChShaftsGearboxAngled.h
    physics/ChShaftsMotor.h
    physics/ChShaftsMotorPosition.h
    physics/ChShaftsMotorSpeed.h
    physics/ChShaftsMotorLoad.h
    physics/ChShaftsPlanetary.h
    physics/ChShaftsTorque.h
    physics/ChShaftsAppliedTorque.h
    physics/ChShaftsTorsionSpring.h
    physics/ChShaftsTorqueConverter.h
    physics/ChShaftsThermalEngine.h
    physics/ChShaftsLoads.h
    physics/ChShaftsFreewheel.h
    )
source_group(physics\\shafts FILES ${Chrono_physics_shafts_SOURCES} ${Chrono_physics_shafts_HEADERS})

# FEA group

set(Chrono_fea_SOURCES
    fea/ChBuilderBeam.cpp
    fea/ChGaussIntegrationRule.cpp
    fea/ChGaussPoint.cpp
    fea/ChMesh.cpp
    fea/ChMeshFileLoader.cpp
    fea/ChMeshExporter.cpp
    fea/ChPolarDecomposition.cpp
    fea/ChMatrixCorotation.cpp
    fea/ChContactSurface.cpp
    fea/ChContactSurfaceSegmentSet.cpp
    fea/ChContactSurfaceNodeCloud.cpp
    fea/ChContactSurfaceMesh.cpp
    fea/ChMeshSurface.cpp
    fea/ChLoadContactSurfaceMesh.cpp
    fea/ChLoadsNodeXYZRot.cpp
    )
set(Chrono_fea_HEADERS
    fea/ChBuilderBeam.h
    fea/ChHexahedronFace.h
    fea/ChTetrahedronFace.h
    fea/ChLoadsBeam.h
    fea/ChGaussIntegrationRule.h
    fea/ChGaussPoint.h
    fea/ChMesh.h
    fea/ChMeshExporter.h
    fea/ChMeshFileLoader.h
    fea/ChPolarDecomposition.h
    fea/ChMatrixCorotation.h
    fea/ChContactSurface.h
    fea/ChContactSurfaceSegmentSet.h
    fea/ChContactSurfaceNodeCloud.h
    fea/ChContactSurfaceMesh.h
    fea/ChMeshSurface.h
    fea/ChLoadContactSurfaceMesh.h
    fea/ChLoadsNodeXYZRot.h
    fea/ChRotUtils.h
    )
source_group(fea FILES ${Chrono_fea_SOURCES} ${Chrono_fea_HEADERS})

# FEA sub-groups

set(Chrono_fea_nodes_SOURCES
    fea/ChNodeFEAxyz.cpp
    fea/ChNodeFEAxyzrot.cpp
    fea/ChNodeFEAxyzP.cpp
    fea/ChNodeFEAxyzD.cpp
    fea/ChNodeFEAxyzDD.cpp
    fea/ChNodeFEAxyzDDD.cpp
    fea/ChNodeFEAcurv.cpp
    )
set(Chrono_fea_nodes_HEADERS
    fea/ChNodeFEAbase.h
    fea/ChNodeFEAxyz.h
    fea/ChNodeFEAxyzrot.h
    fea/ChNodeFEAxyzP.h
    fea/ChNodeFEAxyzD.h
    fea/ChNodeFEAxyzDD.h
    fea/ChNodeFEAxyzDDD.h
    fea/ChNodeFEAcurv.h
    )
source_group(fea\\nodes FILES ${Chrono_fea_nodes_SOURCES} ${Chrono_fea_nodes_HEADERS})

set(Chrono_fea_elements_SOURCES
    fea/ChElementBeamEuler.cpp
    fea/ChElementBeamTaperedTimoshenko.cpp
    fea/ChElementBeamTaperedTimoshenkoFPM.cpp
    fea/ChElementBeamIGA.cpp
    fea/ChElementCableANCF.cpp
    fea/ChElementGeneric.cpp
    fea/ChElementSpring.cpp
    fea/ChElementBar.cpp
    fea/ChElementTetraCorot_4.cpp
    fea/ChElementTetraCorot_10.cpp
    fea/ChElementHexaCorot_8.cpp
    fea/ChElementHexaCorot_20.cpp
    fea/ChElementHexaANCF_3813.cpp
    fea/ChElementHexaANCF_3813_9.cpp
    fea/ChElementHexaANCF_3843.cpp
    fea/ChElementBeamANCF_3243.cpp
    fea/ChElementBeamANCF_3333.cpp
    fea/ChElementShellANCF_3423.cpp
    fea/ChElementShellANCF_3833.cpp
    fea/ChElementShellANCF_3443.cpp
    fea/ChElementShellBST.cpp
    fea/ChElementShellReissner4.cpp
    )
set(Chrono_fea_elements_HEADERS
    fea/ChElementBase.h
    fea/ChElementGeneric.h
    fea/ChElementCorotational.h
    fea/ChElementANCF.h
    fea/ChElementSpring.h
    fea/ChElementBar.h
    fea/ChElementBeam.h
    fea/ChElementBeamANCF_3243.h
    fea/ChElementBeamANCF_3333.h
    fea/ChElementBeamEuler.h
    fea/ChElementBeamTaperedTimoshenko.h
    fea/ChElementBeamTaperedTimoshenkoFPM.h
    fea/ChElementBeamIGA.h
    fea/ChElementCableANCF.h
    fea/ChElementTetrahedron.h
    fea/ChElementTetraCorot_4.h
    fea/ChElementTetraCorot_10.h
    fea/ChElementHexahedron.h
    fea/ChElementHexaANCF_3813.h
    fea/ChElementHexaANCF_3813_9.h
    fea/ChElementHexaCorot_8.h
    fea/ChElementHexaCorot_20.h
    fea/ChElementHexaANCF_3843.h
    fea/ChElementShell.h
    fea/ChElementShellANCF_3423.h
    fea/ChElementShellANCF_3443.h
    fea/ChElementShellANCF_3833.h
    fea/ChElementShellReissner4.h
    fea/ChElementShellBST.h
    )
source_group(fea\\elements FILES ${Chrono_fea_elements_SOURCES} ${Chrono_fea_elements_HEADERS})

set(Chrono_fea_materials_SOURCES
    fea/ChContinuumMaterial.cpp
    fea/ChMaterialBeamANCF.cpp
    fea/ChMaterialShellANCF.cpp
    fea/ChMaterialHexaANCF.cpp
    fea/ChMaterialShellReissner.cpp
    fea/ChMaterialShellKirchhoff.cpp
    )
set(Chrono_fea_materials_HEADERS
    fea/ChContinuumMaterial.h
    fea/ChContinuumPoisson3D.h
    fea/ChContinuumThermal.h
    fea/ChContinuumElectrostatics.h
    fea/ChMaterialBeamANCF.h
    fea/ChMaterialShellANCF.h
    fea/ChMaterialHexaANCF.h
    fea/ChMaterialShellReissner.h
    fea/ChMaterialShellKirchhoff.h
    )
source_group(fea\\materials FILES ${Chrono_fea_materials_SOURCES} ${Chrono_fea_materials_HEADERS})

set(Chrono_fea_sections_SOURCES
    fea/ChBeamSection.cpp
    fea/ChBeamSectionShape.cpp
    fea/ChBeamSectionCosserat.cpp
    fea/ChBeamSectionEuler.cpp
    fea/ChBeamSectionTaperedTimoshenko.cpp
    fea/ChBeamSectionTaperedTimoshenkoFPM.cpp
    fea/ChBeamSectionCable.cpp
    )
set(Chrono_fea_sections_HEADERS
    fea/ChBeamSection.h
    fea/ChBeamSectionShape.h
    fea/ChBeamSectionCosserat.h
    fea/ChBeamSectionEuler.h
    fea/ChBeamSectionTaperedTimoshenko.h
    fea/ChBeamSectionTaperedTimoshenkoFPM.h
    fea/ChBeamSectionCable.h
    )
source_group(fea\\sections FILES ${Chrono_fea_sections_SOURCES} ${Chrono_fea_sections_HEADERS})

set(Chrono_fea_constraints_SOURCES
    fea/ChLinkNodeFrame.cpp
    fea/ChLinkNodeSlopeFrame.cpp
    fea/ChLinkNodeNode.cpp
    fea/ChLinkNodeFace.cpp
    fea/ChLinkBeamIGAFrame.cpp
    )
set(Chrono_fea_constraints_HEADERS
    fea/ChLinkNodeFrame.h
    fea/ChLinkNodeSlopeFrame.h
    fea/ChLinkNodeNode.h
    fea/ChLinkNodeFace.h
    fea/ChLinkBeamIGAFrame.h
    )
source_group(fea\\constraints FILES ${Chrono_fea_constraints_SOURCES} ${Chrono_fea_constraints_HEADERS})

# Geometry group

set(Chrono_geometry_SOURCES
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
    geometry/ChLineBSpline.cpp
    geometry/ChLineCam.cpp
    geometry/ChLinePoly.cpp
    geometry/ChTriangle.cpp
    geometry/ChTriangleMesh.cpp
    geometry/ChTriangleMeshSoup.cpp
    geometry/ChTriangleMeshConnected.cpp
    geometry/ChRoundedBox.cpp
    geometry/ChRoundedCylinder.cpp
    geometry/ChSurface.cpp
    geometry/ChSurfaceNurbs.cpp
    geometry/ChVolume.cpp
    geometry/ChProperty.cpp
    )
set(Chrono_geometry_HEADERS
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
    geometry/ChLineBSpline.h
    geometry/ChTriangle.h
    geometry/ChTriangleMesh.h
    geometry/ChTriangleMeshSoup.h
    geometry/ChTriangleMeshConnected.h
    geometry/ChRoundedBox.h
    geometry/ChRoundedCylinder.h
    geometry/ChSurface.h
    geometry/ChSurfaceNurbs.h
    geometry/ChVolume.h
    geometry/ChBasisToolsBSpline.h
    geometry/ChBasisToolsNurbs.h
    geometry/ChProperty.h
    )
source_group(geometry FILES ${Chrono_geometry_SOURCES} ${Chrono_geometry_HEADERS})

# Asset group

set(Chrono_assets_SOURCES
    assets/ChTexture.cpp
    assets/ChCamera.cpp
    assets/ChColor.cpp
    assets/ChColormap.cpp
    assets/ChGlyphs.cpp
    assets/ChVisualSystem.cpp
    assets/ChVisualModel.cpp
    assets/ChVisualMaterial.cpp
    assets/ChVisualShape.cpp
    assets/ChVisualShapeModelFile.cpp
    assets/ChVisualShapeTriangleMesh.cpp
    assets/ChVisualShapeSphere.cpp
    assets/ChVisualShapeEllipsoid.cpp
    assets/ChVisualShapeBox.cpp
    assets/ChVisualShapeBarrel.cpp
    assets/ChVisualShapeCone.cpp
    assets/ChVisualShapeCylinder.cpp
    assets/ChVisualShapeCapsule.cpp
    assets/ChVisualShapeRoundedCylinder.cpp
    assets/ChVisualShapeRoundedBox.cpp
    assets/ChVisualShapePath.cpp
    assets/ChVisualShapeLine.cpp
    assets/ChVisualShapePointPoint.cpp
    assets/ChVisualShapeSurface.cpp
    assets/ChVisualShapeFEA.cpp
    )
set(Chrono_assets_HEADERS
    assets/ChTexture.h
    assets/ChCamera.h
    assets/ChColor.h
    assets/ChColormap.h
    assets/ChGlyphs.h
    assets/ChVisualSystem.h
    assets/ChVisualModel.h
    assets/ChVisualMaterial.h
    assets/ChVisualShape.h
    assets/ChVisualShapes.h
    assets/ChVisualShapeModelFile.h
    assets/ChVisualShapeTriangleMesh.h
    assets/ChVisualShapeSphere.h
    assets/ChVisualShapeEllipsoid.h
    assets/ChVisualShapeBarrel.h
    assets/ChVisualShapeBox.h
    assets/ChVisualShapeCone.h
    assets/ChVisualShapeCylinder.h
    assets/ChVisualShapeCapsule.h
    assets/ChVisualShapeRoundedCylinder.h
    assets/ChVisualShapeRoundedBox.h
    assets/ChVisualShapePath.h
    assets/ChVisualShapeLine.h
    assets/ChVisualShapePointPoint.h
    assets/ChVisualShapeSurface.h
    assets/ChVisualShapeFEA.h
    )
source_group(assets FILES ${Chrono_assets_SOURCES} ${Chrono_assets_HEADERS})


# Multicore types and math 

if (CHRONO_THRUST_FOUND)
    set(Chrono_MulticoreMath_SOURCES
        multicore_math/matrix.cpp
        multicore_math/real2.cpp
        multicore_math/real3.cpp
        multicore_math/real4.cpp
        multicore_math/vec3.cpp
        )
    set(Chrono_MulticoreMath_HEADERS
        multicore_math/matrix.h
        multicore_math/real.h
        multicore_math/real_double.h
        multicore_math/real_single.h
        multicore_math/real2.h
        multicore_math/real3.h
        multicore_math/real4.h
        multicore_math/simd_avx.h
        multicore_math/simd_non.h
        multicore_math/simd_sse.h
        multicore_math/simd.h
        multicore_math/thrust.h
        multicore_math/types.h
        multicore_math/utility.h
    )
    source_group(multicore_math FILES ${Chrono_MulticoreMath_SOURCES} ${Chrono_MulticoreMath_HEADERS})
else()
    set(Chrono_MulticoreMath_SOURCES "")
    set(Chrono_MulticoreMath_HEADERS "")
endif()

# Collision group

set(Chrono_collision_SOURCES
    collision/ChCollisionInfo.cpp
    collision/ChCollisionModel.cpp
    collision/ChCollisionSystem.cpp
    collision/ChConvexDecomposition.cpp
    collision/ChCollisionShape.cpp
    collision/ChCollisionShapeArc2D.cpp
    collision/ChCollisionShapeBarrel.cpp
    collision/ChCollisionShapeBox.cpp
    collision/ChCollisionShapeCapsule.cpp
    collision/ChCollisionShapeCone.cpp
    collision/ChCollisionShapeConvexHull.cpp
    collision/ChCollisionShapeCylinder.cpp
    collision/ChCollisionShapeCylindricalShell.cpp
    collision/ChCollisionShapeEllipsoid.cpp
    collision/ChCollisionShapePath2D.cpp
    collision/ChCollisionShapePoint.cpp
    collision/ChCollisionShapeRoundedBox.cpp
    collision/ChCollisionShapeRoundedCylinder.cpp
    collision/ChCollisionShapeSegment2D.cpp
    collision/ChCollisionShapeSegment.cpp
    collision/ChCollisionShapeSphere.cpp
    collision/ChCollisionShapeTriangle.cpp
    collision/ChCollisionShapeMeshTriangle.cpp
    collision/ChCollisionShapeTriangleMesh.cpp
    )
set(Chrono_collision_HEADERS
    collision/ChCollisionInfo.h
    collision/ChCollisionModel.h
    collision/ChCollisionPair.h
    collision/ChCollisionSystem.h
    collision/ChConvexDecomposition.h
    collision/ChCollisionShapes.h
    collision/ChCollisionShape.h
    collision/ChCollisionShapeArc2D.h
    collision/ChCollisionShapeBarrel.h
    collision/ChCollisionShapeBox.h
    collision/ChCollisionShapeCapsule.h
    collision/ChCollisionShapeCone.h
    collision/ChCollisionShapeConvexHull.h
    collision/ChCollisionShapeCylinder.h
    collision/ChCollisionShapeCylindricalShell.h
    collision/ChCollisionShapeEllipsoid.h
    collision/ChCollisionShapePath2D.h
    collision/ChCollisionShapePoint.h
    collision/ChCollisionShapeRoundedBox.h
    collision/ChCollisionShapeRoundedCylinder.h
    collision/ChCollisionShapeSegment2D.h
    collision/ChCollisionShapeSegment.h
    collision/ChCollisionShapeSphere.h
    collision/ChCollisionShapeTriangle.h
    collision/ChCollisionShapeMeshTriangle.h
    collision/ChCollisionShapeTriangleMesh.h
    )
source_group(collision FILES ${Chrono_collision_SOURCES} ${Chrono_collision_HEADERS})

# Multicore collision group

if (CHRONO_THRUST_FOUND)
   message(STATUS "Add Chrono multicore collision detection library")       

   set(Chrono_collision_multicore_SOURCES
       collision/multicore/ChCollisionSystemMulticore.h
       collision/multicore/ChCollisionSystemMulticore.cpp
       collision/multicore/ChCollisionModelMulticore.h
       collision/multicore/ChCollisionModelMulticore.cpp
   )
   set(Chrono_collision_multicore_HEADERS
       collision/multicore/ChCollisionData.h
       collision/multicore/ChConvexShape.h
       collision/multicore/ChBroadphase.h
       collision/multicore/ChBroadphase.cpp
       collision/multicore/ChNarrowphase.h
       collision/multicore/ChNarrowphase.cpp
       collision/multicore/ChNarrowphaseMPR.cpp
       collision/multicore/ChNarrowphasePRIMS.cpp
       collision/multicore/ChRayTest.h
       collision/multicore/ChRayTest.cpp
       collision/multicore/ChCollisionUtils.h
       collision/multicore/ChCollisionUtilsBroadphase.cpp
       collision/multicore/ChCollisionUtilsMPR.cpp
       collision/multicore/ChCollisionUtilsPRIMS.cpp
   )
   source_group("collision\\multicore" FILES ${Chrono_collision_multicore_SOURCES} ${Chrono_collision_multicore_HEADERS})
else()
   set(Chrono_collision_multicore_SOURCES "")
   set(Chrono_collision_multicore_HEADERS "")
endif()

# Bullet collision group

set(Chrono_collision_bullet_SOURCES
    collision/bullet/ChCollisionSystemBullet.cpp
    collision/bullet/ChCollisionModelBullet.cpp
    collision/bullet/ChCollisionAlgorithmsBullet.cpp
    collision/bullet/ChCollisionUtilsBullet.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtAxisSweep3.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtSimpleBroadphase.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtOverlappingPairCache.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtDispatcher.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtDbvt.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtDbvtBroadphase.cpp
    collision/bullet/BulletCollision/BroadphaseCollision/cbtQuantizedBvh.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtUnionFind.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionDispatcher.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionDispatcherMt.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtSphereSphereCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionObject.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtSphereBoxCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionWorld.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionWorldImporter.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtSimulationIslandManager.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtCompoundCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtCompoundCompoundCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtManifoldResult.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtConvexConcaveCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtEmptyCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtSphereTriangleCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtConvexConvexAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtDefaultCollisionConfiguration.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtConvexPlaneCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtActivatingCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtBox2dBox2dCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtBoxBoxCollisionAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtBoxBoxDetector.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtGhostObject.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtConvex2dConvex2dAlgorithm.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtInternalEdgeUtility.cpp
    collision/bullet/BulletCollision/CollisionDispatch/cbtHashedSimplePairCache.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtBarrelShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbt2DShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtCEtriangleShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtSegmentShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtTriangleMeshShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtTriangleMesh.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtTriangleIndexVertexArray.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtCollisionShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtTriangleCallback.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtCompoundShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtTetrahedronShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConcaveShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtStridingMeshInterface.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtStaticPlaneShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConvexHullShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConvexShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConvexTriangleMeshShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtOptimizedBvh.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtCylindricalShellShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtMultiSphereShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtEmptyShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtMinkowskiSumShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtTriangleBuffer.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtHeightfieldTerrainShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtCapsuleShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConvexInternalShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtUniformScalingShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtBox2dShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConvex2dShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConvexPointCloudShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtMultimaterialTriangleMeshShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtScaledBvhTriangleMeshShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtShapeHull.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtTriangleIndexVertexMaterialArray.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtSdfCollisionShape.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtMiniSDF.cpp
    collision/bullet/BulletCollision/CollisionShapes/cbtConvexPolyhedron.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtContinuousConvexCollision.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtSubSimplexConvexCast.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtConvexCast.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtRaycastCallback.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtMinkowskiPenetrationDepthSolver.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkConvexCast.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkEpaPenetrationDepthSolver.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtVoronoiSimplexSolver.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkPairDetector.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtGjkEpa2.cpp
    collision/bullet/BulletCollision/NarrowPhaseCollision/cbtPolyhedralContactClipping.cpp
    collision/bullet/LinearMath/cbtQuickprof.cpp
    collision/bullet/LinearMath/cbtAlignedAllocator.cpp
    collision/bullet/LinearMath/cbtGeometryUtil.cpp
    collision/bullet/LinearMath/cbtConvexHull.cpp
    collision/bullet/LinearMath/cbtConvexHullComputer.cpp
    collision/bullet/LinearMath/cbtSerializer.cpp
    collision/bullet/LinearMath/cbtSerializer64.cpp
    collision/bullet/LinearMath/cbtPolarDecomposition.cpp
    collision/bullet/LinearMath/cbtThreads.cpp
    collision/bullet/LinearMath/cbtVector3.cpp
    collision/bullet/LinearMath/TaskScheduler/cbtTaskScheduler.cpp
    collision/bullet/LinearMath/TaskScheduler/cbtThreadSupportPosix.cpp
    collision/bullet/LinearMath/TaskScheduler/cbtThreadSupportWin32.cpp
    )
set(Chrono_collision_bullet_HEADERS
    collision/bullet/ChCollisionSystemBullet.h
    collision/bullet/ChCollisionModelBullet.h
    collision/bullet/ChCollisionAlgorithmsBullet.h
    collision/bullet/ChCollisionUtilsBullet.h
    )
source_group("collision\\bullet" FILES ${Chrono_collision_bullet_SOURCES} ${Chrono_collision_bullet_HEADERS})

# Particle factory group

set(Chrono_particlefactory_SOURCES
    )
set(Chrono_particlefactory_HEADERS
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
source_group(particlefactory FILES ${Chrono_particlefactory_SOURCES} ${Chrono_particlefactory_HEADERS})

# Timestepper group

set(Chrono_timestepper_SOURCES
    timestepper/ChIntegrable.cpp
    timestepper/ChTimestepper.cpp
    timestepper/ChTimestepperExplicit.cpp
    timestepper/ChTimestepperImplicit.cpp
    timestepper/ChTimestepperHHT.cpp
    timestepper/ChStaticAnalysis.cpp
    timestepper/ChAssemblyAnalysis.cpp
    )
set(Chrono_timestepper_HEADERS
    timestepper/ChState.h
    timestepper/ChIntegrable.h
    timestepper/ChTimestepper.h
    timestepper/ChTimestepperExplicit.h
    timestepper/ChTimestepperImplicit.h
    timestepper/ChTimestepperHHT.h
    timestepper/ChStaticAnalysis.h
    timestepper/ChAssemblyAnalysis.h
    )
source_group(timestepper FILES ${Chrono_timestepper_SOURCES} ${Chrono_timestepper_HEADERS})

# Motion functions group

set(Chrono_functions_SOURCES
	functions/ChFunctionBSpline.cpp
	functions/ChFunctionCycloidal.cpp
    functions/ChFunctionBase.cpp
    functions/ChFunctionConst.cpp
    functions/ChFunctionConstAcc.cpp
    functions/ChFunctionConstJerk.cpp
    functions/ChFunctionDerivative.cpp
    functions/ChFunctionFillet3.cpp
    functions/ChFunctionIntegral.cpp
    functions/ChFunctionInterp.cpp
    functions/ChFunctionMirror.cpp
    functions/ChFunctionOperator.cpp
    functions/ChFunctionPoly.cpp
    functions/ChFunctionPoly23.cpp
    functions/ChFunctionPoly345.cpp
    functions/ChFunctionPosition.cpp
    functions/ChFunctionPositionLine.cpp
    functions/ChFunctionPositionSetpoint.cpp
    functions/ChFunctionPositionXYZFunctions.cpp
    functions/ChFunctionRamp.cpp
    functions/ChFunctionRepeat.cpp
    functions/ChFunctionRotation.cpp
    functions/ChFunctionRotationABCFunctions.cpp
    functions/ChFunctionRotationAxis.cpp
    functions/ChFunctionRotationBSpline.cpp
    functions/ChFunctionRotationSetpoint.cpp
    functions/ChFunctionRotationSQUAD.cpp
    functions/ChFunctionSequence.cpp
    functions/ChFunctionSetpoint.cpp
    functions/ChFunctionSine.cpp
    functions/ChFunctionSineStep.cpp
    )
set(Chrono_functions_HEADERS
	functions/ChFunctionBSpline.h
	functions/ChFunctionCycloidal.h
    functions/ChFunction.h
    functions/ChFunctionBase.h
    functions/ChFunctionConst.h
    functions/ChFunctionConstAcc.h
    functions/ChFunctionConstJerk.h
    functions/ChFunctionDerivative.h
    functions/ChFunctionFillet3.h
    functions/ChFunctionIntegral.h
    functions/ChFunctionInterp.h
    functions/ChFunctionLambda.h
    functions/ChFunctionMirror.h
    functions/ChFunctionOperator.h
    functions/ChFunctionPoly.h
    functions/ChFunctionPoly23.h
    functions/ChFunctionPoly345.h
    functions/ChFunctionPosition.h
    functions/ChFunctionPositionLine.h
    functions/ChFunctionPositionSetpoint.h
    functions/ChFunctionPositionXYZFunctions.h
    functions/ChFunctionRamp.h
    functions/ChFunctionRepeat.h
    functions/ChFunctionRotation.h
    functions/ChFunctionRotationABCFunctions.h
    functions/ChFunctionRotationAxis.h
    functions/ChFunctionRotationBSpline.h
    functions/ChFunctionRotationSetpoint.h
    functions/ChFunctionRotationSQUAD.h
    functions/ChFunctionSequence.h
    functions/ChFunctionSetpoint.h
    functions/ChFunctionSine.h
    functions/ChFunctionSineStep.h
    )
source_group(functions FILES ${Chrono_functions_SOURCES} ${Chrono_functions_HEADERS})

# Solver group

set(Chrono_solver_SOURCES
    solver/ChSystemDescriptor.cpp
    solver/ChSolver.cpp
    solver/ChDirectSolverLS.cpp
    solver/ChDirectSolverLScomplex.cpp
    solver/ChIterativeSolver.cpp
    solver/ChIterativeSolverLS.cpp
    solver/ChIterativeSolverVI.cpp
    solver/ChSolverPSOR.cpp
    solver/ChSolverPJacobi.cpp
    solver/ChSolverPSSOR.cpp
    solver/ChSolverPMINRES.cpp
    solver/ChSolverBB.cpp
    solver/ChSolverAPGD.cpp
    solver/ChSolverADMM.cpp
    solver/ChKRMBlock.cpp
    solver/ChNlsolver.cpp
    )
set(Chrono_solver_HEADERS
    solver/ChSystemDescriptor.h
    solver/ChSolver.h
    solver/ChSolverLS.h
    solver/ChSolverVI.h
    solver/ChDirectSolverLS.h
    solver/ChDirectSolverLScomplex.h
    solver/ChIterativeSolver.h
    solver/ChIterativeSolverLS.h
    solver/ChIterativeSolverVI.h
    solver/ChSolverPJacobi.h
    solver/ChSolverPMINRES.h
    solver/ChSolverBB.h
    solver/ChSolverAPGD.h
    solver/ChSolverADMM.h
    solver/ChSolverPSOR.h
    solver/ChSolverPSSOR.h
    solver/ChKRMBlock.h
    solver/ChNlsolver.h
    )
source_group(solver FILES ${Chrono_solver_SOURCES} ${Chrono_solver_HEADERS})

# Solver sub-groups

set(Chrono_solver_variables_SOURCES
    solver/ChVariables.cpp
    solver/ChVariablesGeneric.cpp
    solver/ChVariablesGenericDiagonalMass.cpp
    solver/ChVariablesBody.cpp
    solver/ChVariablesBodySharedMass.cpp
    solver/ChVariablesBodyOwnMass.cpp
    solver/ChVariablesShaft.cpp
    solver/ChVariablesNode.cpp
)
set(Chrono_solver_variables_HEADERS
    solver/ChVariables.h
    solver/ChVariablesBody.h
    solver/ChVariablesBodyOwnMass.h
    solver/ChVariablesBodySharedMass.h
    solver/ChVariablesShaft.h
    solver/ChVariablesGeneric.h
    solver/ChVariablesGenericDiagonalMass.h
    solver/ChVariablesNode.h
)
source_group(solver\\variables FILES ${Chrono_solver_variables_SOURCES} ${Chrono_solver_variables_HEADERS})

set(Chrono_solver_constraints_SOURCES
    solver/ChConstraint.cpp
    solver/ChConstraintTwo.cpp
    solver/ChConstraintTwoGeneric.cpp
    solver/ChConstraintTwoGenericBoxed.cpp
    solver/ChConstraintTwoBodies.cpp
    solver/ChConstraintThree.cpp
    solver/ChConstraintThreeGeneric.cpp
    solver/ChConstraintThreeBBShaft.cpp
    solver/ChConstraintNgeneric.cpp
    solver/ChConstraintTwoTuples.cpp
    solver/ChConstraintContactNormal.cpp
    solver/ChConstraintContactTangential.cpp
    solver/ChConstraintRollingNormal.cpp
    solver/ChConstraintRollingTangential.cpp
)
set(Chrono_solver_constraints_HEADERS
    solver/ChConstraint.h
    solver/ChConstraintThree.h
    solver/ChConstraintThreeBBShaft.h
    solver/ChConstraintThreeGeneric.h
    solver/ChConstraintTwo.h
    solver/ChConstraintTwoBodies.h
    solver/ChConstraintTwoGeneric.h
    solver/ChConstraintTwoGenericBoxed.h
    solver/ChConstraintNgeneric.h
    solver/ChConstraintTuple.h
    solver/ChConstraintTwoTuples.h
    solver/ChConstraintContactNormal.h
    solver/ChConstraintContactTangential.h
    solver/ChConstraintRollingNormal.h
    solver/ChConstraintRollingTangential.h
)
source_group(solver\\constraints FILES ${Chrono_solver_constraints_SOURCES} ${Chrono_solver_constraints_HEADERS})

# Utility group

set(Chrono_utils_SOURCES
    utils/ChUtilsGeometry.cpp
    utils/ChUtilsCreators.cpp
    utils/ChUtilsGenerators.cpp
    utils/ChUtilsChaseCamera.cpp
    utils/ChValidation.cpp
    utils/ChOpenMP.cpp
    utils/ChProfiler.cpp
    utils/ChControllers.cpp
    utils/ChFilters.cpp
    utils/ChConvexHull.cpp
    utils/ChBodyGeometry.cpp
    utils/ChForceFunctors.cpp
    utils/ChSocket.cpp
    utils/ChSocketCommunication.cpp
    )
set(Chrono_utils_HEADERS
    utils/ChConstants.h
    utils/ChUtils.h
    utils/ChOpenMP.h
    utils/ChUtilsGeometry.h
    utils/ChUtilsCreators.h
    utils/ChUtilsGenerators.h
    utils/ChUtilsSamplers.h
    utils/ChUtilsChaseCamera.h
    utils/ChValidation.h
    utils/ChProfiler.h
    utils/ChControllers.h
    utils/ChFilters.h
    utils/ChConvexHull.h
    utils/ChBodyGeometry.h
    utils/ChForceFunctors.h
    utils/ChSocket.h
    utils/ChSocketCommunication.h
)
if(BUILD_BENCHMARKING)
    set(Chrono_utils_HEADERS ${Chrono_utils_HEADERS} utils/ChBenchmark.h)
endif()
source_group(utils FILES ${Chrono_utils_SOURCES} ${Chrono_utils_HEADERS})

# Output group

set(Chrono_inputoutput_HEADERS
    input_output/ChOutput.h
    input_output/ChOutputASCII.h
    input_output/ChCheckpoint.h
    input_output/ChCheckpointASCII.h
    input_output/ChWriterCSV.h
    input_output/ChUtilsInputOutput.h
)
set(Chrono_inputoutput_SOURCES
    input_output/ChOutputASCII.cpp
    input_output/ChCheckpointASCII.cpp
    input_output/ChWriterCSV.cpp
    input_output/ChUtilsInputOutput.cpp
)
if (HDF5_FOUND)
    set(Chrono_inputoutput_HEADERS ${Chrono_inputoutput_HEADERS}
        input_output/ChOutputHDF5.h
    )
    set(Chrono_inputoutput_SOURCES ${Chrono_inputoutput_SOURCES}
        input_output/ChOutputHDF5.cpp
    )
endif()
source_group(input_output FILES ${Chrono_inputoutput_SOURCES} ${Chrono_inputoutput_HEADERS})

# -------------------------------------------------------------------------------------------------
# Third-party files
# -------------------------------------------------------------------------------------------------

# Gimpact group
set(Chrono_collision_gimpact_FILES
    collision/gimpact/GIMPACT/core/gim_box_set.cpp
    collision/gimpact/GIMPACT/core/gim_contact.cpp
    collision/gimpact/GIMPACT/core/gim_memory.cpp
    collision/gimpact/GIMPACT/core/gim_tri_collision.cpp
    collision/gimpact/GIMPACT/Bullet/cbtContactProcessing.cpp
    collision/gimpact/GIMPACT/Bullet/cbtGenericPoolAllocator.cpp
    collision/gimpact/GIMPACT/Bullet/cbtGImpactBvh.cpp
    collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.cpp
    collision/gimpact/GIMPACT/Bullet/cbtGImpactQuantizedBvh.cpp
    collision/gimpact/GIMPACT/Bullet/cbtGImpactShape.cpp
    collision/gimpact/GIMPACT/Bullet/cbtTriangleShapeEx.cpp
    #collision/gimpact/GIMPACTUtils/cbtGImpactConvexDecompositionShape.cpp
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
source_group(collision\\gimpact FILES ${Chrono_collision_gimpact_FILES})

# HACD group
set(Chrono_collision_convexdecompHACD_FILES
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
    ../chrono_thirdparty/HACD/hacdRaycastMesh.cpp
    ../chrono_thirdparty/HACD/hacdMicroAllocator.cpp
    ../chrono_thirdparty/HACD/hacdGraph.cpp
    ../chrono_thirdparty/HACD/hacdMeshDecimator.cpp
    ../chrono_thirdparty/HACD/hacdHACD.cpp
    ../chrono_thirdparty/HACD/hacdManifoldMesh.cpp
    ../chrono_thirdparty/HACD/hacdICHull.cpp
    )
source_group(collision\\convexdecomp\\HACD FILES ${Chrono_collision_convexdecompHACD_FILES})

# HACDv2 group
set(Chrono_collision_convexdecompHACDv2_FILES
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
source_group(collision\\convexdecomp\\HACDv2 FILES ${Chrono_collision_convexdecompHACDv2_FILES})

# TinyObj Group
set(Chrono_tiny_obj_FILES
  ../chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc
  ../chrono_thirdparty/tinyobjloader/tiny_obj_loader.h
)
source_group(utils\\tiny_obj FILES ${Chrono_tiny_obj_FILES})

# LibSTL Group
set(Chrono_libstl_FILES
  ../chrono_thirdparty/libstl/stlfile.c
  ../chrono_thirdparty/libstl/stlfile.h
)
source_group(utils\\libstl FILES ${Chrono_libstl_FILES})

# Chrono configuration and version files
set(Chrono_CONFIG_FILE ${PROJECT_BINARY_DIR}/chrono/ChConfig.h)
set(Chrono_VERSION_FILE ${PROJECT_BINARY_DIR}/chrono/ChVersion.h)
source_group("" FILES ${Chrono_CONFIG_FILE} ${Chrono_VERSION_FILE})

# -------------------------------------------------------------------------------------------------
# Collect all source and header files for the Chrono core library
# -------------------------------------------------------------------------------------------------

set(Chrono_SOURCES
    ${Chrono_core_SOURCES}
    ${Chrono_serialization_SOURCES}
    ${Chrono_physics_SOURCES}
    ${Chrono_physics_contact_SOURCES}
    ${Chrono_physics_links_SOURCES}
    ${Chrono_physics_loads_SOURCES}
    ${Chrono_physics_hydraulics_SOURCES}
    ${Chrono_physics_shafts_SOURCES}
    ${Chrono_fea_SOURCES}
    ${Chrono_fea_nodes_SOURCES}
    ${Chrono_fea_elements_SOURCES}
    ${Chrono_fea_constraints_SOURCES}
    ${Chrono_fea_materials_SOURCES}
    ${Chrono_fea_sections_SOURCES}
    ${Chrono_solver_SOURCES}
    ${Chrono_solver_constraints_SOURCES}
    ${Chrono_solver_variables_SOURCES}
    ${Chrono_MulticoreMath_SOURCES}
    ${Chrono_collision_SOURCES}
    ${Chrono_geometry_SOURCES}
    ${Chrono_assets_SOURCES}
    ${Chrono_particlefactory_SOURCES}
    ${Chrono_timestepper_SOURCES}
    ${Chrono_functions_SOURCES}
    ${Chrono_collision_multicore_SOURCES}
    ${Chrono_collision_bullet_SOURCES}
    ${Chrono_utils_SOURCES}
    ${Chrono_inputoutput_SOURCES}
)

set(Chrono_HEADERS
    ${Chrono_core_HEADERS}
    ${Chrono_serialization_HEADERS}
    ${Chrono_physics_HEADERS}
    ${Chrono_physics_contact_HEADERS}
    ${Chrono_physics_links_HEADERS}
    ${Chrono_physics_loads_HEADERS}
    ${Chrono_physics_hydraulics_HEADERS}
    ${Chrono_physics_shafts_HEADERS}
    ${Chrono_fea_HEADERS}
    ${Chrono_fea_nodes_HEADERS}
    ${Chrono_fea_elements_HEADERS}
    ${Chrono_fea_constraints_HEADERS}
    ${Chrono_fea_materials_HEADERS}
    ${Chrono_fea_sections_HEADERS}
    ${Chrono_fea_constraints_HEADERS}
    ${Chrono_solver_HEADERS}
    ${Chrono_solver_constraints_HEADERS}
    ${Chrono_solver_variables_HEADERS}
    ${Chrono_MulticoreMath_HEADERS}
    ${Chrono_collision_HEADERS}
    ${Chrono_geometry_HEADERS}
    ${Chrono_assets_HEADERS}
    ${Chrono_particlefactory_HEADERS}
    ${Chrono_timestepper_HEADERS}
    ${Chrono_functions_HEADERS}
    ${Chrono_collision_multicore_HEADERS}
    ${Chrono_collision_bullet_HEADERS}
    ${Chrono_utils_HEADERS}
    ${Chrono_inputoutput_HEADERS}
)

set(Chrono_thirdparty_FILES
    ${Chrono_collision_convexdecompHACD_FILES}
    ${Chrono_collision_convexdecompHACDv2_FILES}
    ${Chrono_collision_gimpact_FILES}
    ${Chrono_tiny_obj_FILES}
    ${Chrono_libstl_FILES}
)

set(Chrono_FILES
    ${Chrono_SOURCES}
    ${Chrono_HEADERS}
    ${Chrono_thirdparty_FILES}
    ${Chrono_CONFIG_FILE}
    ${Chrono_VERSION_FILE}
)

# -------------------------------------------------------------------------------------------------
# Disable all warnings from external code
# -------------------------------------------------------------------------------------------------

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${Chrono_collision_bullet_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${Chrono_collision_convexdecompHACD_FILES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${Chrono_collision_convexdecompHACDv2_FILES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${Chrono_collision_gimpact_FILES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${Chrono_tiny_obj_FILES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
elseif(MSVC)
  set_source_files_properties(${Chrono_collision_bullet_SOURCES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${Chrono_collision_convexdecompHACD_FILES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${Chrono_collision_convexdecompHACDv2_FILES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${Chrono_collision_gimpact_FILES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${Chrono_tiny_obj_FILES} PROPERTIES COMPILE_OPTIONS "/w")
else()
  set_source_files_properties(${Chrono_collision_bullet_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${Chrono_collision_convexdecompHACD_FILES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${Chrono_collision_convexdecompHACDv2_FILES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${Chrono_collision_gimpact_FILES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${Chrono_tiny_obj_FILES} PROPERTIES COMPILE_OPTIONS "-w")
endif()

# -------------------------------------------------------------------------------------------------
# Add the Chrono_core library to the project
# -------------------------------------------------------------------------------------------------

add_library(Chrono_core ${Chrono_FILES})
add_library(Chrono::core ALIAS Chrono_core)

set_target_properties(Chrono_core PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_core PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_core PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  target_compile_definitions(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:CH_STATIC>)
  set_target_properties(Chrono_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_core PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

# TODO DARIOM: CH_INCLUDES_BUILD can be removed, used only here now
list(APPEND CH_INCLUDES_BUILD "${CMAKE_SOURCE_DIR}/src")
list(APPEND CH_INCLUDES_BUILD "${PROJECT_BINARY_DIR}")
list(APPEND CH_INCLUDES_BUILD "${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet" )
list(APPEND CH_INCLUDES_BUILD "${CMAKE_SOURCE_DIR}/src/chrono/collision/gimpact" )
list(APPEND CH_INCLUDES_BUILD "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/HACD")
list(APPEND CH_INCLUDES_BUILD "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/HACDv2")

list(APPEND CH_INCLUDES_INSTALL "include")
list(APPEND CH_INCLUDES_INSTALL "include/chrono/collision/bullet")
list(APPEND CH_INCLUDES_INSTALL "include/chrono/collision/gimpact")
list(APPEND CH_INCLUDES_INSTALL "include/chrono_thirdparty/HACD")
list(APPEND CH_INCLUDES_INSTALL "include/chrono_thirdparty/HACDv2")

target_include_directories(Chrono_core PUBLIC
  "$<BUILD_INTERFACE:${CH_INCLUDES_BUILD}>"
  "$<INSTALL_INTERFACE:${CH_INCLUDES_INSTALL}>"
)

target_link_libraries(Chrono_core PUBLIC Eigen3::Eigen)

if(CHRONO_THRUST_FOUND)
  target_link_libraries(Chrono_core PRIVATE Thrust::Thrust)

  if(CH_ENABLE_OPENMP)
    target_compile_definitions(Chrono_core PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP")
    target_compile_definitions(Chrono_core PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP")
  else()
    target_compile_definitions(Chrono_core PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CPP")
    target_compile_definitions(Chrono_core PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")
  endif()
endif()

if(CH_ENABLE_OPENMP)
  target_link_libraries(Chrono_core PUBLIC OpenMP::OpenMP_CXX)
endif()

if(UNIX)
  target_link_libraries(Chrono_core PUBLIC pthread)
endif()

if(HDF5_FOUND)
    if(TARGET hdf5::hdf5-static AND (NOT APPLE))
      # There are some issues with the static version of HDF5 on macOS
      # See https://autotwin.github.io/automesh/installation.html#zlib-target-not-found
      message(STATUS "Linking static libraries hdf5::hdf5-static")
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5-static)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_cpp-static)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_hl-static)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_hl_cpp-static)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_tools-static)
    elseif(TARGET hdf5::hdf5-shared)
      message(STATUS "Linking shared libraries hdf5::hdf5-shared")
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5-shared)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_cpp-shared)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_hl-shared)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_hl_cpp-shared)
      target_link_libraries(Chrono_core PUBLIC hdf5::hdf5_tools-shared)
    elseif(TARGET HDF5::HDF5)
      message(STATUS "Linking HDF5::HDF5")
      target_link_libraries(Chrono_core PUBLIC HDF5::HDF5)
    else()
      message(STATUS "No HDF5 target found")
      target_compile_definitions(Chrono_core PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${HDF5_COMPILE_DEFS}>)
      target_include_directories(Chrono_core PUBLIC ${HDF5_INCLUDE_DIRS})
      target_link_libraries(Chrono_core PUBLIC ${HDF5_CXX_LIBRARIES})
    endif()
endif()

# Add the 'socket' library to the linking, depending on platform.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	if("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        target_link_libraries(Chrono_core PUBLIC Ws2_32.lib)
	elseif("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        target_link_libraries(Chrono_core PUBLIC Ws2_32.lib)
	elseif("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
	elseif("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
endif()

if(CH_USE_SIMD)
    message(STATUS "Using SIMD flags: ${SIMD_FLAGS}")
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:${SIMD_FLAGS}>)
endif() 

#-----------------------------------------------------------------------------
# Miscellaneous compiler flags
#-----------------------------------------------------------------------------

## WARNING: with the new CUDA handling flags are not propagated to the host compiler
# - without generator-expression e.g. $<$<COMPILE_LANGUAGE:CXX>:MYOPTION> the compiler options will end up being propagated to any compiler/language
# - e.g. the NVCC (device) compiler will also get those flag meant to be to for CXX thus causing errors
# - however the CXX flags guarded by $<COMPILE_LANGUAGE:CXX> will not be propagated to the NVCC (host) compiler; the only way is to propagate them manually through:
#       target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler="MYOPTION">)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        target_compile_definitions(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:_CRT_SECURE_NO_DEPRECATE>)  # avoids deprecation warnings
        target_compile_definitions(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:_SCL_SECURE_NO_DEPRECATE>)  # avoids deprecation warnings
        target_compile_definitions(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:NOMINMAX>) # do not use MSVC's min/max macros
        target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/MP>) # perform parallel builds

        target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/Zc:__cplusplus>)
        target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler="/Zc:__cplusplus">) # without this flag the memory alignment seems to be broken
        target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/EHsc>) # Resolve C4530: https://learn.microsoft.com/en-us/cpp/build/reference/eh-exception-handling-model        

        # excluding wrong runtime libraries from the link (LNK4098): https://learn.microsoft.com/en-us/cpp/error-messages/tool-errors/linker-tools-warning-lnk4098
        if(CH_USE_MSVC_STATIC_RUNTIME)
          target_link_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/NODEFAULTLIB:msvcrt.lib;/NODEFAULTLIB:msvcrtd.lib;/NODEFAULTLIB:libcmt$<$<NOT:$<CONFIG:Debug>>:d>.lib>)
        else()
          target_link_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/NODEFAULTLIB:libcmt.lib;/NODEFAULTLIB:libcmtd.lib;/NODEFAULTLIB:msvcrt$<$<NOT:$<CONFIG:Debug>>:d>.lib>)
        endif()

    endif(MSVC)

    if(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        target_compile_definitions(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:MINGW>)
        target_compile_definitions(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:_WINDOWS>)
        target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wl,--enable-runtime-pseudo-reloc>)
        target_link_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wl,--export-all-symbols;-Wl,--enable-auto-import;-Wl,--enable-runtime-pseudo-reloc>)

        if(CMAKE_SIZEOF_VOID_P MATCHES 8)
            target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-m64>)
        endif()
    endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wl,-no_compact_unwind>)
endif()

# Compiler warning level and disbaled warnings

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(STATUS "Warning level set to default")
    #target_compile_options(Chrono_core PUBLIC -Wall)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unknown-warning-option>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-reorder-ctor>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-pragma-pack>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-local-typedef>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-function>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-result>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-switch>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated>)
elseif(MSVC)
    message(STATUS "Warning level set to /W4")
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/W4>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4100>)   # unreferenced formal parameter
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4127>)   # conditional expression is constant
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4201>)   # nameless struct/union
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4244>)   # conversion; possible loss of data
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4250>)   # inheritance via dominance
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)   # class needs to have dll-interface
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4275>)   # non dll-interface class used as base class
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4324>)   # structure padded due to alignment specifier
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4324>) # structure padded due to alignment specifier (CUDA)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4505>)   # unreferenced local function
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4996>) # WARNING: TODO: DARIOM: CUDA Deprecation: The type will be removed in the next major release
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd5038>)   # order of initialization
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd26451>)  # arithmetic overflow
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd26495>)  # uninitialized member variable
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd26812>)  # use of unscoped enums
else()
    message(STATUS "Warning level set to -Wall")
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wint-in-bool-context>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-sign-compare>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-reorder>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-function>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-result>)
    target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated>)
endif()

# Globally define debug and NOT-debug macros based on build type 
target_compile_definitions(Chrono_core PUBLIC
    $<$<COMPILE_LANGUAGE:CXX>:$<$<CONFIG:DEBUG>:DEBUG>>
    $<$<COMPILE_LANGUAGE:CXX>:$<$<CONFIG:DEBUG>:_DEBUG>>
    $<$<COMPILE_LANGUAGE:CXX>:$<$<CONFIG:RELEASE>:NDEBUG>>
)

target_compile_features(Chrono_core PUBLIC ${CHRONO_CXX_STANDARD})

target_compile_definitions(Chrono_core PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE>)
target_compile_definitions(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:CH_IGNORE_DEPRECATED>)

if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_link_directories(Chrono_core INTERFACE $<INSTALL_INTERFACE:lib>)
endif()

#-----------------------------------------------------------------------------
# Collect compiler flags required to build the Chrono libraries
#-----------------------------------------------------------------------------

target_compile_definitions(Chrono_core PRIVATE BT_THREADSAFE)
target_compile_definitions(Chrono_core PRIVATE BP_USE_FIXEDPOINT_INT_32)

if (CH_USE_BULLET_DOUBLE)
    target_compile_definitions(Chrono_core PRIVATE BT_USE_DOUBLE_PRECISION)
endif()

if (CH_USE_BULLET_OPENMP)
    target_compile_definitions(Chrono_core PRIVATE BT_USE_OPENMP)
endif()

# For Bullet to use 32 bit math

if(XCODE_VERSION)
    set_target_properties(Chrono_core PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT ${CH_ENABLE_OPENMP})
endif()
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC AND MSVC_VERSION GREATER_EQUAL 1900)
        target_compile_options(Chrono_core PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/bigobj>)
    endif()
endif()

# Fix for hang-up in compilation with 32 bit on MSVC in Release configuration with EIGEN3.
# Note: this flag may DECREASE the overall calculation performance by approx. 10%
# For more information, see: https://gitlab.com/libeigen/eigen/-/issues/2379
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND MSVC AND ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32"))
    message(NOTICE "Enabling strong inlining of Eigen because of Win32 machine. This may decrease performance.")
	target_compile_definitions(Chrono_core PRIVATE "$<$<CONFIG:RELEASE>:EIGEN_STRONG_INLINE=inline>")
endif()

install(TARGETS Chrono_core
    EXPORT ChronoTargets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include/chrono
)

# -------------------------------------------------------------------------------------------------
# Install files
# -------------------------------------------------------------------------------------------------

# FILE_SET only availabnle in CMake 3.23 and newer!
#install(TARGETS Chrono_core
#        FILE_SET HEADERS
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/chrono
#        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#)

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono
        FILES_MATCHING REGEX ".*\\.(h|inl)$")

# Install 3rd party headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/filesystem
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/cxxopts
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/HACD
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/HACDv2
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/variant
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/rapidjson
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/rapidxml
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/yaml-cpp
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
