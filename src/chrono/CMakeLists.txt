#=============================================================================
#
# CMake configuration file for the main ChronoEngine library.
#
#=============================================================================

message(STATUS "\n==== Chrono Engine core module ====\n")

#-----------------------------------------------------------------------------
# Double precision support for Bullet
#-----------------------------------------------------------------------------
option(USE_BULLET_DOUBLE "Compile Chrono::Engine with double precision support in Bullet" OFF)
cmake_dependent_option(USE_BULLET_OPENMP "Compile Chrono::Engine with OpenMP support in Bullet" ON "ENABLE_OPENMP" OFF)

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

message(STATUS "Chrono includes: ${CH_INCLUDES}")
include_directories(${CH_INCLUDES})

#-----------------------------------------------------------------------------
# Collect files in the main ChronoEngine library
#-----------------------------------------------------------------------------

# Core group

set(ChronoEngine_core_SOURCES
    core/ChClassFactory.cpp
    core/ChQuaternion.cpp
    core/ChRotation.cpp
    core/ChVector3.cpp
    core/ChCoordsys.cpp
    core/ChQuadrature.cpp
    core/ChBezierCurve.cpp
    core/ChCubicSpline.cpp
    core/ChRandom.cpp
    core/ChGlobal.cpp
    )

set(ChronoEngine_core_HEADERS
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
    core/ChGlobal.h
    core/ChTypes.h
    core/ChTensors.h
    )

source_group(core FILES
    ${ChronoEngine_core_SOURCES}
    ${ChronoEngine_core_HEADERS})

# Serialization group

set(ChronoEngine_serialization_SOURCES
    serialization/ChArchive.cpp
    serialization/ChArchiveBinary.cpp
    serialization/ChObjectExplorer.cpp
    serialization/ChArchiveJSON.cpp
    serialization/ChArchiveXML.cpp
    serialization/ChOutputASCII.cpp
    )

set(ChronoEngine_serialization_HEADERS
    serialization/ChArchive.h
    serialization/ChArchiveBinary.h
    serialization/ChObjectExplorer.h
    serialization/ChArchiveJSON.h
    serialization/ChArchiveXML.h
    serialization/ChOutputASCII.h
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
    physics/ChPhysicsItem.cpp
    physics/ChParticleCloud.cpp
    physics/ChIndexedParticles.cpp
    physics/ChIndexedNodes.cpp
    physics/ChNodeBase.cpp
    physics/ChNodeXYZ.cpp
    physics/ChProximityContainer.cpp
    physics/ChConveyor.cpp
    physics/ChFeeder.cpp
    physics/ChExternalDynamics.cpp
    physics/ChAssembly.cpp
    )

set(ChronoEngine_physics_HEADERS
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
    physics/ChExternalDynamics.h
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
    physics/ChContactable.cpp
    physics/ChContactMaterial.cpp
    physics/ChContactMaterialSMC.cpp
    physics/ChContactMaterialNSC.cpp
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
    physics/ChContactMaterial.h
    physics/ChContactMaterialNSC.h
    physics/ChContactMaterialSMC.h
)

source_group(physics\\contact FILES
    ${ChronoEngine_physics_contact_SOURCES}
    ${ChronoEngine_physics_contact_HEADERS})

set(ChronoEngine_physics_links_SOURCES
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
)

set(ChronoEngine_physics_links_HEADERS
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
)

source_group(physics\\links FILES
    ${ChronoEngine_physics_links_SOURCES}
    ${ChronoEngine_physics_links_HEADERS})

set(ChronoEngine_physics_loads_SOURCES
    physics/ChLoaderU.cpp
    physics/ChLoaderUV.cpp
    physics/ChLoaderUVW.cpp
    physics/ChLoad.cpp
    physics/ChLoadsBody.cpp
    physics/ChLoadsNodeXYZ.cpp
    physics/ChLoadBodyMesh.cpp
    physics/ChLoadContainer.cpp
)

set(ChronoEngine_physics_loads_HEADERS
    physics/ChLoadable.h
    physics/ChLoader.h
    physics/ChLoaderU.h
    physics/ChLoaderUV.h
    physics/ChLoaderUVW.h
    physics/ChLoad.h
    physics/ChLoadsNodeXYZ.h
    physics/ChLoadsBody.h
    physics/ChLoadBodyMesh.h
    physics/ChLoadContainer.h
)

source_group(physics\\loads FILES
    ${ChronoEngine_physics_loads_SOURCES}
    ${ChronoEngine_physics_loads_HEADERS})

set(ChronoEngine_physics_hydraulics_SOURCES
    physics/ChHydraulicCircuit.cpp
    physics/ChHydraulicActuator.cpp
)

set(ChronoEngine_physics_hydraulics_HEADERS
    physics/ChHydraulicCircuit.h
    physics/ChHydraulicActuator.h
)

source_group(physics\\hydraulics FILES
    ${ChronoEngine_physics_hydraulics_SOURCES}
    ${ChronoEngine_physics_hydraulics_HEADERS})

set(ChronoEngine_physics_shafts_SOURCES
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

set(ChronoEngine_physics_shafts_HEADERS
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

source_group(physics\\shafts FILES
    ${ChronoEngine_physics_shafts_SOURCES}
    ${ChronoEngine_physics_shafts_HEADERS})

# FEA group

set(ChronoEngine_fea_FILES
    fea/ChBuilderBeam.cpp
    fea/ChGaussIntegrationRule.cpp
    fea/ChGaussPoint.cpp
    fea/ChMesh.cpp
    fea/ChMeshFileLoader.cpp
    fea/ChMeshExporter.cpp
    fea/ChPolarDecomposition.cpp
    fea/ChMatrixCorotation.cpp
    fea/ChContactSurface.cpp
    fea/ChContactSurfaceNodeCloud.cpp
    fea/ChContactSurfaceMesh.cpp
    fea/ChMeshSurface.cpp
    fea/ChLoadContactSurfaceMesh.cpp
    fea/ChLoadsNodeXYZRot.cpp
    #
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
    fea/ChContactSurfaceNodeCloud.h
    fea/ChContactSurfaceMesh.h
    fea/ChMeshSurface.h
    fea/ChLoadContactSurfaceMesh.h
    fea/ChLoadsNodeXYZRot.h
    fea/ChRotUtils.h
)

source_group(fea FILES ${ChronoEngine_fea_FILES})

# FEA sub-groups

set(ChronoEngine_fea_nodes_FILES
    fea/ChNodeFEAxyz.cpp
    fea/ChNodeFEAxyzrot.cpp
    fea/ChNodeFEAxyzP.cpp
    fea/ChNodeFEAxyzD.cpp
    fea/ChNodeFEAxyzDD.cpp
    fea/ChNodeFEAxyzDDD.cpp
    fea/ChNodeFEAcurv.cpp
    #
    fea/ChNodeFEAbase.h
    fea/ChNodeFEAxyz.h
    fea/ChNodeFEAxyzrot.h
    fea/ChNodeFEAxyzP.h
    fea/ChNodeFEAxyzD.h
    fea/ChNodeFEAxyzDD.h
    fea/ChNodeFEAxyzDDD.h
    fea/ChNodeFEAcurv.h
)

source_group(fea\\nodes FILES ${ChronoEngine_fea_nodes_FILES})

set(ChronoEngine_fea_elements_FILES
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
    #
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

source_group(fea\\elements FILES ${ChronoEngine_fea_elements_FILES})

set(ChronoEngine_fea_materials_FILES
    fea/ChContinuumMaterial.cpp
    fea/ChMaterialBeamANCF.cpp
    fea/ChMaterialShellANCF.cpp
    fea/ChMaterialHexaANCF.cpp
    fea/ChMaterialShellReissner.cpp
    fea/ChMaterialShellKirchhoff.cpp
    #
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

source_group(fea\\materials FILES ${ChronoEngine_fea_materials_FILES})

set(ChronoEngine_fea_sections_FILES
    fea/ChBeamSection.cpp
    fea/ChBeamSectionShape.cpp
    fea/ChBeamSectionCosserat.cpp
    fea/ChBeamSectionEuler.cpp
    fea/ChBeamSectionTaperedTimoshenko.cpp
    fea/ChBeamSectionTaperedTimoshenkoFPM.cpp
    fea/ChBeamSectionCable.cpp
    #
    fea/ChBeamSection.h
    fea/ChBeamSectionShape.h
    fea/ChBeamSectionCosserat.h
    fea/ChBeamSectionEuler.h
    fea/ChBeamSectionTaperedTimoshenko.h
    fea/ChBeamSectionTaperedTimoshenkoFPM.h
    fea/ChBeamSectionCable.h
)

source_group(fea\\sections FILES ${ChronoEngine_fea_sections_FILES})

set(ChronoEngine_fea_constraints_FILES
    fea/ChLinkNodeFrame.cpp
    fea/ChLinkNodeSlopeFrame.cpp
    fea/ChLinkNodeNode.cpp
    fea/ChLinkNodeFace.cpp
    fea/ChLinkBeamIGAFrame.cpp
    #
    fea/ChLinkNodeFrame.h
    fea/ChLinkNodeSlopeFrame.h
    fea/ChLinkNodeNode.h
    fea/ChLinkNodeFace.h
    fea/ChLinkBeamIGAFrame.h
)

source_group(fea\\constraints FILES ${ChronoEngine_fea_constraints_FILES})

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

source_group(geometry FILES
    ${ChronoEngine_geometry_SOURCES}
    ${ChronoEngine_geometry_HEADERS})

# Asset group

set(ChronoEngine_assets_SOURCES
    assets/ChTexture.cpp
    assets/ChCamera.cpp
    assets/ChColor.cpp
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

set(ChronoEngine_assets_HEADERS
    assets/ChTexture.h
    assets/ChCamera.h
    assets/ChColor.h
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

source_group(assets FILES
    ${ChronoEngine_assets_SOURCES}
    ${ChronoEngine_assets_HEADERS})


# Multicore types and math 

if (THRUST_FOUND)
    set(ChronoEngine_MulticoreMath_SOURCES
        multicore_math/ChMulticoreMath.h
        multicore_math/ChCudaDefines.h
        multicore_math/matrix.cpp
        multicore_math/matrix.h
        multicore_math/other_types.h
        multicore_math/real.h
        multicore_math/real_double.h
        multicore_math/real_single.h
        multicore_math/real2.h
        multicore_math/real2.cpp
        multicore_math/real3.cpp
        multicore_math/real3.h
        multicore_math/real4.cpp
        multicore_math/real4.h
        multicore_math/simd_avx.h
        multicore_math/simd_non.h
        multicore_math/simd_sse.h
        multicore_math/simd.h
        multicore_math/utility.h
        multicore_math/vec3.cpp
        )
    source_group(multicore_math FILES ${ChronoEngine_MulticoreMath_SOURCES})
elseif()
    set(ChronoEngine_MulticoreMath_SOURCES "")
endif()

# Collision group --------------------------------------------------------------

set(ChronoEngine_collision_SOURCES
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
    collision/ChCollisionShapeSphere.cpp
    collision/ChCollisionShapeTriangle.cpp
    collision/ChCollisionShapeMeshTriangle.cpp
    collision/ChCollisionShapeTriangleMesh.cpp
    )

set(ChronoEngine_collision_HEADERS
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
    collision/ChCollisionShapeSphere.h
    collision/ChCollisionShapeTriangle.h
    collision/ChCollisionShapeMeshTriangle.h
    collision/ChCollisionShapeTriangleMesh.h
    )

source_group(collision FILES
    ${ChronoEngine_collision_SOURCES}
    ${ChronoEngine_collision_HEADERS})

# Chrono collision group

if (THRUST_FOUND)
   message(STATUS "Adding internal multicore collision detection library.")       

   set(ChronoEngine_collision_multicore_SOURCES
       collision/multicore/ChCollisionSystemMulticore.h
       collision/multicore/ChCollisionSystemMulticore.cpp
       collision/multicore/ChCollisionModelMulticore.h
       collision/multicore/ChCollisionModelMulticore.cpp
#
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

   source_group("collision\\multicore" FILES
       ${ChronoEngine_collision_multicore_SOURCES})
endif()

# Bullet collision group

set(ChronoEngine_collision_bullet_SOURCES
    collision/bullet/ChCollisionSystemBullet.h
    collision/bullet/ChCollisionSystemBullet.cpp
    collision/bullet/ChCollisionModelBullet.h
    collision/bullet/ChCollisionModelBullet.cpp
    collision/bullet/ChCollisionAlgorithmsBullet.h
    collision/bullet/ChCollisionAlgorithmsBullet.cpp
    collision/bullet/ChCollisionUtilsBullet.h
    collision/bullet/ChCollisionUtilsBullet.cpp
#
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

source_group("collision\\bullet" FILES
    ${ChronoEngine_collision_bullet_SOURCES})

# Gimpact group

set(ChronoEngine_collision_gimpact_SOURCES
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

# Particle factory group -------------------------------------------------------

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
    )

source_group(timestepper FILES
    ${ChronoEngine_timestepper_SOURCES}
    ${ChronoEngine_timestepper_HEADERS})

# Motion functions group

set(ChronoEngine_functions_SOURCES
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

set(ChronoEngine_functions_HEADERS
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


source_group(functions FILES
    ${ChronoEngine_functions_SOURCES}
    ${ChronoEngine_functions_HEADERS})

# Solver group

set(ChronoEngine_solver_SOURCES
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

set(ChronoEngine_solver_HEADERS
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

# Utility group

set(ChronoEngine_utils_SOURCES
    utils/ChUtilsGeometry.cpp
    utils/ChUtilsCreators.cpp
    utils/ChUtilsGenerators.cpp
    utils/ChUtilsInputOutput.cpp
    utils/ChUtilsChaseCamera.cpp
    utils/ChUtilsValidation.cpp
    utils/ChProfiler.cpp
    utils/ChControllers.cpp
    utils/ChFilters.cpp
    utils/ChCompositeInertia.cpp
    utils/ChConvexHull.cpp
    utils/ChBodyGeometry.cpp
    utils/ChSocket.cpp
    utils/ChSocketCommunication.cpp
    )

set(ChronoEngine_utils_HEADERS
    utils/ChConstants.h
    utils/ChUtils.h
    utils/ChOpenMP.h
    utils/ChUtilsGeometry.h
    utils/ChUtilsCreators.h
    utils/ChUtilsGenerators.h
    utils/ChUtilsSamplers.h
    utils/ChUtilsInputOutput.h
    utils/ChUtilsChaseCamera.h
    utils/ChUtilsValidation.h
    utils/ChProfiler.h
    utils/ChControllers.h
    utils/ChFilters.h
    utils/ChCompositeInertia.h
    utils/ChConvexHull.h
    utils/ChBodyGeometry.h
    utils/ChSocket.h
    utils/ChSocketCommunication.h
)

if(BUILD_BENCHMARKING)
    set(ChronoEngine_utils_HEADERS ${ChronoEngine_utils_HEADERS} utils/ChBenchmark.h)
    message(STATUS "Adding Chrono utilities for benchmarking.")
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

# LibSTL Group

set(ChronoEngine_libstl_FILES
  ../chrono_thirdparty/libstl/stlfile.c
  ../chrono_thirdparty/libstl/stlfile.h
)

source_group(utils\\libstl FILES
    ${ChronoEngine_libstl_FILES})


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
    ${ChronoEngine_physics_hydraulics_SOURCES}
    ${ChronoEngine_physics_hydraulics_HEADERS}
    ${ChronoEngine_physics_shafts_SOURCES}
    ${ChronoEngine_physics_shafts_HEADERS}
    ${ChronoEngine_fea_FILES}
    ${ChronoEngine_fea_nodes_FILES}
    ${ChronoEngine_fea_elements_FILES}
    ${ChronoEngine_fea_constraints_FILES}
    ${ChronoEngine_fea_materials_FILES}
    ${ChronoEngine_fea_sections_FILES}
    ${ChronoEngine_fea_constraints_HEADERS}
    ${ChronoEngine_solver_SOURCES}
    ${ChronoEngine_solver_HEADERS}
    ${ChronoEngine_solver_constraints_SOURCES}
    ${ChronoEngine_solver_constraints_HEADERS}
    ${ChronoEngine_solver_variables_SOURCES}
    ${ChronoEngine_solver_variables_HEADERS}
    ${ChronoEngine_MulticoreMath_SOURCES}
    ${ChronoEngine_collision_SOURCES}
    ${ChronoEngine_collision_HEADERS}
    ${ChronoEngine_geometry_SOURCES}
    ${ChronoEngine_geometry_HEADERS}
    ${ChronoEngine_assets_SOURCES}
    ${ChronoEngine_assets_HEADERS}
    ${ChronoEngine_collision_convexdecompHACD_SOURCES}
    ${ChronoEngine_collision_convexdecompHACDv2_SOURCES}
    ${ChronoEngine_particlefactory_SOURCES}
    ${ChronoEngine_particlefactory_HEADERS}
    ${ChronoEngine_timestepper_SOURCES}
    ${ChronoEngine_timestepper_HEADERS}
    ${ChronoEngine_functions_SOURCES}
    ${ChronoEngine_functions_HEADERS}
    ${ChronoEngine_collision_multicore_SOURCES}
    ${ChronoEngine_collision_bullet_SOURCES}
    ${ChronoEngine_collision_gimpact_SOURCES}
    ${ChronoEngine_utils_SOURCES}
    ${ChronoEngine_utils_HEADERS}
    ${ChronoEngine_tiny_obj_FILES}
    ${ChronoEngine_libstl_FILES}
)

# Add the 'socket' library to the linking, depending on platform.
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
		SET (CH_SOCKET_LIB "Ws2_32.lib")
	ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
		SET (CH_SOCKET_LIB "Ws2_32.lib") # btw not Ws2_64.lib that does not exist
	ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
		SET (CH_SOCKET_LIB "")  # not needed?
	ELSEIF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
		SET (CH_SOCKET_LIB "")  # not needed?
	ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	SET (CH_SOCKET_LIB "")		  # not needed?
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	SET (CH_SOCKET_LIB "")		  # not needed?
ENDIF()
			
SET(CH_SOCKET_LIB ${CH_SOCKET_LIB} PARENT_SCOPE)

# Disable all warnings from external code
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${ChronoEngine_collision_bullet_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${ChronoEngine_collision_convexdecompHACD_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${ChronoEngine_collision_convexdecompHACDv2_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${ChronoEngine_collision_gimpact_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
  set_source_files_properties(${ChronoEngine_tiny_obj_FILES} PROPERTIES COMPILE_OPTIONS "-Wno-everything")
elseif(MSVC)
  set_source_files_properties(${ChronoEngine_collision_bullet_SOURCES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${ChronoEngine_collision_convexdecompHACD_SOURCES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${ChronoEngine_collision_convexdecompHACDv2_SOURCES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${ChronoEngine_collision_gimpact_SOURCES} PROPERTIES COMPILE_OPTIONS "/w")
  set_source_files_properties(${ChronoEngine_tiny_obj_FILES} PROPERTIES COMPILE_OPTIONS "/w")
else()
  set_source_files_properties(${ChronoEngine_collision_bullet_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${ChronoEngine_collision_convexdecompHACD_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${ChronoEngine_collision_convexdecompHACDv2_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${ChronoEngine_collision_gimpact_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
  set_source_files_properties(${ChronoEngine_tiny_obj_FILES} PROPERTIES COMPILE_OPTIONS "-w")
endif()

# Add the ChronoEngine library to the project
add_library(ChronoEngine ${ChronoEngine_FILES})

target_link_libraries(ChronoEngine ${OPENMP_LIBRARIES} ${CH_SOCKET_LIB})
if (UNIX)
  target_link_libraries(ChronoEngine pthread)
endif()

# Set some custom properties of this target
set_target_properties(ChronoEngine PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_LIB}")

target_compile_definitions(ChronoEngine PRIVATE "CH_API_COMPILE")
target_compile_definitions(ChronoEngine PRIVATE "CH_IGNORE_DEPRECATED")

target_compile_definitions(ChronoEngine PRIVATE "BT_THREADSAFE")
if (USE_BULLET_DOUBLE)
    target_compile_definitions(ChronoEngine PRIVATE "BT_USE_DOUBLE_PRECISION")
endif()
if (USE_BULLET_OPENMP)
    target_compile_definitions(ChronoEngine PRIVATE "BT_USE_OPENMP")
endif()

if(XCODE_VERSION)
    set_target_properties(ChronoEngine PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT ${ENABLE_OPENMP})
endif()
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC AND MSVC_VERSION GREATER_EQUAL 1900)
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
    endif()
endif()

# Fix for hang-up in compilation with 32 bit on MSVC in Release configuration with EIGEN3.
# Note: this flag may DECREASE the overall calculation performance by approx. 10%
# For more information, see: https://gitlab.com/libeigen/eigen/-/issues/2379
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND MSVC AND ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32"))
	target_compile_definitions(ChronoEngine PRIVATE "$<$<CONFIG:RELEASE>:EIGEN_STRONG_INLINE=inline>")
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
