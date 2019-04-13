//////////////////////////////////////////////////
//  
//   ChModuleCore.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import pychrono'


%module(directors="1") core


// Turn on the documentation of members, for more intuitive IDE typing

%feature("autodoc", "1");


// Turn on the exception handling to intercept C++ exceptions
%include "exception.i"

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}


// For optional downcasting of polimorphic objects:
%include "chrono_downcast.i" 

// For supporting shared pointers:
%include <std_shared_ptr.i>



// Include C++ headers this way...

%{
#include <typeindex>
#include <cstddef>
#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChNodeBase.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/physics/ChTensors.h"
#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono/physics/ChIndexedNodes.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::geometry;
using namespace chrono::fea;
%}


// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

%ignore CH_ENUM_MAPPER_BEGIN;
%ignore CH_ENUM_VAL;
%ignore CH_ENUM_MAPPER_END;

// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") chrono::ChLoadBase;

// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_vector.i"
%include "typemaps.i"
%include "cpointer.i"

// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);


//
// For each class, keep updated the  A, B, C sections: 
// 


//
// A- ENABLE SHARED POINTERS
//
// Note that this must be done for almost all objects (not only those that are
// handled by shered pointers in C++, but all their chidren and parent classes. It
// is enough that a single class in an inheritance tree uses %shared_ptr, and all other in the 
// tree must be promoted to %shared_ptr too).

%shared_ptr(chrono::ChFrame<double>)
%shared_ptr(chrono::ChFrameMoving<double>)

%shared_ptr(chrono::ChAsset)
%shared_ptr(chrono::ChVisualization)
//%shared_ptr(chrono::ChColor)
%shared_ptr(chrono::ChColorAsset)
%shared_ptr(chrono::ChAssetLevel)
%shared_ptr(chrono::ChObjShapeFile)
%shared_ptr(chrono::ChBoxShape) 
%shared_ptr(chrono::ChSphereShape)
%shared_ptr(chrono::ChEllipsoidShape)
%shared_ptr(chrono::ChCylinderShape)
%shared_ptr(chrono::ChTexture)
%shared_ptr(chrono::ChCamera) 
%shared_ptr(chrono::ChLineShape)
%shared_ptr(chrono::ChSurfaceShape)
%shared_ptr(chrono::ChPathShape)
%shared_ptr(chrono::ChPointPointDrawing)
%shared_ptr(chrono::ChPointPointSegment)
%shared_ptr(chrono::ChPointPointSpring)
%shared_ptr(chrono::ChTriangleMeshShape)

%shared_ptr(chrono::ChFunction)  
%shared_ptr(chrono::ChFunction_Const)
%shared_ptr(chrono::ChFunction_ConstAcc)
%shared_ptr(chrono::ChFunction_Derive)
%shared_ptr(chrono::ChFunction_Fillet3)
%shared_ptr(chrono::ChFunction_Integrate)
%shared_ptr(chrono::ChFunction_Mirror)
%shared_ptr(chrono::ChFunction_Mocap)
%shared_ptr(chrono::ChFunction_Noise)
%shared_ptr(chrono::ChFunction_Operation)
%shared_ptr(chrono::ChFunction_Oscilloscope)
%shared_ptr(chrono::ChFunction_Poly)
%shared_ptr(chrono::ChFunction_Poly345)
%shared_ptr(chrono::ChFunction_Ramp)
%shared_ptr(chrono::ChFunction_Recorder)
%shared_ptr(chrono::ChFunction_Repeat)
%shared_ptr(chrono::ChFunction_Sequence)
%shared_ptr(chrono::ChFunction_Sigma)
%shared_ptr(chrono::ChFunction_Sine)

%shared_ptr(chrono::ChObj)
%shared_ptr(chrono::collision::ChCollisionModel)
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChIndexedNodes)
%shared_ptr(chrono::ChMaterialSurfaceNSC)
%shared_ptr(chrono::ChMaterialSurfaceSMC)
%shared_ptr(chrono::ChMaterialSurface)
%shared_ptr(chrono::ChContinuumMaterial)
%shared_ptr(chrono::ChContinuumElastic)
%shared_ptr(chrono::ChContinuumElastoplastic)
%shared_ptr(chrono::ChContinuumPlasticVonMises)
%shared_ptr(chrono::ChContinuumDruckerPrager)
%shared_ptr(chrono::ChBodyFrame)
%shared_ptr(chrono::ChMarker)
%shared_ptr(chrono::ChForce)
%shared_ptr(chrono::ChBody)
%shared_ptr(chrono::ChBodyAuxRef)
%shared_ptr(chrono::ChBodyEasySphere)
%shared_ptr(chrono::ChBodyEasyBox)
%shared_ptr(chrono::ChBodyEasyEllipsoid)
%shared_ptr(chrono::ChBodyEasyCylinder)
%shared_ptr(chrono::ChBodyEasyConvexHull)
%shared_ptr(chrono::ChBodyEasyConvexHullAuxRef)
%shared_ptr(chrono::ChBodyEasyMesh)
%shared_ptr(chrono::ChBodyEasyClusterOfSpheres)
%shared_ptr(chrono::ChConveyor)
%shared_ptr(chrono::ChAparticle)
%shared_ptr(chrono::ChParticleBase)
%shared_ptr(chrono::ChIndexedParticles)
%shared_ptr(chrono::ChParticlesClones)
%shared_ptr(chrono::ChAssembly)
// shared_ptr macros for ChIntegrable, Chtimestepper and their children classes moved into "ChTimestepper.i"
%shared_ptr(chrono::ChSolver)
%shared_ptr(chrono::ChSystem)
%shared_ptr(chrono::ChSystemNSC)
%shared_ptr(chrono::ChSystemSMC)
%shared_ptr(chrono::ChContactContainer)
%shared_ptr(chrono::ChProximityContainer)
%shared_ptr(chrono::ChLoadContainer)

%shared_ptr(chrono::ChLinkBase)
%shared_ptr(chrono::ChLink)
%shared_ptr(chrono::ChLinkMarkers)
%shared_ptr(chrono::ChLinkLimit)
%shared_ptr(chrono::ChLinkLock)
%shared_ptr(chrono::ChLinkLockRevolute)
%shared_ptr(chrono::ChLinkLockLock)
%shared_ptr(chrono::ChLinkLockSpherical)
%shared_ptr(chrono::ChLinkLockCylindrical)
%shared_ptr(chrono::ChLinkLockPrismatic)
%shared_ptr(chrono::ChLinkLockPointPlane)
%shared_ptr(chrono::ChLinkLockPointLine)
%shared_ptr(chrono::ChLinkLockPlanePlane)
%shared_ptr(chrono::ChLinkLockOldham)
%shared_ptr(chrono::ChLinkLockFree)
%shared_ptr(chrono::ChLinkLockAlign)
%shared_ptr(chrono::ChLinkLockParallel)
%shared_ptr(chrono::ChLinkLockPerpend)
%shared_ptr(chrono::ChLinkLockRevolutePrismatic)
%shared_ptr(chrono::ChLinkDistance)
%shared_ptr(chrono::ChLinkGear)
%shared_ptr(chrono::ChLinkLinActuator)
%shared_ptr(chrono::ChLinkMate)
%shared_ptr(chrono::ChLinkMateGeneric)
%shared_ptr(chrono::ChLinkMatePlane)
%shared_ptr(chrono::ChLinkMateCoaxial)
%shared_ptr(chrono::ChLinkMateSpherical)
%shared_ptr(chrono::ChLinkMateXdistance)
%shared_ptr(chrono::ChLinkMateParallel)
%shared_ptr(chrono::ChLinkMateOrthogonal)
%shared_ptr(chrono::ChLinkMateFix)
%shared_ptr(chrono::ChLinkPulley)
%shared_ptr(chrono::ChLinkRevolute)
%shared_ptr(chrono::ChLinkRevoluteSpherical)
%shared_ptr(chrono::ChLinkScrew)
%shared_ptr(chrono::ChLinkSpring)
%shared_ptr(chrono::ChLinkSpringCB)
%shared_ptr(chrono::ChLinkUniversal)

%shared_ptr(chrono::ChShaft)
%shared_ptr(chrono::ChShaftsBody)
%shared_ptr(chrono::ChShaftsBodyTranslation)
%shared_ptr(chrono::ChShaftsMotorBase)
%shared_ptr(chrono::ChShaftsClutch)
%shared_ptr(chrono::ChShaftsCouple)
%shared_ptr(chrono::ChShaftsGear)
%shared_ptr(chrono::ChShaftsMotor)
%shared_ptr(chrono::ChShaftsPlanetary)
%shared_ptr(chrono::ChShaftsThermalEngine)
%shared_ptr(chrono::ChShaftsTorqueBase)
%shared_ptr(chrono::ChShaftsTorsionSpring) 
%shared_ptr(chrono::ChLinkMotor)
%shared_ptr(chrono::ChLinkMotorLinear)
%shared_ptr(chrono::ChLinkMotorLinearDriveline)
%shared_ptr(chrono::ChLinkMotorLinearForce)
%shared_ptr(chrono::ChLinkMotorLinearPosition)
%shared_ptr(chrono::ChLinkMotorLinearSpeed)
%shared_ptr(chrono::ChLinkMotorRotation)
%shared_ptr(chrono::ChLinkMotorRotationAngle)
%shared_ptr(chrono::ChLinkMotorRotationDriveline)
%shared_ptr(chrono::ChLinkMotorRotationSpeed)
%shared_ptr(chrono::ChLinkMotorRotationTorque)
%shared_ptr(chrono::ChLinkTrajectory)
%shared_ptr(chrono::ChLinkPointSpline)
%shared_ptr(chrono::ChLoadBase)
%shared_ptr(chrono::ChLoad)
%shared_ptr(chrono::ChLoadCustom)
%shared_ptr(chrono::ChLoadCustomMultiple)
%shared_ptr(chrono::ChLoadBodyForce)
%shared_ptr(chrono::ChLoadBodyTorque)
%shared_ptr(chrono::ChLoadBodyBody)
%shared_ptr(chrono::ChLoadBodyBodyTorque)
%shared_ptr(chrono::ChLoadBodyBodyBushingSpherical)
%shared_ptr(chrono::ChLoadBodyBodyBushingPlastic)
%shared_ptr(chrono::ChLoadBodyBodyBushingMate)
%shared_ptr(chrono::ChLoadBodyBodyBushingPlastic)
%shared_ptr(chrono::ChLoadBodyBodyBushingGeneric)

%shared_ptr(chrono::geometry::ChGeometry)
%shared_ptr(chrono::geometry::ChLine)
%shared_ptr(chrono::geometry::ChVolume)
%shared_ptr(chrono::geometry::ChSurface)
%shared_ptr(chrono::geometry::ChBox)
%shared_ptr(chrono::geometry::ChSphere)
%shared_ptr(chrono::geometry::ChCylinder)
%shared_ptr(chrono::geometry::ChCapsule)
%shared_ptr(chrono::geometry::ChCone)
%shared_ptr(chrono::geometry::ChEllipsoid)
%shared_ptr(chrono::geometry::ChLineArc)
%shared_ptr(chrono::geometry::ChLineSegment)
%shared_ptr(chrono::geometry::ChLineNurbs)
%shared_ptr(chrono::geometry::ChLinePath)
%shared_ptr(chrono::geometry::ChLinePoly)
%shared_ptr(chrono::geometry::ChLineBezier)
%shared_ptr(chrono::geometry::ChLineCam)
%shared_ptr(chrono::geometry::ChLineBspline)
%shared_ptr(chrono::geometry::ChTriangle)
%shared_ptr(chrono::geometry::ChSurfaceNurbs)
%shared_ptr(chrono::geometry::ChTriangleMesh)
%shared_ptr(chrono::geometry::ChTriangleMeshConnected)
%shared_ptr(chrono::geometry::ChTriangleMeshSoup)

//
// B- INCLUDE HEADERS
//
//
// 1) 
//    When including with %include all the .i files, make sure that 
// the .i of a derived class is included AFTER the .i of
// a base class, otherwise SWIG is not able to build the type
// infos. 
//
// 2)
//    Then, this said, if one member function in Foo_B.i returns
// an object of Foo_A.i (or uses it as a parameter) and yet you must %include
// A before B, ex.because of rule 1), a 'forward reference' to A must be done in
// B by. Seems that it is enough to write 
//  mynamespace { class myclass; }
// in the .i file, before the %include of the .h, even if already forwarded in .h

//  core/  classes
%include "ChException.i"
%include "ChClassFactory.i"
%include "../chrono/physics/ChGlobal.h"
//%include "ChArchive.i"
%include "ChVector.i"
#define Vector ChVector<double>
%include "ChQuaternion.i"
#define Quaternion ChQuaternion<double>
%include "ChCoordsys.i"
#define Coordsys ChCoordsys<double>
%include "ChFrame.i"
%include "ChFrameMoving.i"
%include "ChLinearAlgebra.i"
%include "ChStream.i"
%include "ChLog.i"
%include "ChMathematics.i"
%include "ChMatrix.i"
%include "ChVectorDynamic.i"
%include "ChTimer.i"
%include "ChRealtimeStep.i"
%include "ChTransform.i"

// motion_functions/   classes
%include "ChFunction_Base.i"

// geometry/   classes
%include "ChGeometry.i"

%include "ChCollisionModel.i"
%include "../chrono/collision/ChCCollisionUtils.h"

// assets
%include "ChAsset.i"
%include "ChColor.i"
%include "ChVisualization.i"
%include "ChColorAsset.i"
%include "ChAssetLevel.i"
%include "ChObjShapeFile.i"
%include "ChBoxShape.i"
%include "ChSphereShape.i"
%include "ChCylinderShape.i"
%include "ChTexture.i"
%include "ChCamera.i"
%include "../chrono/assets/ChLineShape.h"
%include "../chrono/assets/ChPathShape.h"
%include "../chrono/assets/ChPointPointDrawing.h"
%include "../chrono/assets/ChSurfaceShape.h"
%include "../chrono/assets/ChTriangleMeshShape.h"
%include "../chrono/assets/ChEllipsoidShape.h"

// physics/  classes
//%include "../chrono/physics/ChTensors.h"
//%template(ChVoightTensorD) chrono::fea::ChVoightTensor<double>;
//%template(ChStressTensorD) chrono::fea::ChStressTensor<double>;
//%template(ChStrainTensorD) chrono::fea::ChStrainTensor<double>;
%include "../chrono/physics/ChContinuumMaterial.h"
%include "ChObject.i"
%include "ChPhysicsItem.i"
%include "../chrono/physics/ChIndexedNodes.h"
%include "ChMaterialSurface.i"
%include "../chrono/physics/ChNodeBase.h"
%include "../chrono/physics/ChNodeXYZ.h"
%include "ChBodyFrame.i"
%include "ChMarker.i"
%include "ChForce.i"
%include "ChBody.i"
%include "ChBodyAuxRef.i"
%include "../chrono/physics/ChBodyEasy.h"
%include "ChConveyor.i"
%include "ChIndexedParticles.i"
%include "ChParticlesClones.i"
%include "ChAssembly.i"
%include "ChTimestepper.i"
%include "../chrono/solver/ChSolver.h"
%include "ChSystem.i"
%include "ChSystemNSC.i"
%include "ChSystemSMC.i"
%include "ChContactContainer.i"
%include "ChProximityContainer.i"
%include "ChLoadContainer.i"
%include "ChLinkBase.i"
%include "ChLink.i"
%include "ChLinkMarkers.i"
%include "ChLinkLimit.i"
%include "ChLinkForce.i"
%include "ChLinkLock.i"
%include "ChLinkMate.i"
%include "ChLinkDistance.i"
%include "ChLinkLinActuator.i"
%include "ChLinkPulley.i"
%include "ChLinkScrew.i"
%include "ChLinkSpring.i"
%include "ChLinkSpringCB.i"
%include "ChLinkGear.i"
%include "ChLinkRevolute.i"
%include "ChLinkRevoluteSpherical.i"
%include "ChLinkUniversal.i" 
%include "ChLinkTrajectory.i" 
%include "ChLinkPointSpline.i"

%include "ChShaft.i"
%include "ChShaftsCouple.i"
%include "ChShaftsBody.i"
%include "ChShaftsClutch.i"
%include "ChShaftsMotor.i"
%include "ChShaftsTorqueBase.i"
%include "ChShaftsTorsionSpring.i"
%include "ChShaftsPlanetary.i"
%include "ChShaftsThermalEngine.i"
%include "ChLinkMotor.i"
%include "../chrono/physics/ChLoad.h"
%include "../chrono/physics/ChLoadsBody.h"
%include "../chrono/physics/ChLoadContainer.h"





//
// C- DOWNCASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. So in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefChSharedPtrDynamicDowncast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

// enable _automatic_ downcasting from ChAsset to derived classes (shared pointers versions)
%downcast_output_sharedptr(chrono::ChAsset, chrono::ChVisualization, chrono::ChObjShapeFile, chrono::ChBoxShape, chrono::ChSphereShape, chrono::ChCylinderShape, chrono::ChTexture, chrono::ChAssetLevel, chrono::ChCamera, chrono::ChColorAsset)

%DefChSharedPtrDynamicDowncast(ChAsset,ChVisualization)
%DefChSharedPtrDynamicDowncast(ChAsset,ChObjShapeFile)
%DefChSharedPtrDynamicDowncast(ChAsset,ChBoxShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChSphereShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChCylinderShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChTexture)
%DefChSharedPtrDynamicDowncast(ChAsset,ChAssetLevel)
%DefChSharedPtrDynamicDowncast(ChAsset,ChCamera)
%DefChSharedPtrDynamicDowncast(ChAsset,ChLineShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChSurfaceShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChPathShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChPointPointDrawing)
%DefChSharedPtrDynamicDowncast(ChAsset,ChPointPointSegment)
%DefChSharedPtrDynamicDowncast(ChAsset,ChPointPointSpring)
%DefChSharedPtrDynamicDowncast(ChAsset,ChTriangleMeshShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChEllipsoidShape)

%DefChSharedPtrDynamicDowncast(ChBodyFrame, ChBody)
%DefChSharedPtrDynamicDowncast(ChBodyFrame, ChBodyAuxRef)
%DefChSharedPtrDynamicDowncast(ChBodyFrame, ChConveyor)
%DefChSharedPtrDynamicDowncast(ChBody, ChBodyFrame)  // <- upcast, for testing & workaround
%DefChSharedPtrDynamicDowncast(ChBodyAuxRef, ChBodyFrame)  // <- upcast, for testing & workaround
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChBody)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChConveyor)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChBodyAuxRef)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChIndexedParticles)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChParticlesClones)

%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLink)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMarkers)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLock)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockLock)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockRevolute)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockSpherical)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockCylindrical)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPrismatic)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPointPlane)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPointLine)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockOldham)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockFree)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockAlign)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockParallel)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPerpend)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMate)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateGeneric)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMatePlane)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateCoaxial)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateSpherical)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateXdistance)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateParallel)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateOrthogonal)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateFix)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkGear)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkDistance)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLinActuator)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkPulley)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkScrew)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkSpring)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkSpringCB)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotor)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorLinear)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorLinearDriveline)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorLinearForce)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorLinearPosition)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorLinearSpeed)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorRotation)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorRotationAngle)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorRotationDriveline)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorRotationSpeed)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMotorRotationTorque)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLoadContainer)

%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMarkers)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLock)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockLock)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockRevolute)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockSpherical)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockCylindrical)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockPrismatic)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockPointPlane)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockPointLine)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockOldham)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockFree)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockAlign)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockParallel)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLockPerpend)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMate)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMateGeneric)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMatePlane)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMateCoaxial)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMateSpherical)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMateXdistance)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMateParallel)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMateOrthogonal)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkMateFix)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkGear)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkDistance)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkLinActuator)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkPulley)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkScrew)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkSpring)
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkPointSpline) 
%DefChSharedPtrDynamicDowncast(ChLink, ChLinkTrajectory)

%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Const)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_ConstAcc)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Derive)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Fillet3)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Integrate)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Mirror)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Mocap)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Noise)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Operation)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Oscilloscope)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Poly)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Poly345)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Ramp)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Recorder)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Repeat)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Sequence)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Sigma)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Sine)

%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaft)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsBody)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsCouple)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsClutch)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsMotor)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsTorsionSpring)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsPlanetary)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsTorqueBase)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsThermalEngine)

%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadCustom)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadCustomMultiple)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyForce)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyTorque)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyBody)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyBodyTorque)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyBodyBushingSpherical)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyBodyBushingPlastic)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyBodyBushingMate)
%DefChSharedPtrDynamicDowncast(ChLoadBase, ChLoadBodyBodyBushingGeneric)

%DefChSharedPtrDynamicDowncast(ChGeometry, ChTriangleMeshConnected)
%DefChSharedPtrDynamicDowncast(ChGeometry, ChTriangleMeshSoup)

// .. to complete


//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

%inline %{

	// Create a custom ChLog class for logging directly in the Python shell,
	// because the default ChLog was redirecting to std::cout that is not 
	// necessarily the console display of python.
namespace chrono
{
class ChLogPython : public ChLog 
{
public:
	ChLogPython() {}
	virtual ~ChLogPython() {};
			/// Redirect output stream to file wrapper.
	virtual void	Output(const char* data, size_t n) 
		{ 
				char buffer[1000];
				if (n>999) 
					n=999;
				strncpy(buffer, data, n);
				buffer[n]=0;
				PySys_WriteStdout("%s", buffer);
		}
private:
};
};

%}




//
// INITIALIZATION CODE THAT IS EXECUTED AT THE STARTING OF TEH PYTHON UNIT
//

%init %{

		// Create a custom logger to be used all times the GetLog() 
		// funciton is used in C::E to print something. 
	static chrono::ChLogPython static_cout_logger;
	SetLog(static_cout_logger);

%}


//
// ADD PYTHON CODE
//


%pythoncode %{

def ImportSolidWorksSystem(mpath):
    import builtins
    import imp
    import os

    mdirname, mmodulename= os.path.split(mpath)

    builtins.exported_system_relpath = mdirname + "/"

    fp, pathname, description = imp.find_module(mmodulename,[builtins.exported_system_relpath])
    try:
        imported_mod = imp.load_module('imported_mod', fp, pathname, description)
    finally:
        if fp:
            fp.close()

    return imported_mod.exported_items

%}


//  
//%shared_ptr(chrono::ChSystem)
