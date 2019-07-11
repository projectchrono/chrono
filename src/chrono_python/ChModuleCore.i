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
%feature("flatnested", "1");

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
#include "chrono/physics/ChLoadsXYZnode.h"
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
#include "chrono/core/ChBezierCurve.h"

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

//%shared_ptr(chrono::ChColor)
%shared_ptr(chrono::ChObjShapeFile)
%shared_ptr(chrono::ChBoxShape) 
%shared_ptr(chrono::ChSphereShape)
%shared_ptr(chrono::ChEllipsoidShape)
%shared_ptr(chrono::ChCylinderShape)
%shared_ptr(chrono::ChCamera) 
%shared_ptr(chrono::ChLineShape)
%shared_ptr(chrono::ChSurfaceShape)
%shared_ptr(chrono::ChPathShape)
%shared_ptr(chrono::ChPointPointDrawing)
%shared_ptr(chrono::ChPointPointSegment)
%shared_ptr(chrono::ChPointPointSpring)
%shared_ptr(chrono::ChTriangleMeshShape)
%shared_ptr(chrono::ChBezierCurve)

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
%shared_ptr(chrono::ChNodeBase)
%shared_ptr(chrono::ChNodeXYZ)
%shared_ptr(chrono::ChMarker)
%shared_ptr(chrono::ChForce)
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
// shared_ptr macros for ChIntegrable, Chtimestepper and their children classes moved into "ChTimestepper.i"
%shared_ptr(chrono::ChSolver)
%shared_ptr(chrono::ChSystemNSC)
%shared_ptr(chrono::ChSystemSMC)
%shared_ptr(chrono::ChContactContainer)
%shared_ptr(chrono::ChProximityContainer)

%shared_ptr(chrono::ChLinkMarkers)
%shared_ptr(chrono::ChLinkLimit)

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
%shared_ptr(chrono::ChLinkUniversal)
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
%include "../chrono/core/ChBezierCurve.h"
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
%include "ChLoadable.i"
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
%include "ChAssembly.i"
%include "ChTimestepper.i"
%include "../chrono/solver/ChSolver.h"
%include "ChContactContainer.i"
%include "ChSystem.i"
%include "ChSystemNSC.i"
%include "ChSystemSMC.i"
%include "ChProximityContainer.i"
//%import "../chrono/physics/ChLoad.h" // a forward reference done "the %import way" here works ok..

%include "ChLoad.i"
%include "ChLoadContainer.i"

%include "ChShaft.i"
%include "ChShaftMotor.i"
%include "ChLinkMotor.i"

//
// C- DOWNCASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. So in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefSharedPtrDynamicDowncast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

// enable _automatic_ downcasting from ChAsset to derived classes (shared pointers versions)
%downcast_output_sharedptr(chrono::ChAsset, chrono::ChVisualization, chrono::ChObjShapeFile, chrono::ChBoxShape, chrono::ChSphereShape, chrono::ChCylinderShape, chrono::ChTexture, chrono::ChAssetLevel, chrono::ChCamera, chrono::ChColorAsset)

%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChVisualization)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChObjShapeFile)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChBoxShape)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChSphereShape)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChCylinderShape)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChTexture)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChAssetLevel)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChCamera)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChLineShape)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChSurfaceShape)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChPathShape)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChPointPointDrawing)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChPointPointSegment)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChPointPointSpring)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChTriangleMeshShape)
%DefSharedPtrDynamicDowncast(chrono,ChAsset,ChEllipsoidShape)

%DefSharedPtrDynamicDowncast(chrono,ChBodyFrame, ChBody)
%DefSharedPtrDynamicDowncast(chrono,ChBodyFrame, ChBodyAuxRef)
%DefSharedPtrDynamicDowncast(chrono,ChBodyFrame, ChConveyor)
%DefSharedPtrDynamicDowncast(chrono,ChBody, ChBodyFrame)  // <- upcast, for testing & workaround
%DefSharedPtrDynamicDowncast(chrono,ChBodyAuxRef, ChBodyFrame)  // <- upcast, for testing & workaround
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChBody)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChConveyor)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChBodyAuxRef)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChIndexedParticles)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChParticlesClones)

%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLink)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMarkers)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLock)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockLock)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockRevolute)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockSpherical)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockCylindrical)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockPrismatic)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockPointPlane)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockPointLine)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockOldham)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockFree)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockAlign)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockParallel)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLockPerpend)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMate)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMateGeneric)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMatePlane)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMateCoaxial)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMateSpherical)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMateXdistance)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMateParallel)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMateOrthogonal)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMateFix)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkGear)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkDistance)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkLinActuator)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkPulley)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkScrew)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkSpring)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkSpringCB)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotor)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorLinear)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorLinearDriveline)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorLinearForce)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorLinearPosition)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorLinearSpeed)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorRotation)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorRotationAngle)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorRotationDriveline)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorRotationSpeed)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLinkMotorRotationTorque)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChLoadContainer)

%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMarkers)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLock)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockLock)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockRevolute)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockSpherical)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockCylindrical)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockPrismatic)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockPointPlane)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockPointLine)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockOldham)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockFree)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockAlign)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockParallel)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLockPerpend)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMate)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMateGeneric)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMatePlane)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMateCoaxial)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMateSpherical)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMateXdistance)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMateParallel)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMateOrthogonal)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkMateFix)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkGear)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkDistance)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkLinActuator)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkPulley)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkScrew)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkSpring)
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkPointSpline) 
%DefSharedPtrDynamicDowncast(chrono,ChLink, ChLinkTrajectory)

%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Const)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_ConstAcc)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Derive)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Fillet3)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Integrate)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Mirror)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Mocap)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Noise)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Operation)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Oscilloscope)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Poly)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Poly345)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Ramp)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Recorder)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Repeat)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Sequence)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Sigma)
%DefSharedPtrDynamicDowncast(chrono,ChFunction, ChFunction_Sine)

%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaft)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsBody)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsCouple)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsClutch)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsMotor)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsTorsionSpring)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsPlanetary)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsTorqueBase)
%DefSharedPtrDynamicDowncast(chrono,ChPhysicsItem, ChShaftsThermalEngine)

%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadCustom)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadCustomMultiple)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyForce)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyTorque)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyBody)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyBodyTorque)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyBodyBushingSpherical)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyBodyBushingPlastic)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyBodyBushingMate)
%DefSharedPtrDynamicDowncast(chrono,ChLoadBase, ChLoadBodyBodyBushingGeneric)

%DefSharedPtrDynamicDowncast(chrono::geometry,ChGeometry, ChTriangleMeshConnected)
%DefSharedPtrDynamicDowncast(chrono::geometry,ChGeometry, ChTriangleMeshSoup)

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
