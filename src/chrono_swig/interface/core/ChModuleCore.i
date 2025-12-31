// =====================================================================================
//  
// ChModuleCore.i
// Create the Python and C# wrappers for the core Chrono module.
//
// ATTENTION: 
// Must be included from another SWIG interface file which defines the module.
//
// =====================================================================================

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


// For optional casting of polimorphic objects:
%include "../chrono_cast.i" 

// For supporting shared pointers:
%include <std_shared_ptr.i>


// Include C++ headers this way...

%{
#include <typeindex>
#include <cstddef>

#include "chrono/ChConfig.h"

#include "chrono/core/ChApiCE.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChNodeBase.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/physics/ChLoadsNodeXYZ.h"
#include "chrono/physics/ChIndexedNodes.h"

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono/fea/ChMesh.h"


#include "chrono/collision/ChCollisionShape.h"
#include "chrono/collision/ChCollisionShapes.h"
#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#ifdef CHRONO_COLLISION
#include "chrono/collision/multicore/ChCollisionSystemMulticore.h"
#endif

#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/core/ChBezierCurve.h"
#include "Eigen/src/Core/util/Memory.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/input_output/ChUtilsInputOutput.h"
#include "chrono/utils/ChConstants.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono/input_output/ChOutput.h"
#include "chrono/input_output/ChCheckpoint.h"

using namespace chrono;
using namespace chrono::utils;
using namespace chrono::fea;
%}


// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define CH_DEPRECATED(msg)

#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
// I'm not certain if this is a python issue also, so wrap only for csharp for now
// Stop SWIG from generating a baked literal for the __FILENAME__ macro
%ignore __FILENAME__;
%inline %{
// if needed there can be a pinvoke func instead of a broken char constant that SWIG produces from the macro
inline const char* ChUtils_GetFilename() {
    return __FILE__ + SOURCE_PATH_SIZE;
}
%}
// make the new function visible as a public static for c#
%csmethodmodifiers ChUtils_GetFilename "public static"
#endif             // --------------------------------------------------------------------- CSHARP

%ignore CH_ENUM_MAPPER_BEGIN;
%ignore CH_ENUM_VAL;
%ignore CH_ENUM_MAPPER_END;
%ignore CH_CLASS_VERSION;
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
%shared_ptr(chrono::ChBezierCurve)
%shared_ptr(chrono::ChGlyphs)
%shared_ptr(chrono::ChCamera) 
%shared_ptr(chrono::ChVisualMaterial)
%shared_ptr(chrono::ChVisualSystem)

%shared_ptr(chrono::ChFunction)
%shared_ptr(chrono::ChFunctionBSpline)
%shared_ptr(chrono::ChFunctionConst)
%shared_ptr(chrono::ChFunctionConstAcc)
%shared_ptr(chrono::ChFunctionCycloidal)
%shared_ptr(chrono::ChFunctionDerivative)
%shared_ptr(chrono::ChFunctionFillet3)
%shared_ptr(chrono::ChFunctionIntegral)
%shared_ptr(chrono::ChFunctionMirror)
%shared_ptr(chrono::ChFunctionOperator)
%shared_ptr(chrono::ChFunctionPoly)
%shared_ptr(chrono::ChFunctionPoly345)
%shared_ptr(chrono::ChFunctionRamp)
%shared_ptr(chrono::ChFunctionInterp)
%shared_ptr(chrono::ChFunctionRepeat)
%shared_ptr(chrono::ChFunctionSequence)
%shared_ptr(chrono::ChFunctionPoly23)
%shared_ptr(chrono::ChFunctionSine)
%shared_ptr(chrono::ChFunctionSineStep)
%shared_ptr(chrono::ChFunctionSetpoint)
%shared_ptr(chrono::ChFunctionSetpointCallback)

%shared_ptr(chrono::ChFunctionRotation)
%shared_ptr(chrono::ChFunctionRotationAxis)
%shared_ptr(chrono::ChFunctionRotationABCFunctions)
%shared_ptr(chrono::ChFunctionRotationSetpoint)
%shared_ptr(chrono::ChFunctionRotationBSpline)
%shared_ptr(chrono::ChFunctionRotationSQUAD)
%shared_ptr(chrono::ChFunctionPosition)
%shared_ptr(chrono::ChFunctionPositionLine)
%shared_ptr(chrono::ChFunctionPositionSetpoint)
%shared_ptr(chrono::ChFunctionPositionXYZFunctions)

%shared_ptr(chrono::ChObj)
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChContactable)
%shared_ptr(chrono::ChContactable_1vars)
%shared_ptr(chrono::ChContactable_2vars)
%shared_ptr(chrono::ChContactable_3vars)
%shared_ptr(chrono::ChIndexedNodes)
%shared_ptr(chrono::ChContactMaterialNSC)
%shared_ptr(chrono::ChContactMaterialSMC)
%shared_ptr(chrono::ChContactMaterial)
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
%shared_ptr(chrono::ChFeeder)
%shared_ptr(chrono::ChParticle)
%shared_ptr(chrono::ChParticleBase)
%shared_ptr(chrono::ChIndexedParticles)
%shared_ptr(chrono::ChSystemNSC)
%shared_ptr(chrono::ChSystemSMC)
%shared_ptr(chrono::ChContactContainer)
%shared_ptr(chrono::ChProximityContainer)

%shared_ptr(chrono::fea::ChMesh)

%shared_ptr(chrono::ChCollisionShape)
%shared_ptr(chrono::ChCollisionModel)

%shared_ptr(chrono::ChCollisionSystem)
%shared_ptr(chrono::ChCollisionSystemBullet)
#ifdef CHRONO_COLLISION
%shared_ptr(chrono::ChCollisionSystemMulticore)
#endif

%shared_ptr(chrono::ChCollisionSystem::BroadphaseCallback)
%shared_ptr(chrono::ChCollisionSystem::NarrowphaseCallback)
%shared_ptr(chrono::ChCollisionSystem::VisualizationCallback)

%shared_ptr(chrono::ChLinkMarkers)
%shared_ptr(chrono::ChLinkLimit)

%shared_ptr(chrono::ChLinkDistance)
%shared_ptr(chrono::ChLinkLockGear)
%shared_ptr(chrono::ChLinkLockLinActuator)
%shared_ptr(chrono::ChLinkMate)
%shared_ptr(chrono::ChLinkMateGeneric)
%shared_ptr(chrono::ChLinkMatePlanar)
%shared_ptr(chrono::ChLinkMateCylindrical)
%shared_ptr(chrono::ChLinkMateSpherical)
%shared_ptr(chrono::ChLinkMateDistanceZ)
%shared_ptr(chrono::ChLinkMateParallel)
%shared_ptr(chrono::ChLinkMateOrthogonal)
%shared_ptr(chrono::ChLinkMateFix)
%shared_ptr(chrono::ChLinkMateRevolute)
%shared_ptr(chrono::ChLinkMatePrismatic)
%shared_ptr(chrono::ChLinkMateRackPinion)
%shared_ptr(chrono::ChLinkLockPulley)
%shared_ptr(chrono::ChLinkRevolute)
%shared_ptr(chrono::ChLinkRevoluteSpherical)
%shared_ptr(chrono::ChLinkLockScrew)
%shared_ptr(chrono::ChLinkTSDA)
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
%shared_ptr(chrono::ChLinkLockTrajectory)
%shared_ptr(chrono::ChLinkLockPointSpline)
%shared_ptr(chrono::ChLinkMotionImposed)
%shared_ptr(chrono::ChLinkBushing)
%shared_ptr(chrono::ChJoint)

// Cross-inheritance for callbacks that must be inherited.
// Put these 'director' features _before_ class wrapping declaration.

%feature("director") chrono::ChCollisionSystem::BroadphaseCallback;
%feature("director") chrono::ChCollisionSystem::NarrowphaseCallback;
%feature("director") chrono::ChCollisionSystem::VisualizationCallback;

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
%include "ChClassFactory.i"
%include "../../../chrono/core/ChDataPath.h"
//%include "ChArchive.i"
%include "ChMatrix.i"
%include "ChVector2.i"
#define ChVector2d ChVector2d
%include "ChVector3.i"
#define ChVector3d ChVector3d
%include "ChQuaternion.i"
%include "ChTensors.i"
%include "../../../chrono/core/ChBezierCurve.h"
#define Quaternion ChQuaternion<double>
%include "ChMatrix33.i"
%include "ChCoordsys.i"
#define Coordsys ChCoordsys<double>
%include "ChFrame.i"
%include "ChFrameMoving.i"
%include "ChRandom.i"
%include "ChTimer.i"
%include "ChRealtimeStep.i"

%include "ChMassProperties.i" // needed by ChTriangleMeshConnected in ChGeometry.i

// geometry/   classes
%include "ChGeometry.i"


//collision classes
%include "ChContactMaterial.i"
%include "ChCollisionShape.i"
%include "ChCollisionModel.i"
%include "../../../chrono/collision/ChCollisionShape.h"
%include "../../../chrono/collision/ChCollisionShapes.h"
%include "../../../chrono/collision/ChCollisionModel.h"
%include "ChCollisionInfo.i"
%include "../../../chrono/collision/ChCollisionSystem.h"
%include "../../../chrono/collision/bullet/ChCollisionSystemBullet.h"
#ifdef CHRONO_COLLISION
%include "../../../chrono/collision/multicore/ChCollisionSystemMulticore.h"
#endif

// utils classes
%include "ChBodyGeometry.i"
%include "ChUtilsSamplers.i"

// functions/   classes
%include "ChFunction.i"

%include "../../../chrono/fea/ChMesh.h"


// assets
%include "ChColor.i"
%include "ChColormap.i"
%include "ChVisualMaterial.i"
%include "ChVisualShape.i"
%include "ChVisualModel.i"
%include "ChTexture.i"
%include "ChCamera.i"
%include "../../../chrono/assets/ChGlyphs.h"

// physics/  classes
%include "ChControllers.i"
%include "ChLoadable.i"
%include "ChObject.i"
%include "ChPhysicsItem.i"
%include "../../../chrono/physics/ChIndexedNodes.h"
%include "../../../chrono/physics/ChNodeBase.h"
%include "ChNodeXYZ.i"
%include "ChBodyFrame.i"
%include "ChMarker.i"
%include "ChForce.i"
%include "ChBody.i"
%include "ChBodyAuxRef.i"
%include "../../../chrono/physics/ChBodyEasy.h"
%include "ChConveyor.i"
%include "ChFeeder.i"
%include "ChIndexedParticles.i"
%include "ChParticleCloud.i"
%include "ChLinkBase.i"
%include "ChLink.i"
%include "ChLinkMarkers.i"
%include "ChLinkLimit.i"
%include "ChLinkForce.i"
%include "ChLinkLock.i"
%include "ChLinkMate.i"
%include "ChLinkDistance.i"
%include "ChLinkLockLinActuator.i"
%include "ChLinkLockPulley.i"
%include "ChLinkLockScrew.i"
%include "ChLinkTSDA.i"
%include "ChLinkRSDA.i"
%include "ChLinkLockGear.i"
%include "ChLinkRevolute.i"
%include "ChLinkRevoluteSpherical.i"
%include "ChLinkUniversal.i" 
%include "ChLinkLockTrajectory.i" 
%include "ChLinkLockPointSpline.i"
%include "../../../chrono/physics/ChLinkMotionImposed.h"
%include "ChAssembly.i"
%include "ChTimestepper.i"
%include "ChSolver.i"
%include "ChContactContainer.i"
%include "ChSystem.i"
%include "ChSystemNSC.i"
%include "ChSystemSMC.i"
%include "ChProximityContainer.i"
%include "ChLoader.i"
%include "ChLoad.i"
%include "ChLoadContainer.i"
%include "ChShaft.i"
%include "ChShaftMotor.i"
%include "ChLinkMotor.i"
%include "ChLinkBushing.i"
%include "../../../chrono/physics/ChJoint.h"

%include "ChVisualSystem.i" // ChVisualSystem needs to be put after ChSystem

// Utils
// for hulls and meshing
%include "../../../chrono/collision/ChConvexDecomposition.h"

%include "../../../chrono/input_output/ChWriterCSV.h"
%include "../../../chrono/input_output/ChUtilsInputOutput.h"
%include "../../../chrono/utils/ChConstants.h"
%include "../../../chrono/utils/ChUtils.h"
%include "../../../chrono/utils/ChFilters.h"
%include "../../../chrono/utils/ChUtilsCreators.h"
%include "../../../chrono/utils/ChUtilsGeometry.h"

%include "../../../chrono/input_output/ChOutput.h"
%include "../../../chrono/input_output/ChCheckpoint.h"

%include "ChParticleFactory.i"
//
// C- CASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. So in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefSharedPtrDynamicCast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

%DefSharedPtrDynamicCast(chrono, ChContactable, ChBody)
%DefSharedPtrDynamicCast(chrono, ChContactable, ChBodyAuxRef)

%DefSharedPtrDynamicCast(chrono, ChLoadable, ChBody)
%DefSharedPtrDynamicCast(chrono, ChLoadable, ChNodeBase)

%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeFEA)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeModelFile)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeTriangleMesh)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeSphere)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeEllipsoid)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeBarrel)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeBox)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeCone)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeCylinder)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeCapsule)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeRoundedCylinder)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeRoundedBox)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapePath)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeLine)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapePointPoint)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeRotSpring)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeSegment)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeSpring)
%DefSharedPtrDynamicCast(chrono, ChVisualShape, ChVisualShapeSurface)


%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeArc2D)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeBarrel)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeBox)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeCapsule)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeCone)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeConvexHull)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeCylinder)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeCylindricalShell)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeEllipsoid)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapePath2D)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapePoint)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeRoundedBox)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeRoundedCylinder)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeSegment2D)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeSphere)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeTriangle)
%DefSharedPtrDynamicCast(chrono, ChCollisionShape, ChCollisionShapeTriangleMesh)


%DefSharedPtrDynamicCast(chrono, ChCollisionSystem, ChCollisionSystemBullet)
#ifdef CHRONO_COLLISION
%DefSharedPtrDynamicCast(chrono, ChCollisionSystem, ChCollisionSystemMulticore)
#endif

%DefSharedPtrDynamicCast(chrono, ChBodyFrame, ChBody)
%DefSharedPtrDynamicCast(chrono, ChBodyFrame, ChBodyAuxRef)
%DefSharedPtrDynamicCast(chrono, ChBodyFrame, ChConveyor)
%DefSharedPtrDynamicCast(chrono, ChBody, ChBodyFrame)
%DefSharedPtrDynamicCast(chrono, ChBodyAuxRef, ChBodyFrame)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChBody)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChConveyor)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChBodyAuxRef)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChIndexedParticles)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChParticleCloud)
%DefSharedPtrDynamicCast(chrono, ChParticleCloud, ChIndexedParticles)

%DefSharedPtrDynamicCast(chrono, ChNodeBase, ChNodeXYZ)

%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLink)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMarkers)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLock)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockLock)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockRevolute)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockSpherical)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockCylindrical)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockPrismatic)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockPointPlane)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockPointLine)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockOldham)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockFree)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockAlign)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockParallel)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockPerpend)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMate)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateGeneric)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMatePlanar)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateCylindrical)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateSpherical)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateDistanceZ)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateParallel)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateOrthogonal)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateFix)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMateRevolute)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMatePrismatic)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockGear)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkDistance)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockLinActuator)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockPulley)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkLockScrew)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkTSDA)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkRSDA)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotor)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorLinear)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorLinearDriveline)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorLinearForce)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorLinearPosition)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorLinearSpeed)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorRotation)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorRotationAngle)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorRotationDriveline)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorRotationSpeed)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkMotorRotationTorque)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLoadContainer)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChLinkBushing)

%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMarkers)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLock)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockLock)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockRevolute)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockSpherical)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockCylindrical)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockPrismatic)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockPointPlane)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockPointLine)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockOldham)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockFree)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockAlign)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockParallel)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockPerpend)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMate)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateGeneric)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMatePlanar)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateCylindrical)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateSpherical)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateDistanceZ)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateParallel)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateOrthogonal)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateFix)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMateRevolute)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkMatePrismatic)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockGear)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkDistance)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockLinActuator)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockPulley)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockScrew)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkTSDA)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockPointSpline) 
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkLockTrajectory)
%DefSharedPtrDynamicCast(chrono, ChLink, ChLinkBushing)

%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaft)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftBodyRotation)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftBodyTranslation)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsCouple)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsClutch)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsMotor)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsTorsionSpring)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsAppliedTorque)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsPlanetary)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsTorque)
%DefSharedPtrDynamicCast(chrono, ChPhysicsItem, ChShaftsThermalEngine)

%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadCustom)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadCustomMultiple)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyForce)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyTorque)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyInertia)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyBody)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyBodyTorque)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyBodyBushingSpherical)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyBodyBushingPlastic)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyBodyBushingMate)
%DefSharedPtrDynamicCast(chrono, ChLoadBase, ChLoadBodyBodyBushingGeneric)

%DefSharedPtrDynamicCast(chrono, ChGeometry, ChTriangleMeshConnected)
%DefSharedPtrDynamicCast(chrono, ChGeometry, ChTriangleMeshSoup)

// .. to complete


#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

%extend chrono::ChBezierCurveTracker {
public:
  double ClosestPointCurvature(const chrono::ChVector3d& loc, chrono::ChFrame<>& tnb) {
    double curvature;
    int foo = $self->CalcClosestPoint(loc, tnb, curvature);
    return curvature;
  }
};


//
// ADD PYTHON CODE
//


%pythoncode %{

def ImportSolidWorksSystem(mpath):
    import builtins
    import sys
    import os

    mdirname, mmodulename = os.path.split(mpath)

    builtins.exported_system_relpath = mdirname + "/"

    try:
        if sys.version_info[0] == 3 and sys.version_info[1] >= 5:
            import importlib.util
            spec = importlib.util.spec_from_file_location(mmodulename, mpath)
            imported_mod = importlib.util.module_from_spec(spec)
            sys.modules[mmodulename] = imported_mod
            spec.loader.exec_module(imported_mod)
        elif sys.version_info[0] == 3 and sys.version_info[1] < 5:
            import importlib.machinery
            loader = importlib.machinery.SourceFileLoader(mmodulename, mpath)
            imported_mod = loader.load_module()
        else:
            raise Exception("Python version not supported. Please upgrade it.")
    except Exception as e:
        print(f"Error loading module: {e}")
        return None

    return imported_mod.exported_items


%}

#endif             // --------------------------------------------------------------------- PYTHON
