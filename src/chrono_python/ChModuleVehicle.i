//////////////////////////////////////////////////
//  
//   ChModuleVehicle.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import pychrono.vehicle'


%module(directors="1") vehicle


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
#include <string>
#include <vector>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsLoads.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChLinkSpringCB.h"
#include "chrono/physics/ChLinkRotSpringCB.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChPhysicsItem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleOutput.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_vehicle/ChPowertrain.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"
#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"
#include "chrono_vehicle/wheeled_vehicle/brake/BrakeSimple.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_thirdparty/rapidjson/document.h"


using namespace chrono;
using namespace chrono::vehicle;

using namespace chrono::vehicle::generic;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::vehicle::sedan;


%}


// Undefine ChApiFea otherwise SWIG gives a syntax error
#define CH_VEHICLE_API 
#define ChApi

#define CH_MODELS_API



// workaround for trouble
//%ignore chrono::fea::ChContactNodeXYZ::ComputeJacobianForContactPart;


// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_vector.i"
%include "typemaps.i"

// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);



%template(vector_int) std::vector< int >;
%template(TerrainForces) std::vector< chrono::vehicle::TerrainForce >;
%template(WheelStates) std::vector< chrono::vehicle::WheelState >;

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

//from core module:
%shared_ptr(chrono::ChFunction)
%shared_ptr(chrono::ChFrame<double>) 
%shared_ptr(chrono::ChFrameMoving<double>)
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChNodeBase) 
%shared_ptr(chrono::ChNodeXYZ) 
%shared_ptr(chrono::ChTriangleMeshShape)
%shared_ptr(chrono::geometry::ChTriangleMeshConnected)
%shared_ptr(chrono::ChLinkSpring)
%shared_ptr(chrono::ChFunction_Recorder)
%shared_ptr(chrono::ChBezierCurve)
%shared_ptr(chrono::ChLinkMarkers)

/*
from this module: pay attention to inheritance in the model namespace (generic, sedan etc). 
If those classes are wrapped, their parents are marked as shared_ptr while they are not, SWIG can't hanlde them.
Before adding a shared_ptr, mark as shared ptr all its inheritance tree in the model namespaces
*/

%shared_ptr(chrono::vehicle::RigidTerrain::Patch)
%shared_ptr(chrono::vehicle::ChPart)
%shared_ptr(chrono::vehicle::ChWheel)
%shared_ptr(chrono::vehicle::Wheel)
%shared_ptr(chrono::vehicle::ChBrakeSimple)
%shared_ptr(chrono::vehicle::ChBrake)
%shared_ptr(chrono::vehicle::BrakeSimple)
%shared_ptr(chrono::vehicle::ChVehicle)
%shared_ptr(chrono::vehicle::ChWheeledVehicle)
%shared_ptr(chrono::vehicle::WheeledVehicle)


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

%import(module = "pychrono.core")  "ChClassFactory.i"
%import(module = "pychrono.core")  "ChObject.i"
%import(module = "pychrono.core")  "ChPhysicsItem.i"
%import(module = "pychrono.core")  "ChVector.i"
%import(module = "pychrono.core")  "ChQuaternion.i"
%import(module = "pychrono.core")  "ChCoordsys.i"
%import(module = "pychrono.core")  "ChFrame.i"
%import(module = "pychrono.core")  "ChFrameMoving.i"
%import(module = "pychrono.core")  "ChTimestepper.i"
%import(module = "pychrono.core")  "ChSystem.i"
//%import(module = "pychrono.core")  "ChSystemNSC.i"
//%import(module = "pychrono.core")  "ChSystemSMC.i"
%import(module = "pychrono.core")  "ChAssembly.i"
%import(module = "pychrono.core")  "ChCoordsys.i"
%import(module = "pychrono.core")  "ChMatrix.i"
%import(module = "pychrono.core")  "ChBodyFrame.i"
%import(module = "pychrono.core")  "ChBody.i"
%import(module = "pychrono.core")  "ChBodyAuxRef.i"
%import(module = "pychrono.core")  "ChLinkBase.i"
%import(module = "pychrono.core")  "ChLinkLock.i"
%import(module = "pychrono.core")  "ChLinkSpringCB.i"
%import(module = "pychrono.core") "ChLoad.i"
%import(module = "pychrono.core") "ChShaft.i"
%import(module = "pychrono.core") "ChAsset.i"
%import(module = "pychrono.core") "ChAssetLevel.i"
%import(module = "pychrono.core")  "ChVisualization.i"
%import(module = "pychrono.core") "../chrono/motion_functions/ChFunction_Base.h"
%import(module = "pychrono.core")  "ChMaterialSurface.i"
%import(module = "pychrono.core") "../chrono/physics/ChContinuumMaterial.h"
%import(module = "pychrono.core") "../chrono/physics/ChPhysicsItem.h"
//%import(module = "pychrono.core") "../chrono/physics/ChLoadable.h" // disable because strange error in cxx

%import(module = "pychrono.core") "../chrono/physics/ChNodeBase.h"
//%import(module = "pychrono.core") "../chrono/physics/ChNodeXYZ.h"
%import(module = "pychrono.core") "../chrono/physics/ChBodyFrame.h"
%import(module = "pychrono.core") "../chrono/physics/ChLinkBase.h"
%import(module = "pychrono.core") "ChTexture.i"
%import(module = "pychrono.core") "../chrono/assets/ChTriangleMeshShape.h"

// TODO: 
//%include "rapidjson.i"

//%include "../chrono_vehicle/ChApiVehicle.h"
%ignore chrono::vehicle::TrackedCollisionFamily::Enum;
%ignore chrono::vehicle::TrackedCollisionFamily::OutputInformation;
%ignore chrono::vehicle::TrackedCollisionFlag::Enum;
%include "../chrono_vehicle/ChSubsysDefs.h"
%include "../chrono_models/vehicle/ChVehicleModelDefs.h"
//TODO: conversion from std::vectors of ChVehicleOutput
%include "../chrono_vehicle/ChVehicleOutput.h"
%include "../chrono_vehicle/ChVehicleModelData.h"
%include "../chrono_vehicle/ChPart.h"
%include "ChPowertrain.i"
%include "ChChassis.i"
%include "../chrono_vehicle/ChVehicle.h"
%include "ChDriver.i"
%include "ChTerrain.i"
//TODO: antirollbar


// Wheeled parts
%include "../chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
%include "../chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
%include "ChSuspension.i"
%include "ChDriveline.i"
%include "ChSteering.i"

%include "../chrono_vehicle/wheeled_vehicle/ChWheel.h"
%include "../chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"
%include "models/WheelModels.i"

%include "../chrono_vehicle/wheeled_vehicle/ChBrake.h"
%include "../chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"
%include "../chrono_vehicle/wheeled_vehicle/brake/BrakeSimple.h"
%include "models/BrakeModels.i"

%include "ChTire.i"

%include "../chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
%include "../chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
%include "models/VehicleModels.i"

/*
Tracked vehicles are not going to be wrapped in the short term
*/

//
// C- DOWNCASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. 
// Alternatively, in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefSharedPtrDynamicDowncast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

//%DefSharedPtrDynamicDowncast2NS(chrono,chrono::fea,ChPhysicsItem,ChMesh)


//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

/*
%inline %{


%}
*/


//
// ADD PYTHON CODE
//

/*
%pythoncode %{

%}
*/


