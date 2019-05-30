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
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleOutput.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChTerrain.h"

#include "chrono_models/ChApiModels.h"

#include "chrono_thirdparty/rapidjson/document.h"


using namespace chrono;
using namespace chrono::vehicle;


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


//%template(vector_ChNodeFEAxyzrot) std::vector< std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> >;
%template(vector_int) std::vector< int >;

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
%shared_ptr(chrono::ChObj)
%shared_ptr(chrono::ChBodyFrame)
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChIndexedNodes)
%shared_ptr(chrono::ChLinkBase)
%shared_ptr(chrono::ChLoadBase)
%shared_ptr(chrono::ChLoadCustom)
%shared_ptr(chrono::ChLoadCustomMultiple)
%shared_ptr(chrono::ChLoadable) 
%shared_ptr(chrono::ChLoadableU) 
%shared_ptr(chrono::ChLoadableUV) 
%shared_ptr(chrono::ChLoadableUVW)
%shared_ptr(chrono::ChNodeBase) 
%shared_ptr(chrono::ChNodeXYZ) 
%shared_ptr(chrono::ChAsset)
%shared_ptr(chrono::ChAssetLevel)
%shared_ptr(chrono::geometry::ChTriangleMeshConnected)
%shared_ptr(chrono::ChBody)
%shared_ptr(chrono::ChSystem)
%shared_ptr(chrono::ChAssembly)
%shared_ptr(chrono::ChShaft)

//from this module:



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
%import(module = "pychrono.core")  "ChCoordsys.i"
%import(module = "pychrono.core")  "ChMatrix.i"
%import(module = "pychrono.core")  "ChBody.i"
%import "ChVisualization.i"
/* Parse the header file to generate wrappers */
%import(module = "pychrono.core") "../chrono/motion_functions/ChFunction_Base.h"
//%import(module = "pychrono.core") "../chrono/assets/ChAsset.h"
//%import(module = "pychrono.core") "../chrono/assets/ChAssetLevel.h"
%import(module = "pychrono.core")  "ChMaterialSurface.i"
%import(module = "pychrono.core") "../chrono/physics/ChContinuumMaterial.h"
%import(module = "pychrono.core") "../chrono/physics/ChPhysicsItem.h"
%import(module = "pychrono.core") "../chrono/physics/ChIndexedNodes.h"
//%import(module = "pychrono.core") "../chrono/physics/ChLoadable.h" // disable because strange error in cxx
%import(module = "pychrono.core") "../chrono/physics/ChLoad.h"
%import(module = "pychrono.core") "../chrono/physics/ChNodeBase.h"
//%import(module = "pychrono.core") "../chrono/physics/ChNodeXYZ.h"
%import(module = "pychrono.core") "../chrono/physics/ChBodyFrame.h"
%import(module = "pychrono.core") "../chrono/physics/ChLinkBase.h"


//  core/  classes
// ChPhysicsItem is imported, try not to include
%include "../chrono/physics/ChPhysicsItem.h"
//%include "../chrono/fea/ChNodeFEAbase.h"
//%include "../chrono/fea/ChElementBase.h"
//%template(vector_ChNodeFEAbase) std::vector< std::shared_ptr<chrono::fea::ChNodeFEAbase> >;
//%template(vector_ChElementBase) std::vector< std::shared_ptr<chrono::fea::ChElementBase> >;

// TODO: what do we say to rapidjson? Not today.
//%include "rapidjson.i"

%include "../chrono_vehicle/ChPart.h"
%include "ChTerrain.i"
%include "ChChassis.i"
// Shaft end suspensions before driveline
//%include "ChSuspensions.i"
//%include "ChShaft.i"
%include "ChDriveline.i"

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


