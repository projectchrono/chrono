//////////////////////////////////////////////////
//  
//   ChModuleCascade.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import pychrono.cascade'


%module(directors="1") cascade


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

//#include "chrono_cascade/ChApiCASCADE.h"
//#include "chrono_cascade/ChCascadeShapeAsset.h"
#include "chrono_cascade/ChBodyEasyCascade.h"
#include "chrono_cascade/ChCascadeDoc.h"

using namespace chrono;
using namespace chrono::cascade;

%}


// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApiCASCADE 
#define ChApi 
#define Handle(ClassName)  Handle_##ClassName
#define Standard_EXPORT

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

// from core module:
%shared_ptr(chrono::ChFrame<double>)
%shared_ptr(chrono::ChFrameMoving<double>)
%shared_ptr(chrono::ChObj)
%shared_ptr(chrono::ChPhysicsItem)

%shared_ptr(chrono::cascade::ChBodyEasyCascade)
%shared_ptr(chrono::cascade::ChCascadeShapeAsset)


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

//This import gave problems with different PythonOCC versions. Is it 
//%import(module = "OCC.Core.TopoDS") "TopoDS_Shape.hxx"

%import(module = "pychrono.core")  "ChClassFactory.i"
%import(module = "pychrono.core")  "ChObject.i"
%import(module = "pychrono.core")  "ChVector.i"
%import(module = "pychrono.core")  "ChQuaternion.i"
%import(module = "pychrono.core")  "ChCoordsys.i"
%import(module = "pychrono.core")  "ChFrame.i"
%import(module = "pychrono.core")  "ChFrameMoving.i"
%import(module = "pychrono.core") "ChAsset.i"
%import(module = "pychrono.core") "../chrono/physics/ChPhysicsItem.h"
%import(module = "pychrono.core") "ChBodyFrame.i"
%import(module = "pychrono.core") "ChBody.i"
%import(module = "pychrono.core") "ChBodyAuxRef.i"

%include "../chrono_cascade/ChCascadeShapeAsset.h"
%include "../chrono_cascade/ChBodyEasyCascade.h"
%include "../chrono_cascade/ChCascadeDoc.h"


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

//%DefSharedPtrDynamicDowncast(ChSolver,ChSolverMKL) 


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


