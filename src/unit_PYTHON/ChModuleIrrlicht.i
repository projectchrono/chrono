//////////////////////////////////////////////////
//  
//   ChModuleIrrlicht.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import ChronoEngine_PYTHON_irrlicht as ceirrlicht'


%module(directors="1") ChronoEngine_PYTHON_irrlicht


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

// The complete support of smart shared pointers
%include "chrono_shared_ptr.i" 




// Include C++ headers this way...

%{

//#include "unit_IRRLICHT/ChXxxyyyzzz.h"
#include <irrlicht.h>
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "irrlicht_interface/ChIrrAssetConverter.h"

using namespace chrono;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

%}


// Undefine ChApi and other macros that otherwise SWIG gives a syntax error
#define ChApiIrr 
#define _IRR_DEPRECATED_ //

// Include other .i configuration files for SWIG. 
// These are divided in many .i files, each per a
// different c++ class, when possible.

%include "std_string.i"
%include "std_vector.i"
%include "typemaps.i"
%include "wchar.i"

// This is to enable references to double,int,etc. types in function parameters
%pointer_class(int,int_ptr);
%pointer_class(double,double_ptr);
%pointer_class(float,float_ptr);



// IMPORTANT!!!
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
// B by using the %import keyword that tells Swig that somewhere there's
// a type B. Otherwise a name mangling is built anyway, but the runtime is not ok.

//  core/  classes
// %include "ChIrrlichtBase.i" // not existing

%import  "ChSystem.i"
%import  "ChAsset.i"

%include "irrTypes.h"
%include "vector2d.h"
%template(vector2df) irr::core::vector2d<irr::f32>;
%template(vector2di) irr::core::vector2d<irr::s32>;
%include "vector3d.h"
%template(vector3df) irr::core::vector3d<irr::f32>;
%template(vector3di) irr::core::vector3d<irr::s32>;
%include "SColor.h"
%include "IVideoDriver.h"
%include "IrrlichtDevice.h"
%include "ISceneNode.h"
%include "ISceneManager.h"
%include "IGUIEnvironment.h"
%include "dimension2d.h"

%include "ChIrrAppInterface.i"
%include "ChIrrAssetConverter.i"
%include "ChIrrApp.i"
%include "ChIrrNode.i"
%include "ChIrrNodeAsset.i"



//
// UPCASTING OF SHARED POINTERS           (custom inheritance)
//
// Note: SWIG takes care automatically of how to cast from  
// FooDerived* to FooBase* given its swig_cast_info inner structures,
// but this casting does not happen for ChSharedPtr<FooDerived> to
// ChSharedPtr<FooBase> because shared ptrs are different classes (not inherited
// one from the other, just templated versions of a ChSharedPtr<> ).
// A workaround for this problem is using the (undocumented)
// function  %types   , that can work here because each templated version
// of the CSharedPtr has exactly the same data structure, so they can be just
// cast. 
// So, use the %DefChSharedPtrCast(derived,base) macro to enable the upcasting.

%DefChSharedPtrCast(chrono::ChIrrNodeAsset, chrono::ChAsset)

//
// DOWNCASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. 
// Alternatively, in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefChSharedPtrDynamicDowncast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

%DefChSharedPtrDynamicDowncast(ChAsset,ChIrrNodeAsset)


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


