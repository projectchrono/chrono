//////////////////////////////////////////////////
//  
//   ChModuleFea.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import ChronoEngine_PYTHON_fea as fea'


%module(directors="1") ChronoEngine_PYTHON_fea


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

#include "unit_FEA/ChNodeFEAbase.h"
#include "unit_FEA/ChNodeFEAxyz.h"
#include "unit_FEA/ChNodeFEAxyzP.h"
#include "unit_FEA/ChNodeFEAxyzrot.h"
#include "unit_FEA/ChElementBase.h"
#include "unit_FEA/ChElementGeneric.h"
#include "unit_FEA/ChElementBeam.h"
#include "unit_FEA/ChElementBeamEuler.h"
#include "unit_FEA/ChElementTetra_4.h"
#include "unit_FEA/ChElementTetra_10.h"
#include "unit_FEA/ChElementHexa_8.h"
#include "unit_FEA/ChElementHexa_20.h"
#include "unit_FEA/ChBuilderBeam.h"
#include "unit_FEA/ChMesh.h"
#include "physics/ChContinuumMaterial.h"
#include "unit_FEA/ChContinuumElectrostatics.h"
#include "unit_FEA/ChContinuumThermal.h"

using namespace chrono;
using namespace fea;


%}


// Undefine ChApiFea otherwise SWIG gives a syntax error
#define ChApiFea 


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

%import  "ChShared.i"
%import  "ChObject.i"
%import  "ChFrame.i"
%import  "ChFrameMoving.i"
%import  "ChPhysicsItem.i"
%import  "ChBodyFrame.i"

//  core/  classes
%include "../unit_FEA/ChElementBase.h"
%DefChSharedPtr(chrono::fea::,ChElementBase)

%include "../unit_FEA/ChElementGeneric.h"
%DefChSharedPtr(chrono::fea::,ChElementGeneric)

%include "../unit_FEA/ChBeamSection.h"
%DefChSharedPtr(chrono::fea::,ChBeamSection)
%DefChSharedPtr(chrono::fea::,ChBeamSectionAdvanced)

%include "../unit_FEA/ChElementBeam.h"
%DefChSharedPtr(chrono::fea::,ChElementBeam)

%include "../unit_FEA/ChElementBeamEuler.h"
%DefChSharedPtr(chrono::fea::,ChElementBeamEuler)

%include "../physics/ChContinuumMaterial.h"
%DefChSharedPtr(chrono::fea::,ChContinuumMaterial)
%DefChSharedPtr(chrono::fea::,ChContinuumElastic)
%DefChSharedPtr(chrono::fea::,ChContinuumElastoplastic)
%DefChSharedPtr(chrono::fea::,ChContinuumPlasticVonMises)
%DefChSharedPtr(chrono::fea::,ChContinuumDruckerPrager)

%include "../unit_FEA/ChContinuumPoisson3D.h"
%DefChSharedPtr(chrono::fea::,ChContinuumPoisson3D)

%include "../unit_FEA/ChContinuumElectrostatics.h"
%DefChSharedPtr(chrono::fea::,ChContinuumElectrostatics)

%include "../unit_FEA/ChContinuumThermal.h"
%DefChSharedPtr(chrono::fea::,ChContinuumThermal)

//%include "../unit_FEA/ChElementTetrahedron.h"  	// pure virtual: do not create Python obj
//%DefChSharedPtr(chrono::fea::,ChElementTetrahedron)

%include "../unit_FEA/ChElementTetra_4.h"
%DefChSharedPtr(chrono::fea::,ChElementTetrahedron)
%DefChSharedPtr(chrono::fea::,ChElementTetra_4)

%include "../unit_FEA/ChElementTetra_10.h"
%DefChSharedPtr(chrono::fea::,ChElementTetra_10)

//%include "../unit_FEA/ChElementHexahedron.h"		// pure virtual: do not create Python obj
//%DefChSharedPtr(chrono::fea::,ChElementHexahedron)

%include "../unit_FEA/ChElementHexa_8.h"
%DefChSharedPtr(chrono::fea::,ChElementHexahedron)
%DefChSharedPtr(chrono::fea::,ChElementHexa_8)

%include "../unit_FEA/ChElementHexa_20.h"
%DefChSharedPtr(chrono::fea::,ChElementHexa_20)

%include "../unit_FEA/ChNodeFEAbase.h"
%DefChSharedPtr(chrono::fea::,ChNodeFEAbase)

%include "../unit_FEA/ChNodeFEAxyz.h"
%DefChSharedPtr(chrono::fea::,ChNodeFEAxyz)

%include "../unit_FEA/ChNodeFEAxyzP.h"
%DefChSharedPtr(chrono::fea::,ChNodeFEAxyzP)

%include "../unit_FEA/ChNodeFEAxyzrot.h"
%DefChSharedPtr(chrono::fea::,ChNodeFEAxyzrot)

%include "../unit_FEA/ChBuilderBeam.h"
//%DefChSharedPtr(chrono::fea::,ChBuilderBeam)

%include "../unit_FEA/ChMesh.h"
%DefChSharedPtr(chrono::fea::,ChMesh)


//%include "ChPostProcessBase.i"
//%include "ChPovRay.i"

//%import  "ChColor.i"
//%include "ChPovRayAssetCustom.i"



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

//%DefChSharedPtrCast(chrono::fea::ChNodeFEAbase, chrono::ChNodeBase)
%DefChSharedPtrCast(chrono::fea::ChNodeFEAxyz, chrono::fea::ChNodeFEAbase)
%DefChSharedPtrCast(chrono::fea::ChNodeFEAxyzP, chrono::fea::ChNodeFEAbase)
%DefChSharedPtrCast(chrono::fea::ChNodeFEAxyzrot, chrono::fea::ChNodeFEAbase)
%DefChSharedPtrCast(chrono::fea::ChNodeFEAxyzrot, chrono::ChBodyFrame)
%DefChSharedPtrCast(chrono::fea::ChElementGeneric, chrono::fea::ChElementBase)
%DefChSharedPtrCast(chrono::fea::ChElementBeam, chrono::fea::ChElementGeneric)
%DefChSharedPtrCast(chrono::fea::ChElementBeamEuler, chrono::fea::ChElementBeam)
%DefChSharedPtrCast(chrono::fea::ChElementTetrahedron, chrono::fea::ChElementGeneric)
%DefChSharedPtrCast(chrono::fea::ChElementTetra_4, chrono::fea::ChElementTetrahedron)
%DefChSharedPtrCast(chrono::fea::ChElementTetra_10, chrono::fea::ChElementTetrahedron)
%DefChSharedPtrCast(chrono::fea::ChElementHexahedron, chrono::fea::ChElementGeneric)
%DefChSharedPtrCast(chrono::fea::ChElementHexa_8, chrono::fea::ChElementHexahedron)
%DefChSharedPtrCast(chrono::fea::ChElementHexa_20, chrono::fea::ChElementHexahedron)
%DefChSharedPtrCast(chrono::fea::ChContinuumElastic, chrono::fea::ChContinuumMaterial)
%DefChSharedPtrCast(chrono::fea::ChContinuumElastoplastic, chrono::fea::ChContinuumElastic)
%DefChSharedPtrCast(chrono::fea::ChContinuumPlasticVonMises, chrono::fea::ChContinuumElastoplastic)
%DefChSharedPtrCast(chrono::fea::ChContinuumDruckerPrager, chrono::fea::ChContinuumElastoplastic)
%DefChSharedPtrCast(chrono::fea::ChMesh, chrono::ChPhysicsItem)

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

%DefChSharedPtrDynamicDowncast(ChPhysicsItem,ChMesh)
%DefChSharedPtrDynamicDowncast(ChElementBase,ChElementBeamEuler)
%DefChSharedPtrDynamicDowncast(ChElementBase,ChElementTetra_4)
%DefChSharedPtrDynamicDowncast(ChElementBase,ChElementTetra_10)
%DefChSharedPtrDynamicDowncast(ChElementBase,ChElementHexa_8)
%DefChSharedPtrDynamicDowncast(ChElementBase,ChElementHexa_20)
%DefChSharedPtrDynamicDowncast(ChNodeFEAbase,ChNodeFEAxyz)
%DefChSharedPtrDynamicDowncast(ChNodeFEAbase,ChNodeFEAxyzP)
%DefChSharedPtrDynamicDowncast(ChNodeFEAbase,ChNodeFEAxyzrot)

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


