//////////////////////////////////////////////////
//  
//   ChModuleFem.i
//
//   SWIG configuration file.
//   This is processed by SWIG to create the C::E
//   wrapper for Python.
//
///////////////////////////////////////////////////



// Define the module to be used in Python when typing 
//  'import ChronoEngine_PYTHON_fem as fem'


%module(directors="1") ChronoEngine_PYTHON_fem


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

#include "unit_FEM/ChNodeFEMbase.h"
#include "unit_FEM/ChNodeFEMxyz.h"
#include "unit_FEM/ChNodeFEMxyzP.h"
#include "unit_FEM/ChNodeFEMxyzrot.h"
#include "unit_FEM/ChElementBase.h"
#include "unit_FEM/ChElementGeneric.h"
#include "unit_FEM/ChElementBeam.h"
#include "unit_FEM/ChElementBeamEuler.h"
#include "unit_FEM/ChElementTetra_4.h"
#include "unit_FEM/ChElementTetra_10.h"
#include "unit_FEM/ChElementHexa_8.h"
#include "unit_FEM/ChElementHexa_20.h"
#include "unit_FEM/ChBuilderBeam.h"
#include "unit_FEM/ChMesh.h"
#include "physics/ChContinuumMaterial.h"
#include "unit_FEM/ChContinuumElectrostatics.h"
#include "unit_FEM/ChContinuumThermal.h"

using namespace chrono;
using namespace fem;


%}


// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApiFem 


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
%include "../unit_FEM/ChElementBase.h"
%DefChSharedPtr(chrono::fem::,ChElementBase)

%include "../unit_FEM/ChElementGeneric.h"
%DefChSharedPtr(chrono::fem::,ChElementGeneric)

%include "../unit_FEM/ChBeamSection.h"
%DefChSharedPtr(chrono::fem::,ChBeamSection)
%DefChSharedPtr(chrono::fem::,ChBeamSectionAdvanced)

%include "../unit_FEM/ChElementBeam.h"
%DefChSharedPtr(chrono::fem::,ChElementBeam)

%include "../unit_FEM/ChElementBeamEuler.h"
%DefChSharedPtr(chrono::fem::,ChElementBeamEuler)

%include "../physics/ChContinuumMaterial.h"
%DefChSharedPtr(chrono::fem::,ChContinuumMaterial)
%DefChSharedPtr(chrono::fem::,ChContinuumElastic)
%DefChSharedPtr(chrono::fem::,ChContinuumElastoplastic)
%DefChSharedPtr(chrono::fem::,ChContinuumPlasticVonMises)
%DefChSharedPtr(chrono::fem::,ChContinuumDruckerPrager)

%include "../unit_FEM/ChContinuumPoisson3D.h"
%DefChSharedPtr(chrono::fem::,ChContinuumPoisson3D)

%include "../unit_FEM/ChContinuumElectrostatics.h"
%DefChSharedPtr(chrono::fem::,ChContinuumElectrostatics)

%include "../unit_FEM/ChContinuumThermal.h"
%DefChSharedPtr(chrono::fem::,ChContinuumThermal)

//%include "../unit_FEM/ChElementTetrahedron.h"  	// pure virtual: do not create Python obj
//%DefChSharedPtr(chrono::fem::,ChElementTetrahedron)

%include "../unit_FEM/ChElementTetra_4.h"
%DefChSharedPtr(chrono::fem::,ChElementTetrahedron)
%DefChSharedPtr(chrono::fem::,ChElementTetra_4)

%include "../unit_FEM/ChElementTetra_10.h"
%DefChSharedPtr(chrono::fem::,ChElementTetra_10)

//%include "../unit_FEM/ChElementHexahedron.h"		// pure virtual: do not create Python obj
//%DefChSharedPtr(chrono::fem::,ChElementHexahedron)

%include "../unit_FEM/ChElementHexa_8.h"
%DefChSharedPtr(chrono::fem::,ChElementHexahedron)
%DefChSharedPtr(chrono::fem::,ChElementHexa_8)

%include "../unit_FEM/ChElementHexa_20.h"
%DefChSharedPtr(chrono::fem::,ChElementHexa_20)

%include "../unit_FEM/ChNodeFEMbase.h"
%DefChSharedPtr(chrono::fem::,ChNodeFEMbase)

%include "../unit_FEM/ChNodeFEMxyz.h"
%DefChSharedPtr(chrono::fem::,ChNodeFEMxyz)

%include "../unit_FEM/ChNodeFEMxyzP.h"
%DefChSharedPtr(chrono::fem::,ChNodeFEMxyzP)

%include "../unit_FEM/ChNodeFEMxyzrot.h"
%DefChSharedPtr(chrono::fem::,ChNodeFEMxyzrot)

%include "../unit_FEM/ChBuilderBeam.h"
//%DefChSharedPtr(chrono::fem::,ChBuilderBeam)

%include "../unit_FEM/ChMesh.h"
%DefChSharedPtr(chrono::fem::,ChMesh)


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

//%DefChSharedPtrCast(chrono::fem::ChNodeFEMbase, chrono::ChNodeBase)
%DefChSharedPtrCast(chrono::fem::ChNodeFEMxyz, chrono::fem::ChNodeFEMbase)
%DefChSharedPtrCast(chrono::fem::ChNodeFEMxyzP, chrono::fem::ChNodeFEMbase)
%DefChSharedPtrCast(chrono::fem::ChNodeFEMxyzrot, chrono::fem::ChNodeFEMbase)
%DefChSharedPtrCast(chrono::fem::ChNodeFEMxyzrot, chrono::ChBodyFrame)
%DefChSharedPtrCast(chrono::fem::ChElementGeneric, chrono::ChElementBase)
%DefChSharedPtrCast(chrono::fem::ChElementBeam, chrono::ChElementGeneric)
%DefChSharedPtrCast(chrono::fem::ChElementBeamEuler, chrono::ChElementBeam)
%DefChSharedPtrCast(chrono::fem::ChElementTetrahedron, chrono::ChElementGeneric)
%DefChSharedPtrCast(chrono::fem::ChElementTetra_4, chrono::ChElementTetrahedron)
%DefChSharedPtrCast(chrono::fem::ChElementTetra_10, chrono::ChElementTetrahedron)
%DefChSharedPtrCast(chrono::fem::ChElementHexahedron, chrono::ChElementGeneric)
%DefChSharedPtrCast(chrono::fem::ChElementHexa_8, chrono::ChElementHexahedron)
%DefChSharedPtrCast(chrono::fem::ChElementHexa_20, chrono::ChElementHexahedron)
%DefChSharedPtrCast(chrono::fem::ChContinuumElastic, chrono::ChContinuumMaterial)
%DefChSharedPtrCast(chrono::fem::ChContinuumElastoplastic, chrono::ChContinuumElastic)
%DefChSharedPtrCast(chrono::fem::ChContinuumPlasticVonMises, chrono::ChContinuumElastoplastic)
%DefChSharedPtrCast(chrono::fem::ChContinuumDruckerPrager, chrono::ChContinuumElastoplastic)
%DefChSharedPtrCast(chrono::fem::ChMesh, chrono::ChPhysicsItem)

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
%DefChSharedPtrDynamicDowncast(ChNodeFEMbase,ChNodeFEMxyz)
%DefChSharedPtrDynamicDowncast(ChNodeFEMbase,ChNodeFEMxyzP)
%DefChSharedPtrDynamicDowncast(ChNodeFEMbase,ChNodeFEMxyzrot)

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


