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
//  'import ChronoEngine_python_fea as fea'


%module(directors="1") ChronoEngine_python_fea


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

#include "chrono_fea/ChNodeFEAbase.h"
#include "chrono_fea/ChNodeFEAxyz.h"
#include "chrono_fea/ChNodeFEAxyzP.h"
#include "chrono_fea/ChNodeFEAxyzrot.h"
#include "chrono_fea/ChElementBase.h"
#include "chrono_fea/ChElementGeneric.h"
#include "chrono_fea/ChElementBeam.h"
#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChElementBeamANCF.h"
#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChElementTetra_10.h"
#include "chrono_fea/ChElementHexa_8.h"
#include "chrono_fea/ChElementHexa_20.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono/physics/ChContinuumMaterial.h"
#include "chrono_fea/ChContinuumElectrostatics.h"
#include "chrono_fea/ChContinuumThermal.h"
#include "chrono_fea/ChContactSurface.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChMeshSurface.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkPointPoint.h"

using namespace chrono;
using namespace chrono::fea;


%}


// Undefine ChApiFea otherwise SWIG gives a syntax error
#define ChApiFea 
#define ChApi

// workaround for trouble
%ignore chrono::fea::ChContactNodeXYZ::ComputeJacobianForContactPart;
%ignore chrono::fea::ChContactTriangleXYZ::ComputeJacobianForContactPart;
%ignore chrono::fea::ChContactNodeXYZROT::ComputeJacobianForContactPart;
%ignore chrono::fea::ChContactTriangleXYZROT::ComputeJacobianForContactPart;

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
%shared_ptr(chrono::ChFrame<double>)
%shared_ptr(chrono::ChFrameMoving<double>)
%shared_ptr(chrono::ChObj)
%shared_ptr(chrono::ChBodyFrame)
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChLinkBase)
%shared_ptr(chrono::fea::ChElementBase)
%shared_ptr(chrono::fea::ChElementGeneric)
%shared_ptr(chrono::fea::ChBeamSection)
%shared_ptr(chrono::fea::ChBeamSectionBasic)
%shared_ptr(chrono::fea::ChBeamSectionCable)
%shared_ptr(chrono::fea::ChBeamSectionAdvanced)
%shared_ptr(chrono::fea::ChElementBeam)
%shared_ptr(chrono::fea::ChElementBeamEuler)
%shared_ptr(chrono::fea::ChElementBeamANCF)
%shared_ptr(chrono::fea::ChContinuumMaterial)
%shared_ptr(chrono::fea::ChContinuumElastic)
//%shared_ptr(chrono::fea::ChContinuumElastoplastic)
//%shared_ptr(chrono::fea::ChContinuumPlasticVonMises)
//%shared_ptr(chrono::fea::ChContinuumDruckerPrager)
%shared_ptr(chrono::fea::ChContinuumMaterial)
%shared_ptr(chrono::fea::ChContinuumElastoplastic)
%shared_ptr(chrono::fea::ChContinuumPlasticVonMises)
%shared_ptr(chrono::fea::ChContinuumDruckerPrager)
%shared_ptr(chrono::fea::ChContinuumPoisson3D)
%shared_ptr(chrono::fea::ChContinuumElectrostatics)
%shared_ptr(chrono::fea::ChContinuumThermal)
%shared_ptr(chrono::fea::ChElementTetrahedron)
%shared_ptr(chrono::fea::ChElementTetra_4)
%shared_ptr(chrono::fea::ChElementTetra_10)
%shared_ptr(chrono::fea::ChElementHexahedron)
%shared_ptr(chrono::fea::ChElementHexa_8)
%shared_ptr(chrono::fea::ChElementHexa_20)
%shared_ptr(chrono::fea::ChNodeFEAbase)
%shared_ptr(chrono::fea::ChNodeFEAxyz)
%shared_ptr(chrono::fea::ChNodeFEAxyzP)
%shared_ptr(chrono::fea::ChNodeFEAxyzD)
%shared_ptr(chrono::fea::ChNodeFEAxyzrot)
%shared_ptr(chrono::fea::ChMesh)
%shared_ptr(chrono::fea::ChContactSurface)
%shared_ptr(chrono::fea::ChContactSurfaceMesh)
%shared_ptr(chrono::fea::ChContactSurfaceNodeCloud)
%shared_ptr(chrono::fea::ChMeshSurface)
%shared_ptr(chrono::fea::ChVisulizationFEAmesh)
%shared_ptr(chrono::fea::ChLinkDirFrame)
%shared_ptr(chrono::fea::ChLinkPointFrame)
%shared_ptr(chrono::fea::ChLinkPointPoint)

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

%import  "ChClassFactory.i"
%import  "ChObject.i"
%import  "ChFrame.i"
%import  "ChFrameMoving.i"
%import  "ChPhysicsItem.i"
%import  "ChBodyFrame.i"
%import  "ChAsset.i"
%import  "ChLinkBase.i"

//  core/  classes

%include "../chrono_fea/ChElementBase.h"
%include "../chrono_fea/ChElementGeneric.h"
%include "../chrono_fea/ChBeamSection.h"
%include "../chrono_fea/ChElementBeam.h"
%include "../chrono_fea/ChElementBeamEuler.h"
%include "../chrono_fea/ChElementBeamANCF.h"
%import "../chrono/physics/ChContinuumMaterial.h"
%include "../chrono_fea/ChContinuumPoisson3D.h"
%include "../chrono_fea/ChContinuumElectrostatics.h"
%include "../chrono_fea/ChContinuumThermal.h"
//%include "../chrono_fea/ChElementTetrahedron.h"  	// pure virtual: do not create Python obj
%include "../chrono_fea/ChElementTetra_4.h"
%include "../chrono_fea/ChElementTetra_10.h"
//%include "../chrono_fea/ChElementHexahedron.h"		// pure virtual: do not create Python obj
%include "../chrono_fea/ChElementHexa_8.h"
%include "../chrono_fea/ChElementHexa_20.h"
%include "../chrono_fea/ChNodeFEAbase.h"
%include "../chrono_fea/ChNodeFEAxyz.h"
%include "../chrono_fea/ChNodeFEAxyzP.h"
%include "../chrono_fea/ChNodeFEAxyzD.h"
%include "../chrono_fea/ChNodeFEAxyzrot.h"
//%shared_ptr(chrono::fea::ChBuilderBeam)
%include "../chrono_fea/ChBuilderBeam.h"
%include "../chrono_fea/ChMesh.h"
%include "../chrono_fea/ChContactSurface.h"
%include "../chrono_fea/ChContactSurfaceMesh.h"
%include "../chrono_fea/ChContactSurfaceNodeCloud.h"
%include "../chrono_fea/ChMeshSurface.h"
%include "../chrono_fea/ChVisualizationFEAmesh.h"
%include "../chrono_fea/ChLinkDirFrame.h"
%include "../chrono_fea/ChLinkPointFrame.h"
%include "../chrono_fea/ChLinkPointPoint.h"

//
// C- DOWNCASTING OF SHARED POINTERS
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


