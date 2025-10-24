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
//  'import pychrono.fea'


%module(directors="1") fea


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

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/ChCollisionSystem.h"

#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"
#include "chrono/fea/ChNodeFEAxyzD.h"
#include "chrono/fea/ChNodeFEAxyzDD.h"
#include "chrono/fea/ChNodeFEAxyzDDD.h"
#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono/fea/ChElementBase.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChMaterialBeamANCF.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChElementHexaANCF_3813.h"
#include "chrono/fea/ChElementHexaANCF_3813_9.h"
#include "chrono/fea/ChElementShell.h"
#include "chrono/fea/ChMaterialShellReissner.h"
#include "chrono/fea/ChMaterialShellANCF.h"
#include "chrono/fea/ChMaterialShellKirchhoff.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3833.h"
#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChContinuumMaterial.h"
#include "chrono/fea/ChContinuumElectrostatics.h"
#include "chrono/fea/ChContinuumThermal.h"
#include "chrono/fea/ChContactSurface.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChMeshSurface.h"
#include "chrono/core/ChTensors.h"
#include "chrono/physics/ChLoad.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadsNodeXYZ.h"
#include "chrono/physics/ChLoader.h"
#include "chrono/physics/ChLoaderU.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoaderUVW.h"
#include "chrono/fea/ChLoadsBeam.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeNode.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLoadsNodeXYZRot.h"
#include "chrono/fea/ChLoadsBeam.h"
#include "Eigen/src/Core/util/Memory.h"

using namespace chrono;
using namespace chrono::fea;


%}

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define CH_DEPRECATED(msg)

// workaround for trouble
%ignore chrono::fea::ChContactNodeXYZ::ComputeJacobianForContactPart;
%ignore chrono::fea::ChContactTriangleXYZ::ComputeJacobianForContactPart;
%ignore chrono::fea::ChContactNodeXYZRot::ComputeJacobianForContactPart;
%ignore chrono::fea::ChContactTriangleXYZRot::ComputeJacobianForContactPart;
%ignore chrono::fea::ChElementShellBST::ComputeInternalJacobians;

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


%template(vector_ChNodeFEAxyzrot) std::vector< std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> >;
%template(vector_ChNodeFEAxyz)    std::vector< std::shared_ptr<chrono::fea::ChNodeFEAxyz> >;
%template(vector_ChNodeFEAxyzD)    std::vector< std::shared_ptr<chrono::fea::ChNodeFEAxyzD> >;
%template(vector_ChNodeFEAxyzDD)    std::vector< std::shared_ptr<chrono::fea::ChNodeFEAxyzDD> >;
%template(vector_ChNodeFEAxyzDDD)    std::vector< std::shared_ptr<chrono::fea::ChNodeFEAxyzDDD> >;
%template(vector_ChNodeFEAcurv)    std::vector< std::shared_ptr<chrono::fea::ChNodeFEAcurv> >;
%template(vector_ChElementBeamEuler)    std::vector< std::shared_ptr<chrono::fea::ChElementBeamEuler> >;
%template(vector_ChElementBeamIGA)    std::vector< std::shared_ptr<chrono::fea::ChElementBeamIGA> >;
%template(vector_ChElementCableANCF)    std::vector< std::shared_ptr<chrono::fea::ChElementCableANCF> >;
%template(vector_ChElementBeamANCF_3243)    std::vector< std::shared_ptr<chrono::fea::ChElementBeamANCF_3243> >;
%template(vector_ChElementBeamANCF_3333)    std::vector< std::shared_ptr<chrono::fea::ChElementBeamANCF_3333> >;

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
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChIndexedNodes)
%shared_ptr(chrono::ChLoadBase)
%shared_ptr(chrono::ChLoadCustom)
%shared_ptr(chrono::ChLoadCustomMultiple)
%shared_ptr(chrono::ChNodeBase) 
%shared_ptr(chrono::ChNodeXYZ) 

%shared_ptr(chrono::ChCollisionModel)

//from this module:
%shared_ptr(chrono::fea::ChContinuumMaterial)
%shared_ptr(chrono::fea::ChContinuumElastic)
%shared_ptr(chrono::fea::ChContinuumElastoplastic)
%shared_ptr(chrono::fea::ChContinuumPlasticVonMises)
%shared_ptr(chrono::fea::ChContinuumDruckerPrager)
%shared_ptr(chrono::fea::ChBeamSection)
%shared_ptr(chrono::fea::ChBeamSectionShape)
%shared_ptr(chrono::fea::ChBeamSectionShapeCircular)
%shared_ptr(chrono::fea::ChBeamSectionShapeRectangular)
%shared_ptr(chrono::fea::ChBeamSectionShapePolyline)
%shared_ptr(chrono::fea::ChBeamSectionEuler)
%shared_ptr(chrono::fea::ChBeamSectionEulerSimple)
%shared_ptr(chrono::fea::ChBeamSectionEulerAdvanced)
%shared_ptr(chrono::fea::ChBeamSectionEulerAdvancedGeneric)
%shared_ptr(chrono::fea::ChBeamSectionEulerEasyCircular)
%shared_ptr(chrono::fea::ChBeamSectionEulerEasyRectangular)
%shared_ptr(chrono::fea::ChBeamSectionRayleighSimple)
%shared_ptr(chrono::fea::ChBeamSectionRayleighAdvancedGeneric)
%shared_ptr(chrono::fea::ChBeamSectionRayleighEasyCircular)
%shared_ptr(chrono::fea::ChBeamSectionRayleighEasyRectangular)
%shared_ptr(chrono::fea::ChBeamSectionBasic)
%shared_ptr(chrono::fea::ChBeamSectionCable)
%shared_ptr(chrono::fea::ChBeamSectionAdvanced)
%shared_ptr(chrono::fea::ChBeamSectionCosserat)
%shared_ptr(chrono::fea::ChBeamSectionCosseratEasyCircular)
%shared_ptr(chrono::fea::ChBeamSectionCosseratEasyRectangular)
%shared_ptr(chrono::fea::ChElasticityCosserat)
%shared_ptr(chrono::fea::ChElasticityCosseratSimple)
%shared_ptr(chrono::fea::ChElasticityCosseratGeneric)
%shared_ptr(chrono::fea::ChElasticityCosseratAdvanced)
%shared_ptr(chrono::fea::ChElasticityCosseratAdvancedGeneric)
%shared_ptr(chrono::fea::ChElasticityCosseratAdvancedGenericFPM)
%shared_ptr(chrono::fea::ChElasticityCosseratMesh)
%shared_ptr(chrono::fea::ChPlasticityCosserat)
%shared_ptr(chrono::fea::ChPlasticityCosseratLumped)
%shared_ptr(chrono::fea::ChDampingCosserat)
%shared_ptr(chrono::fea::ChDampingCosseratLinear)
%shared_ptr(chrono::fea::ChDampingCosseratRayleigh)
%shared_ptr(chrono::fea::ChInertiaCosserat)
%shared_ptr(chrono::fea::ChInertiaCosseratUniformDensity)
%shared_ptr(chrono::fea::ChInertiaCosseratSimple)
%shared_ptr(chrono::fea::ChInertiaCosseratAdvanced)
%shared_ptr(chrono::fea::ChInertiaCosseratMassref)
%shared_ptr(chrono::fea::ChMaterialBeamANCF)
%shared_ptr(chrono::fea::ChElementBeam)
%shared_ptr(chrono::fea::ChElementBeamEuler)
%shared_ptr(chrono::fea::ChElementBeamANCF_3243)
%shared_ptr(chrono::fea::ChElementBeamANCF_3333)
%shared_ptr(chrono::fea::ChElementBeamIGA)
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
%shared_ptr(chrono::fea::ChElementBase)
%shared_ptr(chrono::fea::ChElementGeneric)
%shared_ptr(chrono::fea::ChElementSpring)
%shared_ptr(chrono::fea::ChElementBar)
%shared_ptr(chrono::fea::ChElementCorotational)
%shared_ptr(chrono::fea::ChElementTetrahedron)
%shared_ptr(chrono::fea::ChElementTetraCorot_4)
%shared_ptr(chrono::fea::ChElementTetraCorot_4_P)
%shared_ptr(chrono::fea::ChElementTetraCorot_10)
%shared_ptr(chrono::fea::ChElementHexahedron)
%shared_ptr(chrono::fea::ChElementHexaCorot_8)
%shared_ptr(chrono::fea::ChElementHexaCorot_20)
%shared_ptr(chrono::fea::ChElementHexaANCF_3813)
%shared_ptr(chrono::fea::ChElementHexaANCF_3813_9)
%shared_ptr(chrono::fea::ChNodeFEAbase)
%shared_ptr(chrono::fea::ChNodeFEAxyz)
%shared_ptr(chrono::fea::ChNodeFEAxyzP)
%shared_ptr(chrono::fea::ChNodeFEAxyzD)
%shared_ptr(chrono::fea::ChNodeFEAxyzDD)
%shared_ptr(chrono::fea::ChNodeFEAxyzDDD)
%shared_ptr(chrono::fea::ChNodeFEAxyzrot)
%shared_ptr(chrono::fea::ChMesh)
%shared_ptr(chrono::ChContactable_1vars)
%shared_ptr(chrono::ChContactable_2vars)
%shared_ptr(chrono::ChContactable_3vars)
%shared_ptr(chrono::fea::ChContactNodeXYZ)
%shared_ptr(chrono::fea::ChContactNodeXYZRot)
%shared_ptr(chrono::fea::ChContactTriangleXYZ)
%shared_ptr(chrono::fea::ChContactTriangleXYZRot)
%shared_ptr(chrono::fea::ChContactSurface)
%shared_ptr(chrono::fea::ChContactSurfaceMesh)
%shared_ptr(chrono::fea::ChContactSurfaceNodeCloud)
%shared_ptr(chrono::fea::ChMeshSurface)
%shared_ptr(chrono::fea::ChVisulizationFEAmesh)
%shared_ptr(chrono::fea::ChLinkNodeSlopeFrame)
%shared_ptr(chrono::fea::ChLinkNodeFrame)
%shared_ptr(chrono::fea::ChLinkNodeFrameGeneric)
%shared_ptr(chrono::fea::ChLinkNodeNode)
%shared_ptr(chrono::fea::ChMaterialShellANCF)
%shared_ptr(chrono::fea::ChMaterialShellReissner)
%shared_ptr(chrono::fea::ChMaterialShellReissnerIsothropic)
%shared_ptr(chrono::fea::ChMaterialShellReissnerOrthotropic)
%shared_ptr(chrono::fea::ChElasticityReissner)
%shared_ptr(chrono::fea::ChElasticityReissnerIsothropic)
%shared_ptr(chrono::fea::ChElasticityReissnerOrthotropic)
%shared_ptr(chrono::fea::ChElasticityReissnerGeneric)
%shared_ptr(chrono::fea::ChPlasticityReissner)
%shared_ptr(chrono::fea::ChDampingReissner)
%shared_ptr(chrono::fea::ChDampingReissnerRayleigh)
%shared_ptr(chrono::fea::ChMaterialShellKirchhoff)
%shared_ptr(chrono::fea::ChElasticityKirchhoff)
%shared_ptr(chrono::fea::ChElasticityKirchhoffIsothropic)
%shared_ptr(chrono::fea::ChElasticityKirchhoffOrthotropic)
%shared_ptr(chrono::fea::ChElasticityKirchhoffGeneric)
%shared_ptr(chrono::fea::ChPlasticityKirchhoff)
%shared_ptr(chrono::fea::ChDampingKirchhoff)
%shared_ptr(chrono::fea::ChDampingKirchhoffRayleigh)
%shared_ptr(chrono::fea::ChElementShell)
%shared_ptr(chrono::fea::ChElementShellReissner4)
%shared_ptr(chrono::fea::ChElementShellANCF_3423)
%shared_ptr(chrono::fea::ChElementShellANCF_3833)
%shared_ptr(chrono::fea::ChElementShellBST)
%shared_ptr(chrono::fea::ChElementCableANCF)
%shared_ptr(chrono::fea::ChBuilderBeamEuler)
%shared_ptr(chrono::fea::ChBuilderBeamIGA)
%shared_ptr(chrono::fea::ChBuilderCableANCF)
%shared_ptr(chrono::fea::ChLoaderBeamWrench)
%shared_ptr(chrono::fea::ChLoadBeamWrench)
%shared_ptr(chrono::fea::ChLoaderBeamWrenchDistributed)
%shared_ptr(chrono::fea::ChLoadBeamWrenchDistributed)
%shared_ptr(chrono::fea::ChExtruderBeamEuler)
%shared_ptr(chrono::fea::ChExtruderBeamIGA)
%shared_ptr(chrono::fea::ChLoadNodeXYZRot)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotForceAbs)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotNodeXYZRot)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotNodeXYZRotBushingSpherical)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotNodeXYZRotBushingPlastic)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotNodeXYZRotBushingMate)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotNodeXYZRotBushingGeneric)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotBody)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotBodyBushingSpherical)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotBodyBushingPlastic)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotBodyBushingMate)
%shared_ptr(chrono::fea::ChLoadNodeXYZRotBodyBushingGeneric)

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

%import(module = "pychrono.core")  "chrono_swig/interface/core/ChClassFactory.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChObject.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVector2.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChVector3.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChQuaternion.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChMatrix.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChMassProperties.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChPhysicsItem.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChCoordsys.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChFrameMoving.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChBodyFrame.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLinkBase.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChTensors.i"
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") chrono::ChFunction;
%import(module = "pychrono.core")  "../../../chrono/functions/ChFunction.h"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChColor.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChColormap.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChContactMaterial.i"
%import(module = "pychrono.core")  "../../../chrono/physics/ChPhysicsItem.h"
%import(module = "pychrono.core")  "../../../chrono/physics/ChIndexedNodes.h"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLoadable.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLoader.i"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChLoad.i"
%import(module = "pychrono.core")  "../../../chrono/physics/ChNodeBase.h"
%import(module = "pychrono.core")  "../../../chrono/physics/ChNodeXYZ.h"
%import(module = "pychrono.core")  "chrono_swig/interface/core/ChContactContainer.i"

//  core/  classes
%include "../../../chrono/physics/ChPhysicsItem.h"

%include "../../../chrono/collision/ChCollisionModel.h"
%include "../../../chrono/collision/ChCollisionSystem.h"

%include "../../../chrono/physics/ChContactable.h"  

%include "../../../chrono/fea/ChContinuumMaterial.h"
// TODO: if eigen::ref can be wrapped, unignore these,
%ignore chrono::fea::ChNodeFEAbase::ComputeKRMmatricesGlobal;
%include "../../../chrono/fea/ChNodeFEAbase.h"
%include "../../../chrono/fea/ChNodeFEAxyz.h"
%include "../../../chrono/fea/ChNodeFEAxyzP.h"
%include "../../../chrono/fea/ChNodeFEAxyzD.h"
%include "../../../chrono/fea/ChNodeFEAxyzDD.h"
%include "../../../chrono/fea/ChNodeFEAxyzDDD.h"
%include "../../../chrono/fea/ChNodeFEAxyzrot.h"
// TODO: if eigen::ref can be wrapped, unignore these
%ignore chrono::fea::ChElementBase::ComputeKRMmatricesGlobal;
%ignore chrono::fea::ChElementBase::ComputeMmatrixGlobal;
%include "../../../chrono/fea/ChElementBase.h"
%include "../../../chrono/fea/ChElementGeneric.h"
%include "../../../chrono/fea/ChElementBar.h"
%include "../../../chrono/fea/ChElementSpring.h"
%include "../../../chrono/fea/ChElementCorotational.h"
%include "../../../chrono/fea/ChBeamSectionShape.h"
%include "../../../chrono/fea/ChBeamSection.h"
%include "../../../chrono/fea/ChBeamSectionCosserat.h"
%include "../../../chrono/fea/ChBeamSectionEuler.h"
%include "../../../chrono/fea/ChBeamSectionCable.h"
%include "../../../chrono/fea/ChMaterialBeamANCF.h"
%include "../../../chrono/fea/ChElementBeam.h"
%include "../../../chrono/fea/ChElementBeamEuler.h"
%feature("notabstract") chrono::fea::ChElementBeamANCF_3243;
%include "../../../chrono/fea/ChElementBeamANCF_3243.h"
%include "../../../chrono/fea/ChElementBeamANCF_3333.h"
%include "../../../chrono/fea/ChElementBeamIGA.h"
%include "../../../chrono/fea/ChContinuumPoisson3D.h"
%include "../../../chrono/fea/ChContinuumElectrostatics.h"
%include "../../../chrono/fea/ChContinuumThermal.h"
%include "../../../chrono/fea/ChElementTetrahedron.h"  	
%include "../../../chrono/fea/ChElementTetraCorot_4.h"
%include "../../../chrono/fea/ChElementTetraCorot_10.h"
%include "../../../chrono/fea/ChElementHexahedron.h"		
%include "../../../chrono/fea/ChElementHexaCorot_8.h"
%include "../../../chrono/fea/ChElementHexaCorot_20.h"
%include "../../../chrono/fea/ChElementHexaANCF_3813.h"
%include "../../../chrono/fea/ChElementHexaANCF_3813_9.h"
%include "../../../chrono/fea/ChMaterialShellANCF.h"
// TODO: if eigen::ref can be wrapped, unignore these,
%ignore chrono::fea::ChElasticityReissner::ComputeStiffnessMatrix;
%ignore chrono::fea::ChPlasticityReissner::ComputeStiffnessMatrixElastoplastic;
%ignore chrono::fea::ChDampingReissner::ComputeDampingMatrix;
%ignore chrono::fea::ChDampingReissnerRayleigh::ComputeDampingMatrix;
%ignore chrono::fea::ChMaterialShellReissner::ComputeStiffnessMatrix;
%include "../../../chrono/fea/ChMaterialShellReissner.h"
// TODO: if eigen::ref can be wrapped, unignore these,
%ignore chrono::fea::ChElasticityKirchhoff::ComputeStiffnessMatrix;
%ignore chrono::fea::ChPlasticityKirchhoff::ComputeStiffnessMatrixElastoplastic;
%ignore chrono::fea::ChDampingKirchhoff::ComputeDampingMatrix;
%ignore chrono::fea::ChDampingKirchhoffReissner::ComputeDampingMatrix;
%ignore chrono::fea::ChMaterialShellKirchhoff::ComputeStiffnessMatrix;
%include "../../../chrono/fea/ChMaterialShellKirchhoff.h"
%include "../../../chrono/fea/ChElementShell.h"
%rename(ShellReissner4Layer) chrono::fea::ChElementShellReissner4::Layer;
%include "../../../chrono/fea/ChElementShellReissner4.h"
%rename(ShellANCF) chrono::fea::ChElementShellANCF_3423::Layer;
%include "../../../chrono/fea/ChElementShellANCF_3423.h"
%rename(ShellANCF_8Layer) chrono::fea::ChElementShellANCF_3833::Layer;
%include "../../../chrono/fea/ChElementShellANCF_3833.h"
%rename(ShellBSTLayer) chrono::fea::ChElementShellBST::Layer;
%include "../../../chrono/fea/ChElementShellBST.h"
%include "../../../chrono/fea/ChElementCableANCF.h"
%include "../../../chrono/fea/ChContactSurface.h"
%include "../../../chrono/fea/ChContactSurfaceMesh.h"
%include "../../../chrono/fea/ChContactSurfaceNodeCloud.h"
%template(vector_ChNodeFEAbase) std::vector< std::shared_ptr<chrono::fea::ChNodeFEAbase> >;
%template(vector_ChElementBase) std::vector< std::shared_ptr<chrono::fea::ChElementBase> >;
%import "../../../chrono/fea/ChMeshSurface.h" // should be already provided by ChModuleCore
%include "../../../chrono/fea/ChMesh.h"
%include "../../../chrono/fea/ChLinkNodeSlopeFrame.h"
%include "../../../chrono/fea/ChLinkNodeFrame.h"
%include "../../../chrono/fea/ChLinkNodeNode.h"
%include "../../../chrono/fea/ChLoadsBeam.h"
%include "../../../chrono/fea/ChBuilderBeam.h"
%include "../../../chrono/fea/ChMeshFileLoader.h"
%include "../../../chrono/fea/ChLoadsNodeXYZRot.h"
%include "../../../chrono/fea/ChLoadsBeam.h"

//
// C- CASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. 
// Alternatively, in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefSharedPtrDynamicCast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChPhysicsItem,ChMesh)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityCosserat,ChElasticityCosseratSimple)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityCosserat,ChElasticityCosseratGeneric)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityCosserat,ChElasticityCosseratAdvanced)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityCosserat,ChElasticityCosseratMesh)
%DefSharedPtrDynamicCast(chrono::fea,ChPlasticityCosserat,ChPlasticityCosseratLumped)
%DefSharedPtrDynamicCast(chrono::fea,ChInertiaCosserat,ChInertiaCosseratSimple)
%DefSharedPtrDynamicCast(chrono::fea,ChInertiaCosserat,ChInertiaCosseratAdvanced)
%DefSharedPtrDynamicCast(chrono::fea,ChInertiaCosserat,ChInertiaCosseratMassref)
%DefSharedPtrDynamicCast(chrono::fea,ChDampingCosserat,ChDampingCosseratLinear)
%DefSharedPtrDynamicCast(chrono::fea,ChDampingCosserat,ChDampingCosseratRayleigh)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityReissner,ChElasticityReissnerIsothropic)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityReissner,ChElasticityReissnerOrthotropic)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityReissner,ChElasticityReissnerGeneric)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityKirchhoff,ChElasticityKirchhoffIsothropic)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityKirchhoff,ChElasticityKirchhoffOrthotropic)
%DefSharedPtrDynamicCast(chrono::fea,ChElasticityKirchhoff,ChElasticityKirchhoffGeneric)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementBar)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementSpring)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementTetraCorot_4)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementTetraCorot_4_P)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementTetraCorot_10)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementHexaCorot_8)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementHexaCorot_20)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementBeamEuler)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementBeamANCF_3243)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementBeamANCF_3333)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementBeamIGA)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementCableANCF)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementShellReissner4)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementShellANCF_3423)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementShellANCF_3833)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementHexaANCF_3813)
%DefSharedPtrDynamicCast(chrono::fea,ChElementBase,ChElementHexaANCF_3813_9)
%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChNodeBase,ChNodeFEAbase)
%DefSharedPtrDynamicCast(chrono::fea,ChNodeFEAbase,ChNodeFEAxyz)
%DefSharedPtrDynamicCast(chrono::fea,ChNodeFEAbase,ChNodeFEAxyzP)
%DefSharedPtrDynamicCast(chrono::fea,ChNodeFEAbase,ChNodeFEAxyzD)
%DefSharedPtrDynamicCast(chrono::fea,ChNodeFEAbase,ChNodeFEAxyzDD)
%DefSharedPtrDynamicCast(chrono::fea,ChNodeFEAbase,ChNodeFEAxyzDDD)
%DefSharedPtrDynamicCast(chrono::fea,ChNodeFEAbase,ChNodeFEAxyzrot)
%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChContactable,ChContactTriangleXYZ)
%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChContactable,ChContactTriangleXYZRot)
%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChContactable,ChContactNodeXYZ)
%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChContactable,ChContactNodeXYZRot)
//%DefSharedPtrDynamicCast2NS(chrono,chrono::fea,ChContactable,ChNodeMeshless)

//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

%inline %{
  chrono::fea::ChContactNodeXYZRot* CastContactableToChContactNodeXYZRot(chrono::ChContactable* base) {
    chrono::fea::ChContactNodeXYZRot* ptr_out = dynamic_cast<chrono::fea::ChContactNodeXYZRot*>(base);
	if (ptr_out == NULL) {
        throw std::invalid_argument( "Wrong Upcast Choice" );
    }
    return ptr_out;
  }
%}
%inline %{
  chrono::fea::ChContactNodeXYZ* CastContactableToChContactNodeXYZ(chrono::ChContactable* base) {
    chrono::fea::ChContactNodeXYZ* ptr_out = dynamic_cast<chrono::fea::ChContactNodeXYZ*>(base);
	if (ptr_out == NULL) {
        throw std::invalid_argument( "Wrong Upcast Choice" );
    }
    return ptr_out;
  }
%}
%inline %{
  chrono::fea::ChContactTriangleXYZRot* CastContactableToChContactTriangleXYZRot(chrono::ChContactable* base) {
    chrono::fea::ChContactTriangleXYZRot* ptr_out = dynamic_cast<chrono::fea::ChContactTriangleXYZRot*>(base);
	if (ptr_out == NULL) {
        throw std::invalid_argument( "Wrong Upcast Choice" );
    }
    return ptr_out;
  }
%}
%inline %{
  chrono::fea::ChContactTriangleXYZ* CastContactableToChContactTriangleXYZ(chrono::ChContactable* base) {
    chrono::fea::ChContactTriangleXYZ* ptr_out = dynamic_cast<chrono::fea::ChContactTriangleXYZ*>(base);
	if (ptr_out == NULL) {
        throw std::invalid_argument( "Wrong Upcast Choice" );
    }
    return ptr_out;
  }
%}

%extend chrono::fea::ChLoadNodeXYZRotNodeXYZRotBushingGeneric{
		public:
			ChLoadNodeXYZRotNodeXYZRotBushingGeneric(
                    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> mnodeA,  ///< node A
                    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> mnodeB,  ///< node B
                    const ChFrame<>& abs_application,
                    chrono::ChMatrixDynamic<double> mstiffness,
                    chrono::ChMatrixDynamic<double> mdamping){

               chrono::fea::ChLoadNodeXYZRotNodeXYZRotBushingGeneric *selfpoint = new ChLoadNodeXYZRotNodeXYZRotBushingGeneric(mnodeA, mnodeB, abs_application, mstiffness, mdamping);
			   return selfpoint;
			   }
		};

%extend chrono::fea::ChLoadNodeXYZRotBodyBushingGeneric{
		public:
			ChLoadNodeXYZRotBodyBushingGeneric(
                    std::shared_ptr<chrono::fea::ChNodeFEAxyzrot> mnodeA,  ///< node A
                    std::shared_ptr<chrono::ChBody> mnodeB,  ///< node B
                    const ChFrame<>& abs_application,
                    chrono::ChMatrixDynamic<double> mstiffness,
                    chrono::ChMatrixDynamic<double> mdamping){
			   
			   chrono::fea::ChLoadNodeXYZRotBodyBushingGeneric *selfpoint = new ChLoadNodeXYZRotBodyBushingGeneric(mnodeA, mnodeB, abs_application, mstiffness, mdamping);

			   return selfpoint;
			   }
		};

//
// ADD PYTHON CODE
//

/*
%pythoncode %{

%}
*/


