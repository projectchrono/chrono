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
//  'import ChronoEngine_PYTHON_core as chrono'


%module(directors="1") ChronoEngine_PYTHON_core


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

#include "physics/ChBody.h"

using namespace chrono;


%}


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

//  core/  classes
%include "ChException.i"
%include "ChHashFunction.i"
%include "ChVector.i" 
#define Vector ChVector<double>
%include "ChQuaternion.i"
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
%include "ChTimer.i"
%include "ChRealtimeStep.i"
%include "ChTrasform.i"
%include "ChShared.i"

// motion_functions/   classes
%include "ChFunction_Base.i"

// geometry/   classes
%include "ChGeometry.i"

%include "ChCollisionModel.i"

// assets
%include "ChAsset.i"
%include "ChVisualization.i"
%include "ChObjShapeFile.i"
%include "ChBoxShape.i"
%include "ChSphereShape.i"
%include "ChCylinderShape.i"
%include "ChTexture.i"
%include "ChCamera.i"
%include "ChAssetLevel.i"
  // enable _automatic_ downcasting from ChAsset to derived classes (shared pointers versions)
%downcast_output_sharedptr(chrono::ChAsset, chrono::ChVisualization, chrono::ChObjShapeFile, chrono::ChBoxShape, chrono::ChSphereShape, chrono::ChCylinderShape, chrono::ChTexture, chrono::ChAssetLevel, chrono::ChCamera)

// physics/  classes
%include "ChObject.i"
%include "ChPhysicsItem.i"
%include "ChMaterialSurface.i"
%include "ChMaterialCouple.i"
%include "ChBody.i"
%include "ChBodyAuxRef.i"
%include "ChConveyor.i"
%include "ChIndexedParticles.i"
%include "ChParticlesClones.i"
%include "ChMarker.i"
%include "ChForce.i"
%include "ChSystem.i"
%include "ChContactContainerBase.i"
%include "ChProximityContainerBase.i"
%include "ChLink.i"
%include "ChLinkMarkers.i"
%include "ChLinkMasked.i"
%include "ChLinkLock.i"
%include "ChLinkEngine.i"
%include "ChLinkMate.i"
%include "ChLinkDistance.i"
%include "ChLinkLinActuator.i"
%include "ChLinkPulley.i"
%include "ChLinkScrew.i"
%include "ChLinkSpring.i"
%include "ChLinkGear.i"
/*
%include "ChShaft.i"
%include "ChShaftsCouple.i"
%include "ChShaftsBody.i"
%include "ChShaftsClutch.i"
%include "ChShaftsMotor.i"
%include "ChShaftsTorsionSpring.i"
%include "ChShaftsPlanetary.i"
*/

// collision/   classes

%include "ChCollisionInfo.i"





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


%DefChSharedPtrCast(chrono::ChBody, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChConveyor, chrono::ChBody)
%DefChSharedPtrCast(chrono::ChBodyAuxRef, chrono::ChBody)
%DefChSharedPtrCast(chrono::ChIndexedParticles, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChParticlesClones, chrono::ChIndexedParticles)
%DefChSharedPtrCast(chrono::ChMarker, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChForce, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChVisualization, chrono::ChAsset)
%DefChSharedPtrCast(chrono::ChSphereShape, chrono::ChVisualization)
%DefChSharedPtrCast(chrono::ChCylinderShape, chrono::ChVisualization)
%DefChSharedPtrCast(chrono::ChBoxShape, chrono::ChVisualization)
%DefChSharedPtrCast(chrono::ChObjShapeFile, chrono::ChVisualization)
%DefChSharedPtrCast(chrono::ChTexture, chrono::ChAsset)
%DefChSharedPtrCast(chrono::ChCamera, chrono::ChAsset)
%DefChSharedPtrCast(chrono::ChAssetLevel, chrono::ChAsset)
%DefChSharedPtrCast(chrono::ChLink, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChLinkMarkers, chrono::ChLink)
%DefChSharedPtrCast(chrono::ChLinkMasked, chrono::ChLinkMarkers)
%DefChSharedPtrCast(chrono::ChLinkLock, chrono::ChLinkMasked)
%DefChSharedPtrCast(chrono::ChLinkLockLock, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockRevolute, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockSpherical, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockCylindrical, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockPrismatic, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockPointPlane, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockPointLine, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockOldham, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockFree, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockAlign, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockHook, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockParallel, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLockPerpend, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkEngine, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkMate, chrono::ChLink)
%DefChSharedPtrCast(chrono::ChLinkMateGeneric, chrono::ChLinkMate)
%DefChSharedPtrCast(chrono::ChLinkMatePlane, chrono::ChLinkMateGeneric)
%DefChSharedPtrCast(chrono::ChLinkMateCoaxial, chrono::ChLinkMateGeneric)
%DefChSharedPtrCast(chrono::ChLinkMateSpherical, chrono::ChLinkMateGeneric)
%DefChSharedPtrCast(chrono::ChLinkMateXdistance, chrono::ChLinkMateGeneric)
%DefChSharedPtrCast(chrono::ChLinkMateParallel, chrono::ChLinkMateGeneric)
%DefChSharedPtrCast(chrono::ChLinkMateOrthogonal, chrono::ChLinkMateGeneric)
%DefChSharedPtrCast(chrono::ChLinkGear, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkDistance, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkLinActuator, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkPulley, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkScrew, chrono::ChLinkLock)
%DefChSharedPtrCast(chrono::ChLinkSpring, chrono::ChLinkMarkers)
/*
%DefChSharedPtrCast(chrono::ChShaft, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChShaftsBody, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChShaftsCouple, chrono::ChPhysicsItem)
%DefChSharedPtrCast(chrono::ChShaftsClutch, chrono::ChShaftsCouple)
%DefChSharedPtrCast(chrono::ChShaftsMotor, chrono::ChShaftsCouple)
%DefChSharedPtrCast(chrono::ChShaftsTorsionSpring, chrono::ChShaftsCouple)
%DefChSharedPtrCast(chrono::ChShaftsPlanetary, chrono::ChPhysicsItem)
*/


//
// DOWNCASTING OF POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. So in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefChSharedPtrDynamicDowncast(base,derived). Do not specify the chrono:: namespace.
// In python, use as:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

%DefChSharedPtrDynamicDowncast(ChAsset,ChVisualization)
%DefChSharedPtrDynamicDowncast(ChAsset,ChObjShapeFile)
%DefChSharedPtrDynamicDowncast(ChAsset,ChBoxShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChSphereShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChTexture)
%DefChSharedPtrDynamicDowncast(ChAsset,ChAssetLevel)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChBody)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChConveyor)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChBodyAuxRef)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChIndexedParticles)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChParticlesClones)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChMarker)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChForce)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChVisualization)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChSphereShape)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChBoxShape)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChObjShapeFile)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLink)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMarkers)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMasked)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLock)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockLock)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockRevolute)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockSpherical)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockCylindrical)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPrismatic)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPointPlane)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPointLine)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockOldham)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockFree)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockAlign)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockHook)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockParallel)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLockPerpend)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkEngine)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMate)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateGeneric)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMatePlane)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateCoaxial)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateSpherical)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateXdistance)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateParallel)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateOrthogonal)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkGear)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkDistance)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLinActuator)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkPulley)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkScrew)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkSpring)
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
	virtual void	Output(const char* data, int n) 
		{ 
				char buffer[1000];
				if (n>999) 
					n=999;
				strncpy(buffer, data, n);
				buffer[n]=0;
				PySys_WriteStdout(buffer);
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

/*
%pythoncode %{

%}
*/


