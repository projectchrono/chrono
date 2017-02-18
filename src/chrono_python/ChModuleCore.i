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
//  'import ChronoEngine_python_core as chrono'


%module(directors="1") ChronoEngine_python_core


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
#include <typeindex>
#include <cstddef>
#include "chrono/core/ChApiCE.h"
#include "chrono/physics/ChBody.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::geometry;
%}


// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


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

%shared_ptr(chrono::ChAsset)
%shared_ptr(chrono::ChVisualization)
//%shared_ptr(chrono::ChColor)
%shared_ptr(chrono::ChColorAsset)
%shared_ptr(chrono::ChAssetLevel)
%shared_ptr(chrono::ChObjShapeFile)
%shared_ptr(chrono::ChBoxShape) 
%shared_ptr(chrono::ChSphereShape)
%shared_ptr(chrono::ChCylinderShape)
%shared_ptr(chrono::ChTexture)
%shared_ptr(chrono::ChCamera) 

%shared_ptr(chrono::ChFunction)  
%shared_ptr(chrono::ChFunction_Const)
%shared_ptr(chrono::ChFunction_ConstAcc)
%shared_ptr(chrono::ChFunction_Derive)
%shared_ptr(chrono::ChFunction_Fillet3)
%shared_ptr(chrono::ChFunction_Integrate)
%shared_ptr(chrono::ChFunction_Mirror)
%shared_ptr(chrono::ChFunction_Mocap)
%shared_ptr(chrono::ChFunction_Noise)
%shared_ptr(chrono::ChFunction_Operation)
%shared_ptr(chrono::ChFunction_Oscilloscope)
%shared_ptr(chrono::ChFunction_Poly)
%shared_ptr(chrono::ChFunction_Poly345)
%shared_ptr(chrono::ChFunction_Ramp)
%shared_ptr(chrono::ChFunction_Recorder)
%shared_ptr(chrono::ChFunction_Repeat)
%shared_ptr(chrono::ChFunction_Sequence)
%shared_ptr(chrono::ChFunction_Sigma)
%shared_ptr(chrono::ChFunction_Sine)

%shared_ptr(chrono::ChObj)
%shared_ptr(chrono::collision::ChCollisionModel)
%shared_ptr(chrono::ChPhysicsItem)
%shared_ptr(chrono::ChMaterialSurface)
%shared_ptr(chrono::ChMaterialSurfaceBase)
%shared_ptr(chrono::ChBodyFrame)
%shared_ptr(chrono::ChMarker)
%shared_ptr(chrono::ChForce)
%shared_ptr(chrono::ChBody)
%shared_ptr(chrono::ChBodyAuxRef)
%shared_ptr(chrono::ChConveyor)
%shared_ptr(chrono::ChAparticle)
%shared_ptr(chrono::ChParticleBase)
%shared_ptr(chrono::ChIndexedParticles)
%shared_ptr(chrono::ChParticlesClones)
%shared_ptr(chrono::ChAssembly)
%shared_ptr(chrono::ChTimestepper)
%shared_ptr(chrono::ChTimestepperIorder)
%shared_ptr(chrono::ChTimestepperIIorder)
%shared_ptr(chrono::ChTimestepperEulerExpl)
%shared_ptr(chrono::ChTimestepperEulerExplIIorder)
%shared_ptr(chrono::ChTimestepperEulerSemiImplicit)
%shared_ptr(chrono::ChTimestepperRungeKuttaExpl)
%shared_ptr(chrono::ChTimestepperHeun)
%shared_ptr(chrono::ChTimestepperLeapfrog)
%shared_ptr(chrono::ChTimestepperEulerImplicit)
%shared_ptr(chrono::ChTimestepperEulerImplicitLinearized)
%shared_ptr(chrono::ChTimestepperEulerImplicitProjected)
%shared_ptr(chrono::ChTimestepperTrapezoidalLinearized)
%shared_ptr(chrono::ChTimestepperTrapezoidalLinearized2)
%shared_ptr(chrono::ChTimestepperTrapezoidal)
%shared_ptr(chrono::ChTimestepperNewmark)
%shared_ptr(chrono::ChImplicitIterativeTimestepper)
%shared_ptr(chrono::ChImplicitTimestepper)
%shared_ptr(chrono::ChSolver)
%shared_ptr(chrono::ChSystem)
%shared_ptr(chrono::ChContactContainerBase)
%shared_ptr(chrono::ChProximityContainerBase)

%shared_ptr(chrono::ChLinkBase)
%shared_ptr(chrono::ChLink)
%shared_ptr(chrono::ChLinkMarkers)
%shared_ptr(chrono::ChLinkMasked)
%shared_ptr(chrono::ChLinkLimit)
%shared_ptr(chrono::ChLinkLock)
%shared_ptr(chrono::ChLinkLockRevolute)
%shared_ptr(chrono::ChLinkLockLock)
%shared_ptr(chrono::ChLinkLockSpherical)
%shared_ptr(chrono::ChLinkLockCylindrical)
%shared_ptr(chrono::ChLinkLockPrismatic)
%shared_ptr(chrono::ChLinkLockPointPlane)
%shared_ptr(chrono::ChLinkLockPointLine)
%shared_ptr(chrono::ChLinkLockPlanePlane)
%shared_ptr(chrono::ChLinkLockOldham)
%shared_ptr(chrono::ChLinkLockFree)
%shared_ptr(chrono::ChLinkLockAlign)
%shared_ptr(chrono::ChLinkLockParallel)
%shared_ptr(chrono::ChLinkLockPerpend)
%shared_ptr(chrono::ChLinkLockRevolutePrismatic)
%shared_ptr(chrono::ChLinkDistance)
%shared_ptr(chrono::ChLinkEngine)
%shared_ptr(chrono::ChLinkGear)
%shared_ptr(chrono::ChLinkLinActuator)
%shared_ptr(chrono::ChLinkMate)
%shared_ptr(chrono::ChLinkMateGeneric)
%shared_ptr(chrono::ChLinkMatePlane)
%shared_ptr(chrono::ChLinkMateCoaxial)
%shared_ptr(chrono::ChLinkMateSpherical)
%shared_ptr(chrono::ChLinkMateXdistance)
%shared_ptr(chrono::ChLinkMateParallel)
%shared_ptr(chrono::ChLinkMateOrthogonal)
%shared_ptr(chrono::ChLinkMateFix)
%shared_ptr(chrono::ChLinkPulley)
%shared_ptr(chrono::ChLinkRevolute)
%shared_ptr(chrono::ChLinkRevoluteSpherical)
%shared_ptr(chrono::ChLinkScrew)
%shared_ptr(chrono::ChLinkSpring)
%shared_ptr(chrono::ChLinkUniversal)

%shared_ptr(chrono::ChShaft)
%shared_ptr(chrono::ChShaftsBody)
%shared_ptr(chrono::ChShaftsClutch)
%shared_ptr(chrono::ChShaftsCouple)
%shared_ptr(chrono::ChShaftsGear)
%shared_ptr(chrono::ChShaftsMotor)
%shared_ptr(chrono::ChShaftsPlanetary)
%shared_ptr(chrono::ChShaftsThermalEngine)
%shared_ptr(chrono::ChShaftsTorqueBase)
%shared_ptr(chrono::ChShaftsTorsionSpring)



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
%include "ChException.i"
%include "ChClassFactory.i"
//%include "ChArchive.i"
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
%include "ChVectorDynamic.i"
%include "ChTimer.i"
%include "ChRealtimeStep.i"
%include "ChTransform.i"

// motion_functions/   classes
%include "ChFunction_Base.i"

// geometry/   classes
%include "ChGeometry.i"

%include "ChCollisionModel.i"

// assets
%include "ChAsset.i"
%include "ChColor.i"
%include "ChVisualization.i"
%include "ChColorAsset.i"
%include "ChAssetLevel.i"
%include "ChObjShapeFile.i"
%include "ChBoxShape.i"
%include "ChSphereShape.i"
%include "ChCylinderShape.i"
%include "ChTexture.i"
%include "ChCamera.i"

// physics/  classes
%include "ChObject.i"
%include "ChPhysicsItem.i"
%include "ChMaterialSurfaceBase.i"
%include "ChMaterialSurface.i"
%include "ChMaterialCouple.i"
%include "ChBodyFrame.i"
%include "ChMarker.i"
%include "ChForce.i"
%include "ChBody.i"
%include "ChBodyAuxRef.i"
%include "ChConveyor.i"
%include "ChIndexedParticles.i"
%include "ChParticlesClones.i"
%include "ChAssembly.i"
%include "../chrono/timestepper/ChTimestepper.h"
%include "../chrono/solver/ChSolver.h"
%include "ChSystem.i"
%include "ChContactContainerBase.i"
%include "ChProximityContainerBase.i"
%include "ChLinkBase.i"
%include "ChLink.i"
%include "ChLinkMarkers.i"
%include "ChLinkMasked.i"
%include "ChLinkLimit.i"
%include "ChLinkForce.i"
%include "ChLinkLock.i"
%include "ChLinkEngine.i"
%include "ChLinkMate.i"
%include "ChLinkDistance.i"
%include "ChLinkLinActuator.i"
%include "ChLinkPulley.i"
%include "ChLinkScrew.i"
%include "ChLinkSpring.i"
%include "ChLinkGear.i"
%include "ChLinkRevolute.i"
%include "ChLinkRevoluteSpherical.i"
%include "ChLinkUniversal.i"

%include "ChShaft.i"
%include "ChShaftsCouple.i"
%include "ChShaftsBody.i"
%include "ChShaftsClutch.i"
%include "ChShaftsMotor.i"
%include "ChShaftsTorsionSpring.i"
%include "ChShaftsPlanetary.i"
%include "ChShaftsTorqueBase.i"
%include "ChShaftsThermalEngine.i"


// collision/   classes
/*
%include "ChCollisionInfo.i"
*/




//
// C- DOWNCASTING OF SHARED POINTERS
// 
// This is not automatic in Python + SWIG, except if one uses the 
// %downcast_output_sharedptr(...) macro, as above, but this causes
// a lot of code bloat. So in the following we create a set of Python-side
// functions to perform casting by hand, thank to the macro 
// %DefChSharedPtrDynamicDowncast(base,derived). 
// Do not specify the "chrono::" namespace before base or derived!
// Later, in python, you can do the following:
//  myvis = chrono.CastToChVisualizationShared(myasset)
//  print ('Could be cast to visualization object?', !myvis.IsNull())

// enable _automatic_ downcasting from ChAsset to derived classes (shared pointers versions)
%downcast_output_sharedptr(chrono::ChAsset, chrono::ChVisualization, chrono::ChObjShapeFile, chrono::ChBoxShape, chrono::ChSphereShape, chrono::ChCylinderShape, chrono::ChTexture, chrono::ChAssetLevel, chrono::ChCamera, chrono::ChColorAsset)

%DefChSharedPtrDynamicDowncast(ChAsset,ChVisualization)
%DefChSharedPtrDynamicDowncast(ChAsset,ChObjShapeFile)
%DefChSharedPtrDynamicDowncast(ChAsset,ChBoxShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChSphereShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChCylinderShape)
%DefChSharedPtrDynamicDowncast(ChAsset,ChTexture)
%DefChSharedPtrDynamicDowncast(ChAsset,ChAssetLevel)
%DefChSharedPtrDynamicDowncast(ChBodyFrame, ChBody)
%DefChSharedPtrDynamicDowncast(ChBodyFrame, ChBodyAuxRef)
%DefChSharedPtrDynamicDowncast(ChBodyFrame, ChConveyor)
%DefChSharedPtrDynamicDowncast(ChBody, ChBodyFrame)  // <- upcast, for testing & workaround
%DefChSharedPtrDynamicDowncast(ChBodyAuxRef, ChBodyFrame)  // <- upcast, for testing & workaround
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChBody)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChConveyor)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChBodyAuxRef)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChIndexedParticles)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChParticlesClones)
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
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkMateFix)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkGear)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkDistance)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkLinActuator)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkPulley)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkScrew)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChLinkSpring)

%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Const)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_ConstAcc)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Derive)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Fillet3)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Integrate)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Mirror)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Mocap)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Noise)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Operation)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Oscilloscope)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Poly)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Poly345)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Ramp)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Recorder)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Repeat)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Sequence)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Sigma)
%DefChSharedPtrDynamicDowncast(ChFunction, ChFunction_Sine)

%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaft)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsBody)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsCouple)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsClutch)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsMotor)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsTorsionSpring)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsPlanetary)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsTorqueBase)
%DefChSharedPtrDynamicDowncast(ChPhysicsItem, ChShaftsThermalEngine)

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
	virtual void	Output(const char* data, size_t n) 
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


%pythoncode %{

def ImportSolidWorksSystem(mpath):
    import builtins
    import imp
    import os

    mdirname, mmodulename= os.path.split(mpath)

    builtins.exported_system_relpath = mdirname + "/"

    fp, pathname, description = imp.find_module(mmodulename,[builtins.exported_system_relpath])
    try:
        imported_mod = imp.load_module('imported_mod', fp, pathname, description)
    finally:
        if fp:
            fp.close()

    return imported_mod.exported_items

%}


//  
//%shared_ptr(chrono::ChSystem)
