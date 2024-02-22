%{
#include "chrono/motion_functions/ChFunctionBase.h"
#include "chrono/motion_functions/ChFunctionBSpline.h"
#include "chrono/motion_functions/ChFunctionConst.h"
#include "chrono/motion_functions/ChFunctionConstAcc.h"
#include "chrono/motion_functions/ChFunctionConstJerk.h"
#include "chrono/motion_functions/ChFunctionCycloidal.h"
#include "chrono/motion_functions/ChFunctionDerive.h"
#include "chrono/motion_functions/ChFunctionFillet3.h"
#include "chrono/motion_functions/ChFunctionIntegrate.h"
#include "chrono/motion_functions/ChFunctionMirror.h"
#include "chrono/motion_functions/ChFunctionOperation.h"
#include "chrono/motion_functions/ChFunctionPoly.h"
#include "chrono/motion_functions/ChFunctionPoly345.h"
#include "chrono/motion_functions/ChFunctionRamp.h"
#include "chrono/motion_functions/ChFunctionInterp.h"
#include "chrono/motion_functions/ChFunctionRepeat.h"
#include "chrono/motion_functions/ChFunctionSequence.h"
#include "chrono/motion_functions/ChFunctionPoly23.h"
#include "chrono/motion_functions/ChFunctionSine.h"
#include "chrono/motion_functions/ChFunctionSineStep.h"
#include "chrono/motion_functions/ChFunctionSetpoint.h"
#include "chrono/motion_functions/ChFunctionRotation.h"
#include "chrono/motion_functions/ChFunctionRotationAxis.h"
#include "chrono/motion_functions/ChFunctionRotationABCFunctions.h"
#include "chrono/motion_functions/ChFunctionRotationSetpoint.h"
#include "chrono/motion_functions/ChFunctionRotationBSpline.h"
#include "chrono/motion_functions/ChFunctionRotationSQUAD.h"
#include "chrono/motion_functions/ChFunctionPosition.h"
#include "chrono/motion_functions/ChFunctionPositionLine.h"
#include "chrono/motion_functions/ChFunctionPositionSetpoint.h"
#include "chrono/motion_functions/ChFunctionPositionXYZFunctions.h"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

// Helper function that will be put in C++ wrapper and that will
// be later used by %typemap in order to do downcasting to proper Python
// class when a method returns a generic pointer to base ChFunction*
SWIGRUNTIME PyObject* DowncastChFunction(chrono::ChFunction* out)
{
  if (out)
  {
		if      ( typeid(*out)==typeid(chrono::ChFunctionBSpline) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionBSpline, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionConst) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionConst, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionConstAcc) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionConstAcc, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionConstJerk) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionConstJerk, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionCycloidal) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionCycloidal, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionDerive) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionDerive, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionFillet3) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionFillet3, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionIntegrate) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionIntegrate, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionMirror) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionMirror, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionOperation) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionOperation, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionPoly) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionPoly, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionPoly23) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionPoly23, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionPoly345) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionPoly345, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionRamp) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionRamp, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionInterp) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionInterp, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionRepeat) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionRepeat, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionSequence) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionSequence, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionSine) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionSine, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionSineStep) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionSineStep, 0 |  0 );
		else
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction, 0 |  0 );
   } 
   else
	return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction, 0 |  0 );
}

#endif             // --------------------------------------------------------------------- PYTHON

%}

%shared_ptr(chrono::ChFunction)  
%shared_ptr(chrono::ChFunctionBSpline)
%shared_ptr(chrono::ChFunctionConst)
%shared_ptr(chrono::ChFunctionConstAcc)
%shared_ptr(chrono::ChFunctionConstJerk)
%shared_ptr(chrono::ChFunctionCycloidal)
%shared_ptr(chrono::ChFunctionDerive)
%shared_ptr(chrono::ChFunctionFillet3)
%shared_ptr(chrono::ChFunctionIntegrate)
%shared_ptr(chrono::ChFunctionMirror)
%shared_ptr(chrono::ChFunctionOperation)
%shared_ptr(chrono::ChFunctionPoly)
%shared_ptr(chrono::ChFunctionPoly345)
%shared_ptr(chrono::ChFunctionRamp)
%shared_ptr(chrono::ChFunctionInterp)
%shared_ptr(chrono::ChFunctionRepeat)
%shared_ptr(chrono::ChFunctionSequence)
%shared_ptr(chrono::ChFunctionPoly23)
%shared_ptr(chrono::ChFunctionSine)
%shared_ptr(chrono::ChFunctionSineStep)
%shared_ptr(chrono::ChFunctionSetpoint)
%shared_ptr(chrono::ChFunctionSetpointCallback)

%shared_ptr(chrono::ChFunctionRotation)
%shared_ptr(chrono::ChFunctionRotationAxis)
%shared_ptr(chrono::ChFunctionRotationABCFunctions)
%shared_ptr(chrono::ChFunctionRotationSetpoint)
%shared_ptr(chrono::ChFunctionRotationBSpline)
%shared_ptr(chrono::ChFunctionRotationSQUAD)
%shared_ptr(chrono::ChFunctionPosition)
%shared_ptr(chrono::ChFunctionPositionLine)
%shared_ptr(chrono::ChFunctionPositionSetpoint)
%shared_ptr(chrono::ChFunctionPositionXYZFunctions)


// Director classes generates C# code with calls to the C# methods .GetType()
// thus clashing with ChFunction::GetType() method. We need to rename the C++ method GetType() to GetFunctionType()
#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP
%rename(GetFunctionType) chrono::ChFunction::GetType() const;  // rename for all derived classes as well
#endif             // --------------------------------------------------------------------- CSHARP


// Cross-inheritance for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") chrono::ChFunction;
%feature("director") chrono::ChFunctionSetpoint;
%feature("director") chrono::ChFunctionSetpointCallback;
%feature("director") chrono::ChFunctionPosition;
%feature("director") chrono::ChFunctionPositionSetpoint;
%feature("director") chrono::ChFunctionRotation;
%feature("director") chrono::ChFunctionRotationSetpoint;
%ignore chrono::ChFunction::Clone;
%ignore chrono::ChFunctionPosition::Clone;
%ignore chrono::ChFunctionRotation::Clone;



// Parse the header file to generate wrappers
%include "../../../chrono/motion_functions/ChFunctionBase.h" 
%include "../../../chrono/motion_functions/ChFunctionBSpline.h" 
%include "../../../chrono/motion_functions/ChFunctionConst.h"
%include "../../../chrono/motion_functions/ChFunctionConstAcc.h"
%include "../../../chrono/motion_functions/ChFunctionConstJerk.h"
%include "../../../chrono/motion_functions/ChFunctionCycloidal.h"
%include "../../../chrono/motion_functions/ChFunctionDerive.h"
%include "../../../chrono/motion_functions/ChFunctionFillet3.h"
%include "../../../chrono/motion_functions/ChFunctionIntegrate.h"
%include "../../../chrono/motion_functions/ChFunctionInterp.h"
%include "../../../chrono/motion_functions/ChFunctionMirror.h"
%include "../../../chrono/motion_functions/ChFunctionOperation.h"
%include "../../../chrono/motion_functions/ChFunctionPoly.h"
%include "../../../chrono/motion_functions/ChFunctionPoly23.h"
%include "../../../chrono/motion_functions/ChFunctionPoly345.h"
%include "../../../chrono/motion_functions/ChFunctionRamp.h"
%include "../../../chrono/motion_functions/ChFunctionRepeat.h"
%include "../../../chrono/motion_functions/ChFunctionSequence.h"
%include "../../../chrono/motion_functions/ChFunctionSetpoint.h"
%include "../../../chrono/motion_functions/ChFunctionSine.h"
%include "../../../chrono/motion_functions/ChFunctionSineStep.h"

%include "../../../chrono/motion_functions/ChFunctionPosition.h"
%include "../../../chrono/motion_functions/ChFunctionPositionLine.h"
%include "../../../chrono/motion_functions/ChFunctionPositionSetpoint.h"
%include "../../../chrono/motion_functions/ChFunctionPositionXYZFunctions.h"
%include "../../../chrono/motion_functions/ChFunctionRotation.h"
%include "../../../chrono/motion_functions/ChFunctionRotationABCFunctions.h"
%include "../../../chrono/motion_functions/ChFunctionRotationAxis.h"
%include "../../../chrono/motion_functions/ChFunctionRotationBSpline.h"
%include "../../../chrono/motion_functions/ChFunctionRotationSetpoint.h"
%include "../../../chrono/motion_functions/ChFunctionRotationSQUAD.h"


//... so use the following custom trick

%typemap(out) chrono::ChFunction* {
	$result=DowncastChFunction($1);
}


%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionBSpline)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionConst)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionConstAcc)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionConstJerk)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionCycloidal)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionDerive)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionFillet3)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionIntegrate)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionMirror)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionOperation)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionPoly)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionPoly345)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionRamp)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionInterp)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionRepeat)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionSequence)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionPoly23)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionSine)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionSineStep)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunctionSetpoint)
