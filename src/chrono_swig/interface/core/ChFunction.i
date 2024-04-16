%{
#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionBSpline.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionConstAcc.h"
#include "chrono/functions/ChFunctionConstJerk.h"
#include "chrono/functions/ChFunctionCycloidal.h"
#include "chrono/functions/ChFunctionDerivative.h"
#include "chrono/functions/ChFunctionFillet3.h"
#include "chrono/functions/ChFunctionIntegral.h"
#include "chrono/functions/ChFunctionMirror.h"
#include "chrono/functions/ChFunctionOperator.h"
#include "chrono/functions/ChFunctionPoly.h"
#include "chrono/functions/ChFunctionPoly345.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/functions/ChFunctionInterp.h"
#include "chrono/functions/ChFunctionRepeat.h"
#include "chrono/functions/ChFunctionSequence.h"
#include "chrono/functions/ChFunctionPoly23.h"
#include "chrono/functions/ChFunctionSine.h"
#include "chrono/functions/ChFunctionSineStep.h"
#include "chrono/functions/ChFunctionSetpoint.h"
#include "chrono/functions/ChFunctionRotation.h"
#include "chrono/functions/ChFunctionRotationAxis.h"
#include "chrono/functions/ChFunctionRotationABCFunctions.h"
#include "chrono/functions/ChFunctionRotationSetpoint.h"
#include "chrono/functions/ChFunctionRotationBSpline.h"
#include "chrono/functions/ChFunctionRotationSQUAD.h"
#include "chrono/functions/ChFunctionPosition.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionPositionSetpoint.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"

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
		else if ( typeid(*out)==typeid(chrono::ChFunctionDerivative) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionDerivative, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionFillet3) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionFillet3, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionIntegral) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionIntegral, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionMirror) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionMirror, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunctionOperator) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunctionOperator, 0 |  0 );
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
%shared_ptr(chrono::ChFunctionDerivative)
%shared_ptr(chrono::ChFunctionFillet3)
%shared_ptr(chrono::ChFunctionIntegral)
%shared_ptr(chrono::ChFunctionMirror)
%shared_ptr(chrono::ChFunctionOperator)
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
%include "../../../chrono/functions/ChFunctionBase.h" 
%include "../../../chrono/functions/ChFunctionBSpline.h" 
%include "../../../chrono/functions/ChFunctionConst.h"
%include "../../../chrono/functions/ChFunctionConstAcc.h"
%include "../../../chrono/functions/ChFunctionConstJerk.h"
%include "../../../chrono/functions/ChFunctionCycloidal.h"
%include "../../../chrono/functions/ChFunctionDerivative.h"
%include "../../../chrono/functions/ChFunctionFillet3.h"
%include "../../../chrono/functions/ChFunctionIntegral.h"
%include "../../../chrono/functions/ChFunctionInterp.h"
%include "../../../chrono/functions/ChFunctionMirror.h"
%include "../../../chrono/functions/ChFunctionOperator.h"
%include "../../../chrono/functions/ChFunctionPoly.h"
%include "../../../chrono/functions/ChFunctionPoly23.h"
%include "../../../chrono/functions/ChFunctionPoly345.h"
%include "../../../chrono/functions/ChFunctionRamp.h"
%include "../../../chrono/functions/ChFunctionRepeat.h"
%include "../../../chrono/functions/ChFunctionSequence.h"
%include "../../../chrono/functions/ChFunctionSetpoint.h"
%include "../../../chrono/functions/ChFunctionSine.h"
%include "../../../chrono/functions/ChFunctionSineStep.h"

%include "../../../chrono/functions/ChFunctionPosition.h"
%include "../../../chrono/functions/ChFunctionPositionLine.h"
%include "../../../chrono/functions/ChFunctionPositionSetpoint.h"
%include "../../../chrono/functions/ChFunctionPositionXYZFunctions.h"
%include "../../../chrono/functions/ChFunctionRotation.h"
%include "../../../chrono/functions/ChFunctionRotationABCFunctions.h"
%include "../../../chrono/functions/ChFunctionRotationAxis.h"
%include "../../../chrono/functions/ChFunctionRotationBSpline.h"
%include "../../../chrono/functions/ChFunctionRotationSetpoint.h"
%include "../../../chrono/functions/ChFunctionRotationSQUAD.h"


//... so use the following custom trick

%typemap(out) chrono::ChFunction* {
	$result=DowncastChFunction($1);
}


%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionBSpline)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionConst)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionConstAcc)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionConstJerk)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionCycloidal)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionDerivative)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionFillet3)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionIntegral)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionMirror)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionOperator)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionPoly)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionPoly345)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionRamp)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionInterp)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionRepeat)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionSequence)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionPoly23)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionSine)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionSineStep)
%DefSharedPtrDynamicCast(chrono, ChFunction, ChFunctionSetpoint)
