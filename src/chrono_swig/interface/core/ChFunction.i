%{
#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_BSpline.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/motion_functions/ChFunction_ConstAcc.h"
#include "chrono/motion_functions/ChFunction_Cycloidal.h"
#include "chrono/motion_functions/ChFunction_Derive.h"
#include "chrono/motion_functions/ChFunction_DoubleS.h"
#include "chrono/motion_functions/ChFunction_Fillet3.h"
#include "chrono/motion_functions/ChFunction_Integrate.h"
#include "chrono/motion_functions/ChFunction_Mirror.h"
#include "chrono/motion_functions/ChFunction_Mocap.h"
#include "chrono/motion_functions/ChFunction_Noise.h"
#include "chrono/motion_functions/ChFunction_Operation.h"
#include "chrono/motion_functions/ChFunction_Oscilloscope.h"
#include "chrono/motion_functions/ChFunction_Poly.h"
#include "chrono/motion_functions/ChFunction_Poly345.h"
#include "chrono/motion_functions/ChFunction_Ramp.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Repeat.h"
#include "chrono/motion_functions/ChFunction_Sequence.h"
#include "chrono/motion_functions/ChFunction_Sigma.h"
#include "chrono/motion_functions/ChFunction_Sine.h"
#include "chrono/motion_functions/ChFunction_Setpoint.h"

#include "chrono/motion_functions/ChFunctionRotation.h"
#include "chrono/motion_functions/ChFunctionRotation_axis.h"
#include "chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
#include "chrono/motion_functions/ChFunctionRotation_setpoint.h"
#include "chrono/motion_functions/ChFunctionRotation_spline.h"
#include "chrono/motion_functions/ChFunctionRotation_SQUAD.h"
#include "chrono/motion_functions/ChFunctionPosition.h"
#include "chrono/motion_functions/ChFunctionPosition_line.h"
#include "chrono/motion_functions/ChFunctionPosition_setpoint.h"
#include "chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

// Helper function that will be put in C++ wrapper and that will
// be later used by %typemap in order to do downcasting to proper Python
// class when a method returns a generic pointer to base ChFunction*
SWIGRUNTIME PyObject* DowncastChFunction(chrono::ChFunction* out)
{
  if (out)
  {
		if      ( typeid(*out)==typeid(chrono::ChFunction_BSpline) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_BSpline, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Const) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Const, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_ConstAcc) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_ConstAcc, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Cycloidal) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Cycloidal, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Derive) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Derive, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Fillet3) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Fillet3, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Integrate) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Integrate, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Mirror) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Mirror, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Mocap) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Mocap, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Noise) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Noise, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Operation) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Operation, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Oscilloscope) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Oscilloscope, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Poly) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Poly, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Poly345) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Poly345, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Ramp) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Ramp, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Recorder) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Recorder, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Repeat) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Repeat, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Sequence) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Sequence, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Sigma) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Sigma, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_Sine) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Sine, 0 |  0 );
		else
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction, 0 |  0 );
   } 
   else
	return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction, 0 |  0 );
}

#endif             // --------------------------------------------------------------------- PYTHON

%}

%shared_ptr(chrono::ChFunction)  
%shared_ptr(chrono::ChFunction_BSpline)
%shared_ptr(chrono::ChFunction_Const)
%shared_ptr(chrono::ChFunction_ConstAcc)
%shared_ptr(chrono::ChFunction_Cycloidal)
%shared_ptr(chrono::ChFunction_Derive)
%shared_ptr(chrono::ChFunction_DoubleS)
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
%shared_ptr(chrono::ChFunction_Setpoint)
%shared_ptr(chrono::ChFunction_SetpointCallback)

%shared_ptr(chrono::ChFunctionRotation)
%shared_ptr(chrono::ChFunctionRotation_axis)
%shared_ptr(chrono::ChFunctionRotation_ABCfunctions)
%shared_ptr(chrono::ChFunctionRotation_setpoint)
%shared_ptr(chrono::ChFunctionRotation_spline)
%shared_ptr(chrono::ChFunctionRotation_SQUAD)
%shared_ptr(chrono::ChFunctionPosition)
%shared_ptr(chrono::ChFunctionPosition_line)
%shared_ptr(chrono::ChFunctionPosition_setpoint)
%shared_ptr(chrono::ChFunctionPosition_XYZfunctions)


// Cross-inheritance for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") chrono::ChFunction;
%feature("director") chrono::ChFunction_Setpoint;
%feature("director") chrono::ChFunction_SetpointCallback;
%feature("director") chrono::ChFunctionPosition;
%feature("director") chrono::ChFunctionPosition_setpoint;
%feature("director") chrono::ChFunctionRotation;
%feature("director") chrono::ChFunctionRotation_setpoint;
%ignore chrono::ChFunction::Clone;
%ignore chrono::ChFunctionPosition::Clone;
%ignore chrono::ChFunctionRotation::Clone;

// Parse the header file to generate wrappers
%include "../../../chrono/motion_functions/ChFunction_Base.h" 
%include "../../../chrono/motion_functions/ChFunction_BSpline.h" 
%include "../../../chrono/motion_functions/ChFunction_Const.h"
%include "../../../chrono/motion_functions/ChFunction_ConstAcc.h"
%include "../../../chrono/motion_functions/ChFunction_Cycloidal.h"
%include "../../../chrono/motion_functions/ChFunction_Derive.h"
%include "../../../chrono/motion_functions/ChFunction_DoubleS.h"
%include "../../../chrono/motion_functions/ChFunction_Fillet3.h"
%include "../../../chrono/motion_functions/ChFunction_Integrate.h"
%include "../../../chrono/motion_functions/ChFunction_Mirror.h"
%include "../../../chrono/motion_functions/ChFunction_Mocap.h"
%include "../../../chrono/motion_functions/ChFunction_Noise.h"
%include "../../../chrono/motion_functions/ChFunction_Operation.h"
%include "../../../chrono/motion_functions/ChFunction_Oscilloscope.h"
%include "../../../chrono/motion_functions/ChFunction_Poly.h"
%include "../../../chrono/motion_functions/ChFunction_Poly345.h"
%include "../../../chrono/motion_functions/ChFunction_Ramp.h"
%include "../../../chrono/motion_functions/ChFunction_Recorder.h"
%include "../../../chrono/motion_functions/ChFunction_Repeat.h"
%include "../../../chrono/motion_functions/ChFunction_Sequence.h"
%include "../../../chrono/motion_functions/ChFunction_Sigma.h"
%include "../../../chrono/motion_functions/ChFunction_Sine.h"
%include "../../../chrono/motion_functions/ChFunction_Setpoint.h"

%include "../../../chrono/motion_functions/ChFunctionRotation.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_axis.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_setpoint.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_spline.h"
%include "../../../chrono/motion_functions/ChFunctionRotation_SQUAD.h"
%include "../../../chrono/motion_functions/ChFunctionPosition.h"
%include "../../../chrono/motion_functions/ChFunctionPosition_line.h"
%include "../../../chrono/motion_functions/ChFunctionPosition_setpoint.h"
%include "../../../chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"


//... so use the following custom trick

%typemap(out) chrono::ChFunction* {
	$result=DowncastChFunction($1);
}


%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_BSpline)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Const)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_ConstAcc)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Cycloidal)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Derive)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_DoubleS)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Fillet3)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Integrate)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Mirror)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Mocap)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Noise)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Operation)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Oscilloscope)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Poly)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Poly345)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Ramp)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Recorder)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Repeat)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Sequence)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Sigma)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Sine)
%DefSharedPtrDynamicDowncast(chrono, ChFunction, ChFunction_Setpoint)
