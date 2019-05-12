%{

/* Includes the header in the wrapper code */
#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/motion_functions/ChFunction_ConstAcc.h"
#include "chrono/motion_functions/ChFunction_Derive.h"
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

	// Helper function that will be put in C++ wrapper and that will
	// be later used by %typemap in order to do downcasting to proper Python
	// class when a method returns a generic pointer to base ChFunction*
SWIGRUNTIME PyObject* DowncastChFunction(chrono::ChFunction* out)
{
  if (out)
  {
		if      ( typeid(*out)==typeid(chrono::ChFunction_Const) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_Const, 0 |  0 );
		else if ( typeid(*out)==typeid(chrono::ChFunction_ConstAcc) )
			return SWIG_NewPointerObj(SWIG_as_voidptr(out), SWIGTYPE_p_chrono__ChFunction_ConstAcc, 0 |  0 );
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


%}


// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") chrono::ChFunction;
%ignore chrono::ChFunction::Clone;

/* Parse the header file to generate wrappers */
%include "../chrono/motion_functions/ChFunction_Base.h"  
%include "../chrono/motion_functions/ChFunction_Const.h"
%include "../chrono/motion_functions/ChFunction_ConstAcc.h"
%include "../chrono/motion_functions/ChFunction_Derive.h"
%include "../chrono/motion_functions/ChFunction_Fillet3.h"
%include "../chrono/motion_functions/ChFunction_Integrate.h"
%include "../chrono/motion_functions/ChFunction_Mirror.h"
%include "../chrono/motion_functions/ChFunction_Mocap.h"
%include "../chrono/motion_functions/ChFunction_Noise.h"
%include "../chrono/motion_functions/ChFunction_Operation.h"
%include "../chrono/motion_functions/ChFunction_Oscilloscope.h"
%include "../chrono/motion_functions/ChFunction_Poly.h"
%include "../chrono/motion_functions/ChFunction_Poly345.h"
%include "../chrono/motion_functions/ChFunction_Ramp.h"
%include "../chrono/motion_functions/ChFunction_Recorder.h"
%include "../chrono/motion_functions/ChFunction_Repeat.h"
%include "../chrono/motion_functions/ChFunction_Sequence.h"
%include "../chrono/motion_functions/ChFunction_Sigma.h"
%include "../chrono/motion_functions/ChFunction_Sine.h"

// The following is clean but may cause code bloat...

// %downcast_output (chrono::ChFunction, chrono::ChFunction_Sine, chrono::ChFunction_Const)


//... so use the following custom trick

%typemap(out) chrono::ChFunction* {
	$result=DowncastChFunction($1);
}


