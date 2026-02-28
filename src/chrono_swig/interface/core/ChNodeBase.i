%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChNodeBase.h"

%}

#ifdef SWIGCSHARP

%interface_impl(chrono::ChNodeBase);

#endif

%feature("director") chrono::ChNodeBase;

%shared_ptr(chrono::ChNodeBase)

// Tell SWIG about parent class in Python

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChNodeBase.h"  






