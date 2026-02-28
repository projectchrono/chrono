%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLoadable.h"

%}

#ifdef SWIGCSHARP

// %interface_impl(chrono::ChLoadable);
%ignore chrono::ChLoadable;
%interface_impl(chrono::ChLoadableU);
%interface_impl(chrono::ChLoadableUV);
%interface_impl(chrono::ChLoadableUVW);

#endif

%feature("director") chrono::ChLoadable;
%feature("director") chrono::ChLoadableU;
%feature("director") chrono::ChLoadableUV;
%feature("director") chrono::ChLoadableUVW;

%shared_ptr(chrono::ChLoadable)
%shared_ptr(chrono::ChLoadableU)
%shared_ptr(chrono::ChLoadableUV)
%shared_ptr(chrono::ChLoadableUVW)


// Tell SWIG about parent class in Python

/* Parse the header file to generate wrappers */
%include "../../../chrono/physics/ChLoadable.h"  






