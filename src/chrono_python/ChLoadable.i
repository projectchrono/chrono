%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChLoadable.h"

%}
 
%shared_ptr(chrono::ChLoadable)
%shared_ptr(chrono::ChLoadableU)
%shared_ptr(chrono::ChLoadableUV)
%shared_ptr(chrono::ChLoadableUVW)


// Tell SWIG about parent class in Python



/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLoadable.h"  







