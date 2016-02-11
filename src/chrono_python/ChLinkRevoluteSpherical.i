%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkRevoluteSpherical.h"

%}
 
// Tell SWIG about parent class in Python
//%import "ChLink.i"



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChLinkRevoluteSpherical)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkRevoluteSpherical.h"  







