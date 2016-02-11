%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkGear.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkLock.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer 
%shared_ptr(chrono::ChLinkGear)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkGear.h"  







