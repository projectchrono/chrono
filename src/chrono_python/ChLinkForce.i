%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkForce.h"

%}
 
// Tell SWIG about parent class in Python
//%import "..."



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer 
%shared_ptr(chrono::ChLinkLimit)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkForce.h"







