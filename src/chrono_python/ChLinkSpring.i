%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkSpring.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer
%shared_ptr(chrono::ChLinkSpring)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLinkSpring.h"  







