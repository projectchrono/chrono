%{

/* Includes the header in the wrapper code */
#include "physics/ChLinkMasked.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChLinkMarkers.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Define also the shared pointer
%shared_ptr(chrono::ChLinkMasked)

/* Parse the header file(s) to generate wrappers */
%include "../chrono/physics/ChLinkForce.h"
%include "../chrono/physics/ChLinkMasked.h"







