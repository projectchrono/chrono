%{

/* Includes the header in the wrapper code */
#include "physics/ChLink.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChPhysicsItem.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

// Enable shared pointer 
%shared_ptr(chrono::ChLink)

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChLink.h"  







